#include <assert.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_sleep.h"
#include "esp_pm.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "remote_config.h"
#include "mesh_link.h"
#include "jq8900.h"

typedef enum {
    AUDIO_OFF = 0,
    AUDIO_POWERUP = 1,
    AUDIO_WAIT_BUSY = 2,
    AUDIO_PLAYING = 3,
} audio_state_t;

typedef enum {
    ALERT_NONE = 0,
    ALERT_REAR = 1,
    ALERT_SOS = 2,
} alert_kind_t;

typedef enum {
    ALERT_COMPLETED = 0,
    ALERT_STOPPED = 1,
    ALERT_PREEMPTED = 2,
    ALERT_FAULT = 15,
} alert_result_t;

static QueueHandle_t commands;
static nvs_handle_t storage;
static esp_pm_lock_handle_t unprovisioned_lock;
static bool lamp, pending_sos, button_down, audio_fault;
static bool audio_uart_active;
static bool unprovisioned_lock_held;
static uint32_t sos_id, last_rear_command_id, alert_event_id;
static uint8_t audio_track;
static audio_state_t audio_state;
static alert_kind_t alert_kind;
static int64_t audio_deadline, audio_started, alert_deadline;
static int64_t next_status, next_sos;
static int button_stable = 1, button_raw = 1;
static int64_t button_changed;
static volatile bool button_irq_latched;
static volatile bool button_wakeup_armed;

static int64_t now_ms(void) { return esp_timer_get_time() / 1000; }

static void save_u32(const char *key, uint32_t value)
{
    ESP_ERROR_CHECK(nvs_set_u32(storage, key, value));
    ESP_ERROR_CHECK(nvs_commit(storage));
}

static void set_lamp(bool on)
{
    lamp = on;
    ESP_ERROR_CHECK(gpio_set_level(LAMP_GPIO,
                                   on ? LAMP_ACTIVE_LEVEL : !LAMP_ACTIVE_LEVEL));
}

static void IRAM_ATTR sos_isr(void *arg)
{
    (void)arg;

    // GPIO wakeup on ESP32-C3 is level-triggered. Mask it immediately so a
    // held button cannot flood the queue/CPU before the task debounces it.
    gpio_intr_disable(SOS_GPIO);
    button_wakeup_armed = false;
    if (button_irq_latched) return;
    button_irq_latched = true;

    remote_command_t wake = {0};
    BaseType_t higher_priority_task_woken = pdFALSE;
    if (xQueueSendFromISR(commands, &wake,
                          &higher_priority_task_woken) != pdTRUE) {
        button_irq_latched = false;
    }
    if (higher_priority_task_woken) portYIELD_FROM_ISR();
}

static void command_received(const remote_command_t *command)
{
    if (xQueueSend(commands, command, 0) != pdTRUE) {
        ESP_LOGW("remote", "Command queue full; gateway must retry");
    }
}

static bool jq_busy(void)
{
    return audio_state != AUDIO_OFF &&
           gpio_get_level(JQ_BUSY_GPIO) == JQ_BUSY_ACTIVE_LEVEL;
}

static uint8_t status_flags(void)
{
    return (audio_state != AUDIO_OFF ? 1 : 0) |
           (pending_sos ? 2 : 0) |
           (lamp ? 4 : 0) |
           (jq_busy() ? 8 : 0) |
           (mesh_link_friend_ready() ? 16 : 0) |
           (audio_fault ? 32 : 0) |
           (alert_kind == ALERT_REAR ? 64 : 0) |
           (alert_kind == ALERT_SOS ? 128 : 0);
}

static uint8_t status_detail(void)
{
    return ((uint8_t)alert_kind << 4) | (uint8_t)audio_state;
}

static void send_event_detail(uint8_t event, uint32_t id, uint8_t detail)
{
    esp_err_t err = mesh_link_send(event, id, status_flags(), detail);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGW("remote", "Event %u not sent: %s", event, esp_err_to_name(err));
    }
}

static void send_event(uint8_t event, uint32_t id)
{
    send_event_detail(event, id, status_detail());
}

static void audio_power_off(bool send_stop)
{
    if (send_stop && audio_uart_active) {
        esp_err_t err = jq8900_stop_playback();
        if (err != ESP_OK) ESP_LOGW("audio", "STOP failed: %s", esp_err_to_name(err));
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    if (audio_uart_active) {
        jq8900_uart_stop();
        audio_uart_active = false;
    }

    // Avoid back-powering JQ8900 through UART when its 5 V load switch is off.
    gpio_reset_pin(JQ_TX_GPIO);
    gpio_reset_pin(JQ_RX_GPIO);
    gpio_set_direction(JQ_TX_GPIO, GPIO_MODE_INPUT);
    gpio_set_pull_mode(JQ_TX_GPIO, GPIO_FLOATING);
    gpio_set_pull_mode(JQ_RX_GPIO, GPIO_FLOATING);
    ESP_ERROR_CHECK(gpio_set_level(JQ_POWER_GPIO, !JQ_POWER_ACTIVE_LEVEL));
    audio_state = AUDIO_OFF;
}

static void end_alert(alert_result_t result)
{
    if (alert_kind == ALERT_NONE) return;

    alert_kind_t completed_kind = alert_kind;
    uint32_t completed_id = alert_event_id;
    bool module_may_still_play = audio_state != AUDIO_OFF &&
                                 (audio_state != AUDIO_PLAYING || jq_busy());
    audio_power_off(module_may_still_play);
    alert_kind = ALERT_NONE;
    alert_deadline = 0;
    audio_fault = result == ALERT_FAULT;

    uint8_t detail = ((uint8_t)completed_kind << 4) | (uint8_t)result;
    send_event_detail(EVENT_ALERT_DONE, completed_id, detail);
    next_status = now_ms();
    ESP_LOGI("audio", "Alert done kind=%u id=%lu result=%u",
             (unsigned)completed_kind, (unsigned long)completed_id,
             (unsigned)result);
}

static void begin_alert(alert_kind_t kind, uint32_t event_id,
                        uint8_t track, int64_t duration_ms)
{
    audio_fault = false;
    alert_kind = kind;
    alert_event_id = event_id;
    audio_track = track;
    alert_deadline = duration_ms > 0 ? now_ms() + duration_ms : 0;
    ESP_ERROR_CHECK(gpio_set_level(JQ_POWER_GPIO, JQ_POWER_ACTIVE_LEVEL));
    audio_state = AUDIO_POWERUP;
    audio_deadline = now_ms() + JQ_POWERUP_MS;
    next_status = now_ms();
    ESP_LOGI("audio", "Power on; kind=%u track=%u id=%lu",
             (unsigned)kind, track, (unsigned long)event_id);
}

static bool play_current_track(void)
{
    esp_err_t err = jq8900_prepare_and_play(audio_track, JQ_VOLUME);
    if (err != ESP_OK) {
        ESP_LOGE("audio", "Unable to play JQ8900 track: %s", esp_err_to_name(err));
        end_alert(ALERT_FAULT);
        return false;
    }
    audio_state = AUDIO_WAIT_BUSY;
    audio_deadline = now_ms() + JQ_BUSY_START_TIMEOUT_MS;
    ESP_LOGI("audio", "PLAY track=%u; waiting for BUSY", audio_track);
    return true;
}

static void service_audio(int64_t now)
{
    if (alert_kind == ALERT_REAR && now >= alert_deadline) {
        end_alert(ALERT_COMPLETED);
        return;
    }

    if (audio_state == AUDIO_POWERUP && now >= audio_deadline) {
        esp_err_t err = jq8900_uart_start();
        if (err != ESP_OK) {
            ESP_LOGE("audio", "Unable to start JQ8900 UART: %s", esp_err_to_name(err));
            end_alert(ALERT_FAULT);
            return;
        }
        audio_uart_active = true;
        if (!play_current_track()) return;
        now = now_ms();
    }

    if (audio_state == AUDIO_WAIT_BUSY) {
        if (jq_busy()) {
            audio_state = AUDIO_PLAYING;
            audio_started = now;
            audio_deadline = now + JQ_TRACK_TIMEOUT_MS;
            ESP_LOGI("audio", "BUSY active; playback started");
        } else if (now >= audio_deadline) {
            ESP_LOGE("audio", "BUSY did not become active");
            end_alert(ALERT_FAULT);
        }
    } else if (audio_state == AUDIO_PLAYING) {
        if (!jq_busy() && now - audio_started >= 100) {
            if (alert_kind == ALERT_REAR && now < alert_deadline) {
                ESP_LOGI("audio", "Rear track ended; replaying until 3-minute deadline");
                play_current_track();
            } else {
                end_alert(ALERT_COMPLETED);
            }
        } else if (now >= audio_deadline) {
            ESP_LOGE("audio", "Track exceeded safety timeout");
            end_alert(ALERT_FAULT);
        }
    }
}

static void start_rear_alert(uint32_t id)
{
    if (alert_kind == ALERT_REAR) end_alert(ALERT_STOPPED);
    begin_alert(ALERT_REAR, id, JQ_REAR_TRACK, REAR_ALERT_MS);
}

static void start_sos_alert(void)
{
    if (alert_kind == ALERT_REAR) {
        end_alert(ALERT_PREEMPTED);
    } else if (alert_kind == ALERT_SOS) {
        // A second physical press restarts the same local SOS track.
        audio_power_off(true);
        alert_kind = ALERT_NONE;
    }
    begin_alert(ALERT_SOS, sos_id, JQ_SOS_TRACK, 0);
}

static void handle_command(const remote_command_t *command)
{
    switch (command->type) {
    case CMD_REAR_CONFIRM:
        if (command->id == 0 || command->id <= last_rear_command_id) {
            send_event(EVENT_STATUS, command->id);
            break;
        }
        if (pending_sos || alert_kind == ALERT_SOS) {
            ESP_LOGW("remote", "REAR rejected while SOS has priority");
            send_event(EVENT_STATUS, command->id);
            break;
        }
        last_rear_command_id = command->id;
        save_u32("rear_id", last_rear_command_id);
        start_rear_alert(command->id);
        send_event(EVENT_STATUS, command->id);
        break;

    case CMD_SOS_ACK:
        if (pending_sos && command->id == sos_id) {
            pending_sos = false;
            save_u32("pending", 0);
            // ACK stops retransmission but does not interrupt audio or turn off the lamp.
        }
        send_event(EVENT_STATUS, command->id);
        break;

    case CMD_LAMP:
        if (command->arg <= 1 && !(pending_sos && command->arg == 0)) {
            set_lamp(command->arg != 0);
        }
        send_event(EVENT_STATUS, command->id);
        break;

    case CMD_GET_STATUS:
        send_event(EVENT_STATUS, command->id);
        break;

    case CMD_REAR_STOP:
        if (alert_kind == ALERT_REAR) end_alert(ALERT_STOPPED);
        // A remote command is never allowed to silence local SOS audio.
        send_event(EVENT_STATUS, command->id);
        break;

    default:
        break;
    }
}

static void service_button(int64_t now)
{
    int level = gpio_get_level(SOS_GPIO);
    if (level != button_raw) {
        ESP_LOGI("button", "SOS GPIO%d -> %s", SOS_GPIO,
                 level == 0 ? "LOW (pressed)" : "HIGH (released)");
        button_raw = level;
        button_changed = now;
    }
    if (level == 0) {
        if (button_wakeup_armed) {
            gpio_wakeup_disable(SOS_GPIO);
            button_wakeup_armed = false;
        }
        button_down = true;
    }
    if (now - button_changed < SOS_DEBOUNCE_MS) return;
    if (level == button_stable) {
        // Also rearm after a short rejected LOW pulse returns HIGH.
        if (level == 1 && !button_wakeup_armed) {
            button_down = false;
            button_irq_latched = false;
            ESP_ERROR_CHECK(gpio_wakeup_disable(SOS_GPIO));
            ESP_ERROR_CHECK(gpio_wakeup_enable(SOS_GPIO,
                                               GPIO_INTR_LOW_LEVEL));
            ESP_ERROR_CHECK(gpio_intr_enable(SOS_GPIO));
            button_wakeup_armed = true;
        }
        return;
    }
    button_stable = level;

    if (level == 0) {
        set_lamp(true);
        if (!pending_sos) {
            ++sos_id;
            if (!sos_id) sos_id = 1;
            save_u32("sos_id", sos_id);
            save_u32("pending", sos_id);
            pending_sos = true;
        }
        next_sos = now;
        start_sos_alert();
        ESP_LOGW("remote", "SOS latched id=%lu", (unsigned long)sos_id);
    } else {
        button_down = false;
        button_irq_latched = false;
        ESP_ERROR_CHECK(gpio_wakeup_disable(SOS_GPIO));
        ESP_ERROR_CHECK(gpio_wakeup_enable(SOS_GPIO, GPIO_INTR_LOW_LEVEL));
        ESP_ERROR_CHECK(gpio_intr_enable(SOS_GPIO));
        button_wakeup_armed = true;
    }
}

static void service_sleep_policy(void)
{
    bool provisioned = mesh_link_is_provisioned();
    if (!provisioned && !unprovisioned_lock_held) {
        ESP_ERROR_CHECK(esp_pm_lock_acquire(unprovisioned_lock));
        unprovisioned_lock_held = true;
        ESP_LOGI("power", "Mesh unprovisioned: light sleep held off");
    } else if (provisioned && unprovisioned_lock_held) {
        ESP_ERROR_CHECK(esp_pm_lock_release(unprovisioned_lock));
        unprovisioned_lock_held = false;
        ESP_LOGI("power", "Mesh provisioned: light sleep enabled");
    }
}

void app_main(void)
{
    // Preserve Bluetooth Mesh credentials instead of erasing NVS on errors.
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(nvs_open("remote", NVS_READWRITE, &storage));
    nvs_get_u32(storage, "sos_id", &sos_id);
    nvs_get_u32(storage, "rear_id", &last_rear_command_id);
    uint32_t persisted = 0;
    nvs_get_u32(storage, "pending", &persisted);
    pending_sos = persisted != 0;
    if (pending_sos) sos_id = persisted;

    commands = xQueueCreate(16, sizeof(remote_command_t));
    assert(commands);

    gpio_config_t outputs = {
        .pin_bit_mask = (1ULL << JQ_POWER_GPIO) | (1ULL << LAMP_GPIO),
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&outputs));
    audio_power_off(false);
    set_lamp(pending_sos);

    gpio_config_t busy_input = {
        .pin_bit_mask = 1ULL << JQ_BUSY_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&busy_input));

    gpio_config_t sos_input = {
        .pin_bit_mask = 1ULL << SOS_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&sos_input));
    button_raw = gpio_get_level(SOS_GPIO);
    button_stable = 1;
    button_down = button_raw == 0;
    button_changed = now_ms();
    ESP_LOGI("button", "SOS GPIO%d initial=%s", SOS_GPIO,
             button_raw == 0 ? "LOW" : "HIGH");
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(SOS_GPIO, sos_isr, NULL));
    ESP_ERROR_CHECK(gpio_wakeup_enable(SOS_GPIO, GPIO_INTR_LOW_LEVEL));
    button_wakeup_armed = true;
    button_irq_latched = button_raw == 0;
    ESP_ERROR_CHECK(esp_sleep_enable_gpio_wakeup());

    esp_pm_config_t pm = {
        .max_freq_mhz = 160,
        .min_freq_mhz = 40,
        .light_sleep_enable = true,
    };
    ESP_ERROR_CHECK(esp_pm_configure(&pm));
    ESP_ERROR_CHECK(esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0,
                                       "mesh_unprovisioned", &unprovisioned_lock));
    ESP_ERROR_CHECK(esp_pm_lock_acquire(unprovisioned_lock));
    unprovisioned_lock_held = true;
    ESP_ERROR_CHECK(mesh_link_init(command_received));
    service_sleep_policy();
    ESP_LOGI("remote", "READY: waiting for Mesh provisioning; JQ8900 OFF");

    next_status = now_ms() + IDLE_STATUS_MS;
    if (pending_sos) start_sos_alert();

    for (;;) {
        remote_command_t command = {0};
        int wait_ms = (audio_state != AUDIO_OFF || button_down ||
                       button_raw != button_stable) ? 10 : 1000;
        if (xQueueReceive(commands, &command, pdMS_TO_TICKS(wait_ms)) == pdTRUE) {
            handle_command(&command);
        }

        int64_t now = now_ms();
        service_sleep_policy();
        service_button(now);
        service_audio(now);

        if (pending_sos && now >= next_sos) {
            send_event(EVENT_SOS, sos_id);
            next_sos = now + SOS_RETRY_MS;
        }
        if (now >= next_status) {
            send_event(EVENT_STATUS, alert_event_id);
            next_status = now + (audio_state != AUDIO_OFF ? ACTIVE_STATUS_MS : IDLE_STATUS_MS);
        }
    }
}
