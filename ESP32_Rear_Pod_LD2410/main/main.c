#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_pm.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "ld2410.h"
#include "mesh_link.h"
#include "rear_config.h"

#define PENDING_EVENT_SLOTS 4
#define CMD_LOCAL_BUTTON 0x80

typedef struct {
    bool active;
    uint8_t event;
    uint8_t detail;
    uint32_t id;
    int64_t next_send_ms;
} pending_event_t;

enum {
    SCAN_DONE_TIMEOUT = 0,
    SCAN_DONE_COMMAND = 1,
    SCAN_DONE_RADAR_FAULT = 2,
};

enum {
    FAULT_RADAR_START = 1,
    FAULT_RADAR_UART_TIMEOUT = 2,
    FAULT_FRIENDSHIP = 4,
};

static const char *TAG = "rear_pod";
static QueueHandle_t command_queue;
static nvs_handle_t storage;
static esp_pm_lock_handle_t unprovisioned_lock;
static esp_pm_lock_handle_t scan_lock;
static bool unprovisioned_lock_held;
static bool scan_lock_held;

static pending_event_t pending_events[PENDING_EVENT_SLOTS];
static uint32_t event_counter;
static uint32_t last_command_id;
static uint32_t persisted_rear_id;

static bool scanning;
static bool radar_powered;
static bool radar_fault;
static uint8_t radar_fault_code;
static int64_t scan_started_ms;
static int64_t scan_deadline_ms;
static int64_t radar_warmup_deadline_ms;
static int64_t radar_first_frame_deadline_ms;
static int64_t radar_last_frame_ms;
static int64_t next_active_report_ms;
static int64_t next_idle_status_ms;

static ld2410_measurement_t radar_measurement;
static bool radar_target;
static bool radar_moving;
static bool radar_stationary;
static uint16_t radar_distance_cm;
static uint8_t last_report_flags;
static uint8_t last_report_detail;
static bool report_snapshot_valid;

static int button_raw = 1;
static int button_stable = 1;
static int64_t button_low_since_ms;
static int64_t button_high_since_ms;
static bool button_action_fired;
static volatile bool button_wakeup_armed;
static volatile bool button_irq_latched;

static int64_t now_ms(void)
{
    return esp_timer_get_time() / 1000;
}

static void save_u32(const char *key, uint32_t value)
{
    ESP_ERROR_CHECK(nvs_set_u32(storage, key, value));
    ESP_ERROR_CHECK(nvs_commit(storage));
}

static uint32_t next_event_id(void)
{
    ++event_counter;
    if (event_counter == 0) event_counter = 1;
    save_u32("event_id", event_counter);
    return event_counter;
}

static uint8_t distance_detail(void)
{
    if (!radar_target) return 0;
    if (radar_distance_cm == 0) return 0xFF;
    uint32_t decimeters = ((uint32_t)radar_distance_cm + 5U) / 10U;
    return decimeters > 254U ? 254U : (uint8_t)decimeters;
}

static uint8_t status_flags(void)
{
    return (radar_powered ? (1U << 0) : 0) |
           (scanning ? (1U << 1) : 0) |
           (radar_target ? (1U << 2) : 0) |
           (radar_moving ? (1U << 3) : 0) |
           (radar_stationary ? (1U << 4) : 0) |
           (mesh_link_friend_ready() ? (1U << 5) : 0) |
           (radar_fault ? (1U << 6) : 0);
}

static void send_event(uint8_t event, uint32_t id, uint8_t detail)
{
    esp_err_t error = mesh_link_send(event, id, status_flags(), detail);
    if (error != ESP_OK && error != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "Event %u id=%lu not sent: %s",
                 event, (unsigned long)id, esp_err_to_name(error));
    }
}

static void send_status(uint32_t id)
{
    uint8_t detail = radar_fault ? radar_fault_code : distance_detail();
    send_event(EVENT_STATUS, id, detail);
}

static bool pending_event_exists(uint8_t event, uint32_t id)
{
    for (size_t index = 0; index < PENDING_EVENT_SLOTS; ++index) {
        if (pending_events[index].active &&
            pending_events[index].event == event &&
            pending_events[index].id == id) {
            return true;
        }
    }
    return false;
}

static void queue_critical_event(uint8_t event, uint32_t id, uint8_t detail)
{
    if (id == 0 || pending_event_exists(event, id)) return;

    for (size_t index = 0; index < PENDING_EVENT_SLOTS; ++index) {
        if (!pending_events[index].active) {
            pending_events[index] = (pending_event_t) {
                .active = true,
                .event = event,
                .detail = detail,
                .id = id,
                .next_send_ms = 0,
            };
            if (event == EVENT_REAR_CONFIRM) {
                persisted_rear_id = id;
                save_u32("pending_rear", id);
            }
            return;
        }
    }

    ESP_LOGE(TAG, "Critical-event queue full; sending event once");
    send_event(event, id, detail);
}

static void acknowledge_event(uint32_t id)
{
    for (size_t index = 0; index < PENDING_EVENT_SLOTS; ++index) {
        if (!pending_events[index].active || pending_events[index].id != id) {
            continue;
        }
        ESP_LOGI(TAG, "ACK event=%u id=%lu", pending_events[index].event,
                 (unsigned long)id);
        if (pending_events[index].event == EVENT_REAR_CONFIRM &&
            persisted_rear_id == id) {
            persisted_rear_id = 0;
            save_u32("pending_rear", 0);
        }
        pending_events[index].active = false;
    }
}

static void service_pending_events(int64_t now)
{
    for (size_t index = 0; index < PENDING_EVENT_SLOTS; ++index) {
        pending_event_t *pending = &pending_events[index];
        if (!pending->active || now < pending->next_send_ms) continue;
        send_event(pending->event, pending->id, pending->detail);
        pending->next_send_ms = now + EVENT_RETRY_MS;
    }
}

static void set_radar_fault(bool fault, uint8_t code)
{
    if (fault) {
        if (radar_fault && radar_fault_code == code) return;
        radar_fault = true;
        radar_fault_code = code;
        uint32_t id = next_event_id();
        queue_critical_event(EVENT_FAULT, id, code);
        ESP_LOGE(TAG, "Radar fault code=%u", code);
    } else if (radar_fault) {
        radar_fault = false;
        radar_fault_code = 0;
        send_status(event_counter);
        ESP_LOGI(TAG, "Radar communication recovered");
    }
}

static void reset_radar_values(void)
{
    radar_measurement = (ld2410_measurement_t) {0};
    radar_target = false;
    radar_moving = false;
    radar_stationary = false;
    radar_distance_cm = 0;
    radar_last_frame_ms = 0;
    report_snapshot_valid = false;
}

static int64_t scan_duration_ms(uint8_t minutes)
{
    if (minutes == 0) return DEFAULT_SCAN_MS;
    if (minutes > MAX_SCAN_MINUTES) minutes = MAX_SCAN_MINUTES;
    return (int64_t)minutes * 60LL * 1000LL;
}

static void start_scan(uint8_t minutes)
{
    int64_t now = now_ms();
    if (scanning) {
        scan_deadline_ms = now + scan_duration_ms(minutes);
        ESP_LOGI(TAG, "Scan deadline refreshed");
        return;
    }

    scanning = true;
    scan_started_ms = now;
    scan_deadline_ms = now + scan_duration_ms(minutes);
    radar_warmup_deadline_ms = now + RADAR_WARMUP_MS;
    radar_first_frame_deadline_ms = now + RADAR_FIRST_FRAME_MS;
    next_active_report_ms = radar_warmup_deadline_ms;
    reset_radar_values();
    save_u32("scan_active", 1);

    if (!scan_lock_held) {
        ESP_ERROR_CHECK(esp_pm_lock_acquire(scan_lock));
        scan_lock_held = true;
    }

    ESP_ERROR_CHECK(gpio_set_level(RADAR_POWER_GPIO,
                                   RADAR_POWER_ACTIVE_LEVEL));
    radar_powered = true;
    esp_err_t error = ld2410_uart_start();
    if (error != ESP_OK) {
        set_radar_fault(true, FAULT_RADAR_START);
    }
    ESP_LOGI(TAG, "Radar ON; scan duration=%lld ms",
             (long long)(scan_deadline_ms - now));
}

static void stop_scan(uint8_t reason, bool emit_event)
{
    if (!scanning) return;

    ld2410_uart_stop();
    ESP_ERROR_CHECK(gpio_set_level(RADAR_POWER_GPIO,
                                   !RADAR_POWER_ACTIVE_LEVEL));
    radar_powered = false;
    scanning = false;
    save_u32("scan_active", 0);
    reset_radar_values();

    if (scan_lock_held) {
        ESP_ERROR_CHECK(esp_pm_lock_release(scan_lock));
        scan_lock_held = false;
    }

    if (emit_event) {
        queue_critical_event(EVENT_SCAN_DONE, next_event_id(), reason);
    }
    next_idle_status_ms = now_ms();
    ESP_LOGI(TAG, "Radar OFF; scan done reason=%u; light sleep allowed", reason);
}

static void service_radar(int64_t now)
{
    if (!scanning) return;
    if (now >= scan_deadline_ms) {
        stop_scan(SCAN_DONE_TIMEOUT, true);
        return;
    }

    if (ld2410_uart_is_started()) {
        ld2410_measurement_t measurement;
        if (ld2410_poll(&measurement)) {
            radar_measurement = measurement;
            radar_last_frame_ms = now;
            radar_target = measurement.target;
            radar_moving = measurement.moving;
            radar_stationary = measurement.stationary;
            radar_distance_cm = measurement.detection_distance_cm;
            set_radar_fault(false, 0);
        }
    }

    if (now < radar_warmup_deadline_ms) return;

    if (radar_last_frame_ms == 0 && now >= radar_first_frame_deadline_ms) {
        set_radar_fault(true, FAULT_RADAR_UART_TIMEOUT);
    } else if (radar_last_frame_ms != 0 &&
               now - radar_last_frame_ms >= RADAR_FRAME_STALE_MS) {
        set_radar_fault(true, FAULT_RADAR_UART_TIMEOUT);
    }

    // OUT remains a useful presence fallback if UART data is temporarily stale.
    if (radar_last_frame_ms == 0 ||
        now - radar_last_frame_ms >= RADAR_FRAME_STALE_MS) {
        radar_target = gpio_get_level(RADAR_OUT_GPIO) != 0;
        radar_moving = false;
        radar_stationary = false;
        radar_distance_cm = 0;
    }

    uint8_t flags = status_flags();
    uint8_t detail = distance_detail();
    const uint8_t immediate_mask = (1U << 2) | (1U << 6);
    bool important_changed = report_snapshot_valid &&
                             (((flags ^ last_report_flags) &
                               immediate_mask) != 0);

    // Distance and moving/stationary states naturally fluctuate from frame to
    // frame.  Do not turn each fluctuation into a Mesh packet.  Report those
    // values on the periodic interval, while presence and fault transitions
    // remain immediate.
    if (!report_snapshot_valid || important_changed ||
        now >= next_active_report_ms) {
        send_event(EVENT_RADAR_UPDATE, event_counter, detail);
        last_report_flags = flags;
        last_report_detail = detail;
        report_snapshot_valid = true;
        next_active_report_ms = now + ACTIVE_REPORT_MS;
    }
}

static void IRAM_ATTR button_isr(void *argument)
{
    (void)argument;

    // gpio_wakeup_enable() configures a LOW-level interrupt on ESP32-C3.
    // Mask it immediately in the ISR so one physical press cannot keep
    // retriggering the CPU and filling command/Mesh queues.  The task enables
    // it again only after GPIO3 has been HIGH for the release debounce time.
    gpio_intr_disable(REAR_BUTTON_GPIO);
    button_wakeup_armed = false;

    if (button_irq_latched) {
        return;
    }
    button_irq_latched = true;
    rear_command_t wake_message = {.type = CMD_LOCAL_BUTTON};
    BaseType_t task_woken = pdFALSE;
    if (xQueueSendFromISR(command_queue, &wake_message, &task_woken) != pdTRUE) {
        button_irq_latched = false;
    }
    if (task_woken) portYIELD_FROM_ISR();
}

static void mesh_command_received(const rear_command_t *command)
{
    if (xQueueSend(command_queue, command, 0) != pdTRUE) {
        ESP_LOGW(TAG, "Command queue full; gateway must retry");
    }
}

static void fire_rear_confirm(void)
{
    uint32_t id = next_event_id();
    queue_critical_event(EVENT_REAR_CONFIRM, id, 0);
    start_scan(0);
    ESP_LOGW(TAG, "REAR CONFIRM id=%lu; 5-minute scan started",
             (unsigned long)id);
}

static void service_button(int64_t now)
{
    int level = gpio_get_level(REAR_BUTTON_GPIO);
    if (level != button_raw) {
        button_raw = level;
        if (level == 0) {
            button_low_since_ms = now;
            button_high_since_ms = 0;
        } else {
            button_low_since_ms = 0;
            button_high_since_ms = now;
        }
        ESP_LOGI(TAG, "Button GPIO%d=%s", REAR_BUTTON_GPIO,
                 level == 0 ? "LOW" : "HIGH");
    }

    if (level == 0) {
        button_stable = 0;
        button_high_since_ms = 0;

        // A GPIO wake on ESP32-C3 is level-sensitive.  Treat its ISR only as
        // a wake hint and accept a press solely after GPIO3 has remained LOW
        // long enough.  This rejects stale wake status and narrow noise pulses.
        if (button_low_since_ms == 0) {
            button_low_since_ms = now;
        }
        if (!button_action_fired &&
            now - button_low_since_ms >= BUTTON_PRESS_MS) {
            button_action_fired = true;
            fire_rear_confirm();
        }
        return;
    }

    button_low_since_ms = 0;
    if (button_action_fired || button_stable == 0 ||
        button_irq_latched || !button_wakeup_armed) {
        if (button_high_since_ms == 0) {
            button_high_since_ms = now;
        }
        if (now - button_high_since_ms >= BUTTON_RELEASE_MS) {
            button_action_fired = false;
            button_stable = 1;
            button_irq_latched = false;

            if (!button_wakeup_armed) {
                // Reset the wake source before arming the next independent
                // press. gpio_wakeup_enable intentionally selects LOW level.
                ESP_ERROR_CHECK(gpio_wakeup_disable(REAR_BUTTON_GPIO));
                ESP_ERROR_CHECK(gpio_wakeup_enable(REAR_BUTTON_GPIO,
                                                   GPIO_INTR_LOW_LEVEL));
                ESP_ERROR_CHECK(gpio_intr_enable(REAR_BUTTON_GPIO));
                button_wakeup_armed = true;
            }
        }
    } else {
        button_stable = 1;
    }
}

static bool accept_new_action_command(const rear_command_t *command)
{
    if (command->id == 0 || command->id <= last_command_id) {
        send_status(command->id);
        return false;
    }
    last_command_id = command->id;
    save_u32("command_id", last_command_id);
    return true;
}

static void handle_command(const rear_command_t *command)
{
    switch (command->type) {
    case CMD_LOCAL_BUTTON:
        // Do not create an event directly from a level-triggered wake IRQ.
        // service_button() verifies a real, debounced LOW level first.
        button_stable = 0;
        button_low_since_ms = 0;
        button_high_since_ms = 0;
        break;

    case CMD_START_SCAN:
        if (accept_new_action_command(command)) {
            start_scan(command->arg);
            send_status(command->id);
        }
        break;

    case CMD_STOP_SCAN:
        if (accept_new_action_command(command)) {
            stop_scan(SCAN_DONE_COMMAND, true);
            send_status(command->id);
        }
        break;

    case CMD_ACK_EVENT:
        acknowledge_event(command->id);
        send_status(command->id);
        break;

    case CMD_GET_STATUS:
        send_status(command->id);
        break;

    case CMD_CLEAR_FAULT:
        if (accept_new_action_command(command)) {
            radar_fault = false;
            radar_fault_code = 0;
            send_status(command->id);
        }
        break;

    default:
        break;
    }
}

static void service_sleep_policy(void)
{
    bool provisioned = mesh_link_is_provisioned();
    if (!provisioned && !unprovisioned_lock_held) {
        ESP_ERROR_CHECK(esp_pm_lock_acquire(unprovisioned_lock));
        unprovisioned_lock_held = true;
        ESP_LOGI(TAG, "Unprovisioned: automatic light sleep held off");
    } else if (provisioned && unprovisioned_lock_held) {
        ESP_ERROR_CHECK(esp_pm_lock_release(unprovisioned_lock));
        unprovisioned_lock_held = false;
        ESP_LOGI(TAG, "Provisioned: Mesh LPN automatic light sleep enabled");
    }
}

static void initialise_nvs(void)
{
    esp_err_t error = nvs_flash_init();
    if (error == ESP_ERR_NVS_NO_FREE_PAGES ||
        error == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        error = nvs_flash_init();
    }
    ESP_ERROR_CHECK(error);
    ESP_ERROR_CHECK(nvs_open("rear_pod", NVS_READWRITE, &storage));
    nvs_get_u32(storage, "event_id", &event_counter);
    nvs_get_u32(storage, "command_id", &last_command_id);
    nvs_get_u32(storage, "pending_rear", &persisted_rear_id);
}

static void initialise_gpio(void)
{
    gpio_config_t power_output = {
        .pin_bit_mask = 1ULL << RADAR_POWER_GPIO,
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&power_output));
    ESP_ERROR_CHECK(gpio_set_level(RADAR_POWER_GPIO,
                                   !RADAR_POWER_ACTIVE_LEVEL));
    radar_powered = false;
    ld2410_uart_stop();

    gpio_config_t radar_out = {
        .pin_bit_mask = 1ULL << RADAR_OUT_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&radar_out));

    gpio_config_t rear_button = {
        .pin_bit_mask = 1ULL << REAR_BUTTON_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&rear_button));
    button_raw = gpio_get_level(REAR_BUTTON_GPIO);
    button_stable = button_raw != 0 ? 1 : 0;
    button_action_fired = button_raw == 0;
    button_irq_latched = button_raw == 0;
    button_low_since_ms = button_raw == 0 ? now_ms() : 0;
    button_high_since_ms = button_raw != 0 ? now_ms() : 0;

    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(REAR_BUTTON_GPIO, button_isr, NULL));
    ESP_ERROR_CHECK(gpio_wakeup_enable(REAR_BUTTON_GPIO,
                                       GPIO_INTR_LOW_LEVEL));
    button_wakeup_armed = true;
    ESP_ERROR_CHECK(esp_sleep_enable_gpio_wakeup());
}

static void initialise_power_management(void)
{
    esp_pm_config_t power_management = {
        .max_freq_mhz = 160,
        .min_freq_mhz = 40,
        .light_sleep_enable = true,
    };
    ESP_ERROR_CHECK(esp_pm_configure(&power_management));
    ESP_ERROR_CHECK(esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0,
                                       "mesh_unprovisioned",
                                       &unprovisioned_lock));
    ESP_ERROR_CHECK(esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0,
                                       "radar_scan", &scan_lock));
    ESP_ERROR_CHECK(esp_pm_lock_acquire(unprovisioned_lock));
    unprovisioned_lock_held = true;
}

void app_main(void)
{
    initialise_nvs();
    command_queue = xQueueCreate(16, sizeof(rear_command_t));
    assert(command_queue != NULL);
    initialise_gpio();
    initialise_power_management();

    ESP_ERROR_CHECK(mesh_link_init(mesh_command_received));
    service_sleep_policy();

    if (persisted_rear_id != 0) {
        queue_critical_event(EVENT_REAR_CONFIRM, persisted_rear_id, 0);
        ESP_LOGW(TAG, "Restored unacknowledged REAR event id=%lu",
                 (unsigned long)persisted_rear_id);
    }

    uint32_t resume_scan = 0;
    nvs_get_u32(storage, "scan_active", &resume_scan);
    if (resume_scan != 0) {
        ESP_LOGW(TAG, "Reset occurred during scan; restarting safe 5-minute scan");
        start_scan(0);
    }

    next_idle_status_ms = now_ms() + IDLE_STATUS_MS;
    ESP_LOGI(TAG, "READY: button GPIO3, radar OFF, waiting for Mesh");

    for (;;) {
        rear_command_t command = {0};
        int wait_ms = scanning ? 20 :
                      (button_action_fired || button_raw == 0 ||
                       button_stable == 0 ? 10 : 1000);
        if (xQueueReceive(command_queue, &command,
                          pdMS_TO_TICKS(wait_ms)) == pdTRUE &&
            command.type != 0) {
            handle_command(&command);
        }

        int64_t now = now_ms();
        service_sleep_policy();
        service_button(now);
        service_radar(now);
        service_pending_events(now);

        if (!scanning && now >= next_idle_status_ms) {
            send_status(event_counter);
            next_idle_status_ms = now + IDLE_STATUS_MS;
        }
    }
}
