/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "gps.h"
#include "a7670.h"
#include "tb_mqtt.h"
#include "jq8900.h"
#include "pi_link.h"
#include "tim.h"
#include "usart.h"
#include "tripguard_rtos.h"
#include "tripguard_config.h"
#include "tripguard_event.h"
#include "tripguard_gateway.h"
#include "tripguard_power.h"
#include "tripguard_protocol.h"
#include "tripguard_supervisor.h"
#include "camera.h"

#ifndef TB_MQTT_SAFE_QUIESCE_API
#error "tb_mqtt.h is old: replace Core/Inc/tb_mqtt.h with the FullFix V2 header"
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TRIPGUARD_DIAGNOSTICS_PERIOD_MS 250U
#define TRIPGUARD_BUTTON_DEBOUNCE_MS     80U
#define TRIPGUARD_BUTTON_RELEASE_MS     150U
#define TRIPGUARD_RADAR_SAMPLE_MS         50U
#define TRIPGUARD_RADAR_HIGH_CONFIRM_MS  2000U
#define TRIPGUARD_RADAR_LOW_CONFIRM_MS   8000U
#define TRIPGUARD_RADAR_ALERT_GUARD_MS  30000U
#define TRIPGUARD_CAMERA_ALERT_GUARD_MS 30000U
#define TRIPGUARD_CELLULAR_FLAG_RADAR_SMS (1UL << 0)
#define TRIPGUARD_CELLULAR_FLAG_SOS        (1UL << 1)
#define TRIPGUARD_ALERT_COMMAND_TIMEOUT_MS  5000U
#define TRIPGUARD_ALERT_SMS_TIMEOUT_MS     60000U
#define TRIPGUARD_ALERT_DIAL_TIMEOUT_MS    30000U
#define TRIPGUARD_TELEMETRY_RETRY_MS          250U
#define TRIPGUARD_REAR_HISTORY_SIZE             8U

typedef enum
{
  TRIPGUARD_ALERT_NONE = 0U,
  TRIPGUARD_ALERT_RADAR_SMS,
  TRIPGUARD_ALERT_SOS_SMS_CALL
} TripGuard_AlertKind_t;

typedef enum
{
  ALERT_IDLE = 0U,
  ALERT_WAIT_NETWORK,
  ALERT_SMS_MODE_START,
  ALERT_SMS_MODE_WAIT,
  ALERT_SMS_CHARSET_START,
  ALERT_SMS_CHARSET_WAIT,
  ALERT_SMS_DCS_START,
  ALERT_SMS_DCS_WAIT,
  ALERT_SMS_PROMPT_START,
  ALERT_SMS_PROMPT_WAIT,
  ALERT_SMS_BODY_START,
  ALERT_SMS_BODY_WAIT,
  ALERT_CALL_START,
  ALERT_CALL_WAIT,
  ALERT_CALL_ACTIVE,
  ALERT_HANGUP_START,
  ALERT_HANGUP_WAIT,
  ALERT_RESTORE_CHARSET_START,
  ALERT_RESTORE_CHARSET_WAIT
} TripGuard_AlertPhase_t;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
static char rtos_telemetry_json[2560];

volatile uint32_t tripguard_default_heartbeat;
volatile uint32_t tripguard_stack_overflow_fault;
volatile GPS_State_t tripguard_gps_state;
volatile GPS_CommState_t tripguard_gps_comm_state;
volatile float tripguard_speed_kmh;
volatile TripGuard_DrivingState_t tripguard_driving_state;
volatile uint8_t tripguard_radar_online;
volatile uint8_t tripguard_person_detected;
volatile uint8_t tripguard_radar_state;
volatile uint16_t tripguard_radar_distance_cm;
volatile uint16_t tripguard_radar_moving_distance_cm;
volatile uint16_t tripguard_radar_still_distance_cm;
volatile uint8_t tripguard_radar_moving_energy;
volatile uint8_t tripguard_radar_still_energy;
volatile uint32_t tripguard_radar_valid_frame_count;
volatile uint32_t tripguard_radar_invalid_frame_count;
volatile uint32_t tripguard_radar_uart_error_count;
volatile uint32_t tripguard_radar_rx_overflow_count;
volatile uint8_t tripguard_radar_alert_armed;
volatile uint32_t tripguard_radar_alert_count;
volatile uint32_t tripguard_radar_last_alert_ms;
volatile uint8_t tripguard_camera_person;
volatile uint16_t tripguard_camera_confidence_permille;
volatile uint8_t tripguard_camera_alert_armed;
volatile uint32_t tripguard_camera_alert_count;
volatile uint32_t tripguard_camera_last_alert_ms;
volatile uint16_t tripguard_person_alert_value;
volatile uint16_t tripguard_servo_angle_deg;
volatile uint16_t tripguard_servo_pulse_us;
volatile uint8_t tripguard_servo_moving;
volatile uint8_t tripguard_servo_tracking_radar;
volatile uint32_t tripguard_servo_move_count;
volatile uint8_t tripguard_sos_pending;
volatile uint32_t tripguard_sos_count;
volatile uint8_t tripguard_sos_button_latched;
volatile uint8_t tripguard_sos_button_raw;
volatile uint32_t tripguard_driver_ack_count;
volatile uint8_t tripguard_ack_button_latched;
volatile uint8_t tripguard_ack_button_raw;
volatile uint8_t tripguard_audio_busy_raw;
volatile uint16_t tripguard_audio_track;
volatile uint32_t tripguard_audio_error_count;
volatile uint8_t tripguard_alert_active;
volatile uint8_t tripguard_alert_phase;
volatile uint8_t tripguard_alert_last_kind;
volatile A7670_CommandResult_t tripguard_alert_last_result;
volatile uint8_t tripguard_alert_last_success;
volatile uint32_t tripguard_sms_sent_count;
volatile uint32_t tripguard_call_started_count;
volatile uint32_t tripguard_alert_error_count;
volatile char tripguard_alert_last_response[192];
volatile uint8_t tripguard_radar_alert_enabled = 1U;
volatile uint32_t tripguard_telemetry_period_ms =
    TRIPGUARD_TELEMETRY_PERIOD_DEFAULT_MS;
volatile uint32_t tripguard_remote_command_count;
volatile uint32_t tripguard_remote_rejected_count;
volatile char tripguard_last_rpc_method[TB_RPC_METHOD_SIZE];
volatile char tripguard_last_rpc_result[48];
volatile uint8_t tripguard_rear_confirmed;
volatile uint8_t tripguard_rear_confirm_node;
volatile uint16_t tripguard_rear_confirm_sequence;
volatile uint32_t tripguard_rear_confirm_count;
volatile uint32_t tripguard_rear_duplicate_count;
volatile uint32_t tripguard_rear_invalid_count;
volatile uint8_t tripguard_remote_sos_received;
volatile uint16_t tripguard_remote_sos_sequence;
volatile uint32_t tripguard_remote_sos_count;
volatile uint32_t tripguard_remote_sos_duplicate_count;
volatile uint32_t tripguard_telemetry_queue_full_count;
volatile uint32_t tripguard_telemetry_coalesced_count;
volatile uint8_t tripguard_telemetry_pending;
volatile uint8_t tripguard_gateway_init_ok;
volatile uint32_t tripguard_gateway_ack_queue_full_count;

static TripGuard_AlertKind_t alert_kind;
static TripGuard_AlertPhase_t alert_phase;
static uint8_t alert_sequence_ok;
static uint8_t alert_sos_pending;
static uint32_t alert_deadline_ms;
static char alert_command[96];
static uint32_t servo_stop_deadline_ms;

static uint8_t rear_history_node[TRIPGUARD_REAR_HISTORY_SIZE];
static uint16_t rear_history_sequence[TRIPGUARD_REAR_HISTORY_SIZE];
static uint8_t rear_history_count;
static uint8_t rear_history_next;

static uint8_t remote_sos_history_node[TRIPGUARD_REAR_HISTORY_SIZE];
static uint16_t remote_sos_history_sequence[TRIPGUARD_REAR_HISTORY_SIZE];
static uint8_t remote_sos_history_count;
static uint8_t remote_sos_history_next;

extern volatile uint8_t tripguard_last_publish_queued;

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for SafetyTask */
osThreadId_t SafetyTaskHandle;
const osThreadAttr_t SafetyTask_attributes = {
  .name = "SafetyTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for CellularTask */
osThreadId_t CellularTaskHandle;
const osThreadAttr_t CellularTask_attributes = {
  .name = "CellularTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for GpsTask */
osThreadId_t GpsTaskHandle;
const osThreadAttr_t GpsTask_attributes = {
  .name = "GpsTask",
  .stack_size = 384 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for TelemetryTask */
osThreadId_t TelemetryTaskHandle;
const osThreadAttr_t TelemetryTask_attributes = {
  .name = "TelemetryTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for AudioTask */
osThreadId_t AudioTaskHandle;
const osThreadAttr_t AudioTask_attributes = {
  .name = "AudioTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for HealthTask */
osThreadId_t HealthTaskHandle;
const osThreadAttr_t HealthTask_attributes = {
  .name = "HealthTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for GatewayLinkTask */
osThreadId_t GatewayLinkTaskHandle;
const osThreadAttr_t GatewayLinkTask_attributes = {
  .name = "GatewayLinkTask",
  .stack_size = 384 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for PowerManagerTask */
osThreadId_t PowerManagerTaskHandle;
const osThreadAttr_t PowerManagerTask_attributes = {
  .name = "PowerManagerTask",
  .stack_size = 384 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for TripGuardSupervisorTask */
osThreadId_t TripGuardSupervisorTaskHandle;
const osThreadAttr_t TripGuardSupervisorTask_attributes = {
  .name = "TripGuardSupervisorTask",
  .stack_size = 384 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for PiLinkTask */
osThreadId_t PiLinkTaskHandle;
const osThreadAttr_t PiLinkTask_attributes = {
  .name = "PiLinkTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for audioQueue */
osMessageQueueId_t audioQueueHandle;
const osMessageQueueAttr_t audioQueue_attributes = {
  .name = "audioQueue"
};
/* Definitions for telemetryEventQueue */
osMessageQueueId_t telemetryEventQueueHandle;
const osMessageQueueAttr_t telemetryEventQueue_attributes = {
  .name = "telemetryEventQueue"
};
/* Definitions for tripguardEventQueue */
osMessageQueueId_t tripguardEventQueueHandle;
const osMessageQueueAttr_t tripguardEventQueue_attributes = {
  .name = "tripguardEventQueue"
};
/* SafetyTask -> GatewayLinkTask: an ACK exists only after safety commit. */
osMessageQueueId_t gatewayAckQueueHandle;
const osMessageQueueAttr_t gatewayAckQueue_attributes = {
  .name = "gatewayAckQueue"
};
/* State-machine input and applied-RPC result queues. */
osMessageQueueId_t tripguardSupervisorQueueHandle;
const osMessageQueueAttr_t tripguardSupervisorQueue_attributes = {
  .name = "tripguardSupervisorQueue"
};
osMessageQueueId_t tripguardRpcResultQueueHandle;
const osMessageQueueAttr_t tripguardRpcResultQueue_attributes = {
  .name = "tripguardRpcResultQueue"
};
/* Definitions for gpsDataMutex */
osMutexId_t gpsDataMutexHandle;
const osMutexAttr_t gpsDataMutex_attributes = {
  .name = "gpsDataMutex"
};
/* Definitions for systemEvents */
osEventFlagsId_t systemEventsHandle;
const osEventFlagsAttr_t systemEvents_attributes = {
  .name = "systemEvents"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void TripGuard_QueueAudio(TripGuard_AudioCommand_t command);
static void TripGuard_QueueTelemetry(TripGuard_TelemetryEvent_t event);
static void TripGuard_ProcessGatewayEvents(void);
static uint8_t TripGuard_IsRearDuplicate(uint8_t node_id,
                                         uint16_t sequence);
static void TripGuard_RememberRear(uint8_t node_id, uint16_t sequence);
static void TripGuard_BuildTelemetry(char *buffer, size_t buffer_size,
                                     TripGuard_TelemetryEvent_t event);
static void TripGuard_AlertStep(void);
static void TripGuard_AlertBegin(TripGuard_AlertKind_t kind);
static void TripGuard_AlertFinish(void);
static void TripGuard_AlertSmsDone(uint8_t success);
static uint8_t TripGuard_AlertTakeResult(A7670_CommandResult_t *result);
static void TripGuard_HandleRpcCommand(void);
static uint8_t TripGuard_RpcBool(const char *params, uint8_t *value);
static uint8_t TripGuard_RpcUint32(const char *params, uint32_t *value);
static void TripGuard_SetRpcResult(const char *method, const char *result);
static uint8_t TripGuard_TimeReached(uint32_t now_ms, uint32_t deadline_ms);
static void TripGuard_ServoInit(void);
static void TripGuard_ServoMoveToRadar(uint32_t now_ms);
static void TripGuard_ServoMoveToHome(uint32_t now_ms);
static void TripGuard_ServoUpdate(uint32_t now_ms);
static void TripGuard_ServoStop(void);

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartSafetyTask(void *argument);
void StartCellularTask(void *argument);
void StartGpsTask(void *argument);
void StartTelemetryTask(void *argument);
void StartAudioTask(void *argument);
void StartHealthTask(void *argument);
void StartGatewayLinkTask(void *argument);
void StartPowerManagerTask(void *argument);
void StartTripGuardSupervisorTask(void *argument);
void StartPiLinkTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
  (void)xTask;
  (void)pcTaskName;
  tripguard_stack_overflow_fault = 1U;
  taskDISABLE_INTERRUPTS();
  for (;;)
  {
  }
}
/* USER CODE END 4 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of gpsDataMutex */
  gpsDataMutexHandle = osMutexNew(&gpsDataMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of audioQueue */
  audioQueueHandle = osMessageQueueNew (8, sizeof(uint32_t), &audioQueue_attributes);

  /* creation of telemetryEventQueue */
  telemetryEventQueueHandle = osMessageQueueNew (8, sizeof(uint32_t), &telemetryEventQueue_attributes);

  /* creation of tripguardEventQueue */
  tripguardEventQueueHandle = osMessageQueueNew (8, sizeof(tripguard_event_t), &tripguardEventQueue_attributes);

  /* creation of gatewayAckQueue */
  gatewayAckQueueHandle = osMessageQueueNew(
      8U, sizeof(tripguard_ack_t), &gatewayAckQueue_attributes);

  tripguardSupervisorQueueHandle = osMessageQueueNew(
      8U, sizeof(TripGuard_SupervisorEvent_t),
      &tripguardSupervisorQueue_attributes);

  tripguardRpcResultQueueHandle = osMessageQueueNew(
      4U, sizeof(TripGuard_SupervisorRpcResult_t),
      &tripguardRpcResultQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of SafetyTask */
  SafetyTaskHandle = osThreadNew(StartSafetyTask, NULL, &SafetyTask_attributes);

  /* creation of CellularTask */
  CellularTaskHandle = osThreadNew(StartCellularTask, NULL, &CellularTask_attributes);

  /* creation of GpsTask */
  GpsTaskHandle = osThreadNew(StartGpsTask, NULL, &GpsTask_attributes);

  /* creation of TelemetryTask */
  TelemetryTaskHandle = osThreadNew(StartTelemetryTask, NULL, &TelemetryTask_attributes);

  /* creation of AudioTask */
  AudioTaskHandle = osThreadNew(StartAudioTask, NULL, &AudioTask_attributes);

  /* creation of HealthTask */
  HealthTaskHandle = osThreadNew(StartHealthTask, NULL, &HealthTask_attributes);

  /* creation of GatewayLinkTask */
  GatewayLinkTaskHandle = osThreadNew(StartGatewayLinkTask, NULL, &GatewayLinkTask_attributes);

  /* creation of PowerManagerTask */
  PowerManagerTaskHandle = osThreadNew(StartPowerManagerTask, NULL, &PowerManagerTask_attributes);

  /* creation of TripGuardSupervisorTask */
  TripGuardSupervisorTaskHandle = osThreadNew(
      StartTripGuardSupervisorTask, NULL,
      &TripGuardSupervisorTask_attributes);

  /* creation of PiLinkTask */
  PiLinkTaskHandle = osThreadNew(StartPiLinkTask, NULL,
                                 &PiLinkTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* creation of systemEvents */
  systemEventsHandle = osEventFlagsNew(&systemEvents_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
  tripguard_gateway_init_ok = (uint8_t)TripGuard_Gateway_Init(
      tripguardEventQueueHandle,
      gatewayAckQueueHandle,
      systemEventsHandle);
  TripGuard_Power_Init(tripguardEventQueueHandle,
                       gatewayAckQueueHandle,
                       telemetryEventQueueHandle,
                       audioQueueHandle);
  (void)TripGuard_Supervisor_Init(tripguardSupervisorQueueHandle,
                                  tripguardRpcResultQueueHandle,
                                  telemetryEventQueueHandle);
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    tripguard_default_heartbeat++;
    osDelay(1000U);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartSafetyTask */
/**
* @brief Function implementing the SafetyTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSafetyTask */
void StartSafetyTask(void *argument)
{
  /* USER CODE BEGIN StartSafetyTask */
  uint32_t flags;
  uint32_t now_ms;
  uint32_t radar_high_since_ms = 0U;
  uint32_t radar_low_since_ms = 0U;
  uint8_t radar_raw_state;
  uint8_t camera_raw_state;
  uint8_t sensor_ready = 0U;
  uint8_t radar_filtered_state = 0U;
  uint8_t camera_person_latched = 0U;
  uint8_t detection_was_active = 0U;
  uint8_t sos_button_latched;
  uint8_t ack_button_latched;
  uint32_t sos_release_since_ms = 0U;
  uint32_t ack_release_since_ms = 0U;

  (void)argument;
  TripGuard_ServoInit();
  tripguard_servo_tracking_radar = 0U;
  radar_raw_state = TripGuard_Gateway_IsRadarPersonDetected();
  tripguard_radar_state = radar_raw_state;
  tripguard_person_detected = 0U;
  tripguard_radar_online = 0U;
  tripguard_radar_alert_armed = 0U;
  tripguard_camera_person = 0U;
  tripguard_camera_confidence_permille = 0U;
  tripguard_camera_alert_armed = 0U;
  tripguard_camera_alert_count = 0U;
  tripguard_camera_last_alert_ms = 0U;
  tripguard_person_alert_value = 0U;
  if (radar_raw_state != 0U)
  {
    radar_high_since_ms = HAL_GetTick();
  }
  else
  {
    radar_low_since_ms = HAL_GetTick();
  }

  /* A button already held during boot must be released before it can fire. */
  sos_button_latched =
      (HAL_GPIO_ReadPin(SOS_BTN_GPIO_Port, SOS_BTN_Pin) == GPIO_PIN_RESET) ?
      1U : 0U;
  ack_button_latched =
      (HAL_GPIO_ReadPin(DRIVER_ACK_BTN_GPIO_Port,
                        DRIVER_ACK_BTN_Pin) == GPIO_PIN_RESET) ? 1U : 0U;
  tripguard_sos_button_latched = sos_button_latched;
  tripguard_ack_button_latched = ack_button_latched;
  tripguard_sos_button_raw = sos_button_latched;
  tripguard_ack_button_raw = ack_button_latched;

  for(;;)
  {
    flags = osEventFlagsWait(systemEventsHandle,
                             TRIPGUARD_EVENT_ALL_SAFETY,
                             osFlagsWaitAny,
                             TRIPGUARD_RADAR_SAMPLE_MS);
    if (flags == (uint32_t)osFlagsErrorTimeout)
    {
      flags = 0U;
    }
    else if ((flags & osFlagsError) != 0U)
    {
      osDelay(10U);
      continue;
    }

    /* Keep raw OUT and filtered presence separate. */
    now_ms = HAL_GetTick();
    TripGuard_ServoUpdate(now_ms);

    if ((flags & TRIPGUARD_EVENT_GATEWAY_RX) != 0U)
    {
      /* SafetyTask is the single writer of the remote-confirmation state. */
      TripGuard_ProcessGatewayEvents();
    }

    tripguard_sos_button_raw =
        (HAL_GPIO_ReadPin(SOS_BTN_GPIO_Port, SOS_BTN_Pin) == GPIO_PIN_RESET) ?
        1U : 0U;
    tripguard_ack_button_raw =
        (HAL_GPIO_ReadPin(DRIVER_ACK_BTN_GPIO_Port,
                          DRIVER_ACK_BTN_Pin) == GPIO_PIN_RESET) ? 1U : 0U;

    /*
     * Release debounce is non-blocking.  Once a press is accepted, all
     * further EXTI chatter is ignored until the input has stayed high long
     * enough.  This guarantees one physical press produces one event.
     */
    if (HAL_GPIO_ReadPin(SOS_BTN_GPIO_Port, SOS_BTN_Pin) == GPIO_PIN_SET)
    {
      if (sos_button_latched != 0U)
      {
        if (sos_release_since_ms == 0U)
        {
          sos_release_since_ms = now_ms;
        }
        else if ((uint32_t)(now_ms - sos_release_since_ms) >=
                 TRIPGUARD_BUTTON_RELEASE_MS)
        {
          sos_button_latched = 0U;
          tripguard_sos_button_latched = 0U;
          sos_release_since_ms = 0U;
          (void)osEventFlagsClear(systemEventsHandle,
                                  TRIPGUARD_EVENT_SOS_PRESSED);
        }
      }
    }
    else
    {
      sos_release_since_ms = 0U;
    }

    if (HAL_GPIO_ReadPin(DRIVER_ACK_BTN_GPIO_Port,
                         DRIVER_ACK_BTN_Pin) == GPIO_PIN_SET)
    {
      if (ack_button_latched != 0U)
      {
        if (ack_release_since_ms == 0U)
        {
          ack_release_since_ms = now_ms;
        }
        else if ((uint32_t)(now_ms - ack_release_since_ms) >=
                 TRIPGUARD_BUTTON_RELEASE_MS)
        {
          ack_button_latched = 0U;
          tripguard_ack_button_latched = 0U;
          ack_release_since_ms = 0U;
          (void)osEventFlagsClear(systemEventsHandle,
                                  TRIPGUARD_EVENT_ACK_PRESSED);
        }
      }
    }
    else
    {
      ack_release_since_ms = 0U;
    }

    radar_raw_state = TripGuard_Gateway_IsRadarPersonDetected();
    tripguard_radar_online = TripGuard_Gateway_IsRadarOnline();
    tripguard_radar_state = radar_raw_state;
    tripguard_radar_distance_cm =
        TripGuard_Gateway_GetRadarDistanceCm();
    tripguard_radar_moving_distance_cm = tripguard_radar_distance_cm;
    tripguard_radar_still_distance_cm = 0U;
    tripguard_radar_moving_energy = 0U;
    tripguard_radar_still_energy = 0U;
    tripguard_radar_valid_frame_count =
        tripguard_gateway_radar_person_count +
        tripguard_gateway_radar_clear_count +
        tripguard_gateway_radar_health_count;
    tripguard_radar_invalid_frame_count =
        tripguard_gateway_radar_duplicate_count;
    tripguard_radar_uart_error_count = 0U;
    tripguard_radar_rx_overflow_count = 0U;

    camera_raw_state = ((PiLink_IsOnline() != 0U) &&
                        (PiLink_IsReady() != 0U)) ?
                       PiLink_GetCameraPerson() : 0U;
    tripguard_camera_person = camera_raw_state;
    tripguard_camera_confidence_permille =
        (camera_raw_state != 0U) ? PiLink_GetCameraConfidencePermille() : 0U;

    /* Pi camera may alert only in ACTIVE; it never owns arming. */
    if (TripGuard_Supervisor_IsDetectionActive() == 0U)
    {
      /* Keep collecting diagnostics, but block every safety side effect. */
      radar_raw_state = 0U;
      camera_raw_state = 0U;
      camera_person_latched = 0U;
      tripguard_camera_alert_armed = 0U;
      detection_was_active = 0U;
    }
    else if (detection_was_active == 0U)
    {
      /* A person already visible when ACTIVE starts must still alert once. */
      detection_was_active = 1U;
      camera_person_latched = 0U;
      tripguard_camera_alert_armed = 1U;
    }

    if (radar_raw_state != 0U)
    {
      radar_low_since_ms = 0U;
      if (radar_high_since_ms == 0U)
      {
        radar_high_since_ms = now_ms;
      }
    }
    else
    {
      radar_high_since_ms = 0U;
      if (radar_low_since_ms == 0U)
      {
        radar_low_since_ms = now_ms;
      }
    }

    /* Ignore the sensor's power-on settling interval completely. */
    if (TripGuard_Supervisor_IsDetectionActive() == 0U)
    {
      radar_filtered_state = 0U;
      tripguard_person_detected = 0U;
      tripguard_radar_alert_armed = 0U;
      radar_high_since_ms = 0U;
      radar_low_since_ms = 0U;
      if (tripguard_servo_tracking_radar != 0U)
      {
        TripGuard_ServoStop();
        tripguard_servo_tracking_radar = 0U;
      }
    }
    else
    {
      sensor_ready = TripGuard_Gateway_IsRadarHealthy();
      if ((sensor_ready != 0U) &&
          (tripguard_radar_alert_armed == 0U) &&
          (tripguard_radar_last_alert_ms == 0U))
      {
        tripguard_radar_alert_armed = 1U;
      }

      /* Khong dung trang thai cu khi day radar bi thao hoac mat nguon. */
      if (sensor_ready == 0U)
      {
        radar_filtered_state = 0U;
        tripguard_person_detected = 0U;
        tripguard_radar_state = 0U;
        radar_high_since_ms = 0U;
        if (tripguard_servo_tracking_radar != 0U)
        {
          TripGuard_ServoStop();
          tripguard_servo_tracking_radar = 0U;
        }
      }

      /* A short high pulse is noise; require two continuous seconds. */
      if ((sensor_ready != 0U) &&
          (radar_raw_state != 0U) &&
          (radar_filtered_state == 0U) &&
          (radar_high_since_ms != 0U) &&
          ((uint32_t)(now_ms - radar_high_since_ms) >=
           TRIPGUARD_RADAR_HIGH_CONFIRM_MS))
      {
        radar_filtered_state = 1U;
        tripguard_person_detected = 1U;

        /* Radar dung co dinh: quay camera den goc da can chinh ve radar. */
        if (tripguard_servo_tracking_radar == 0U)
        {
          TripGuard_ServoMoveToRadar(now_ms);
          tripguard_servo_tracking_radar = 1U;
        }

        if ((tripguard_radar_alert_armed != 0U) &&
            (tripguard_radar_alert_enabled != 0U) &&
            ((tripguard_camera_last_alert_ms == 0U) ||
             ((uint32_t)(now_ms - tripguard_camera_last_alert_ms) >=
              TRIPGUARD_CAMERA_ALERT_GUARD_MS)))
        {
          tripguard_radar_alert_armed = 0U;
          tripguard_radar_last_alert_ms = now_ms;
          tripguard_radar_alert_count++;
          tripguard_person_alert_value = tripguard_radar_distance_cm;
          TripGuard_QueueAudio(TRIPGUARD_AUDIO_RADAR_WARNING);
          TripGuard_QueueTelemetry(TRIPGUARD_TELEMETRY_SAFETY_EVENT);
          if (CellularTaskHandle != NULL)
          {
            (void)osThreadFlagsSet(CellularTaskHandle,
                                   TRIPGUARD_CELLULAR_FLAG_RADAR_SMS);
          }
        }
      }

      /* Hold presence through chatter; clear only after eight quiet seconds. */
      if ((radar_raw_state == 0U) &&
          (radar_filtered_state != 0U) &&
          (radar_low_since_ms != 0U) &&
          ((uint32_t)(now_ms - radar_low_since_ms) >=
           TRIPGUARD_RADAR_LOW_CONFIRM_MS))
      {
        radar_filtered_state = 0U;
        tripguard_person_detected = 0U;

        /* Het nguoi lien tuc 8 giay: tra camera ve huong mac dinh. */
        if (tripguard_servo_tracking_radar != 0U)
        {
          TripGuard_ServoMoveToHome(now_ms);
          tripguard_servo_tracking_radar = 0U;
        }
      }

      if ((radar_filtered_state == 0U) &&
          (radar_raw_state == 0U) &&
          (tripguard_radar_alert_armed == 0U) &&
          (radar_low_since_ms != 0U) &&
          ((uint32_t)(now_ms - radar_low_since_ms) >=
           TRIPGUARD_RADAR_LOW_CONFIRM_MS) &&
          ((tripguard_radar_last_alert_ms == 0U) ||
           ((uint32_t)(now_ms - tripguard_radar_last_alert_ms) >=
            TRIPGUARD_RADAR_ALERT_GUARD_MS)))
      {
        tripguard_radar_alert_armed = 1U;
      }
    }

    if (TripGuard_Supervisor_IsDetectionActive() != 0U)
    {
      if (camera_raw_state == 0U)
      {
        camera_person_latched = 0U;
        if ((tripguard_camera_last_alert_ms == 0U) ||
            ((uint32_t)(now_ms - tripguard_camera_last_alert_ms) >=
             TRIPGUARD_CAMERA_ALERT_GUARD_MS))
        {
          tripguard_camera_alert_armed = 1U;
        }
      }
      else if (camera_person_latched == 0U)
      {
        camera_person_latched = 1U;
        if ((tripguard_camera_alert_armed != 0U) &&
            ((tripguard_radar_last_alert_ms == 0U) ||
             ((uint32_t)(now_ms - tripguard_radar_last_alert_ms) >=
              TRIPGUARD_RADAR_ALERT_GUARD_MS)))
        {
          tripguard_camera_alert_armed = 0U;
          tripguard_camera_last_alert_ms = now_ms;
          tripguard_camera_alert_count++;
          tripguard_person_alert_value =
              tripguard_camera_confidence_permille;
          TripGuard_QueueAudio(TRIPGUARD_AUDIO_RADAR_WARNING);
          TripGuard_QueueTelemetry(TRIPGUARD_TELEMETRY_SAFETY_EVENT);
          if (CellularTaskHandle != NULL)
          {
            (void)osThreadFlagsSet(CellularTaskHandle,
                                   TRIPGUARD_CELLULAR_FLAG_RADAR_SMS);
          }
        }
      }
    }

    tripguard_person_detected =
        ((radar_filtered_state != 0U) || (camera_raw_state != 0U)) ? 1U : 0U;

    /* Polling la duong du phong neu EXTI bi lo canh tren day nut dai. */
    if (((flags & TRIPGUARD_EVENT_SOS_PRESSED) != 0U) ||
        (tripguard_sos_button_raw != 0U))
    {
      if (sos_button_latched == 0U)
      {
        if (HAL_GPIO_ReadPin(SOS_BTN_GPIO_Port, SOS_BTN_Pin) == GPIO_PIN_RESET)
        {
          sos_button_latched = 1U;
          tripguard_sos_button_latched = 1U;
          sos_release_since_ms = 0U;
          (void)osEventFlagsClear(systemEventsHandle,
                                  TRIPGUARD_EVENT_SOS_PRESSED);

          /* Vua nhan PB1 la phat loa va xep SOS ngay; latch chong lap. */
          tripguard_sos_count++;
          tripguard_sos_pending = 1U;
          TripGuard_QueueAudio(TRIPGUARD_AUDIO_SOS_WARNING);
          TripGuard_QueueTelemetry(TRIPGUARD_TELEMETRY_SAFETY_EVENT);
          if (CellularTaskHandle != NULL)
          {
            (void)osThreadFlagsSet(CellularTaskHandle,
                                   TRIPGUARD_CELLULAR_FLAG_SOS);
          }
        }
      }
      else
      {
        /* Discard a bounce flag that arrived after the accepted edge. */
        (void)osEventFlagsClear(systemEventsHandle,
                                TRIPGUARD_EVENT_SOS_PRESSED);
      }
    }

    if (((flags & TRIPGUARD_EVENT_ACK_PRESSED) != 0U) ||
        (tripguard_ack_button_raw != 0U))
    {
      if (ack_button_latched == 0U)
      {
        osDelay(TRIPGUARD_BUTTON_DEBOUNCE_MS);
        if (HAL_GPIO_ReadPin(DRIVER_ACK_BTN_GPIO_Port,
                             DRIVER_ACK_BTN_Pin) == GPIO_PIN_RESET)
        {
          ack_button_latched = 1U;
          tripguard_ack_button_latched = 1U;
          ack_release_since_ms = 0U;
          (void)osEventFlagsClear(systemEventsHandle,
                                  TRIPGUARD_EVENT_ACK_PRESSED);
          tripguard_sos_pending = 0U;
          tripguard_driver_ack_count++;
          if (PiLink_GetCameraPerson() != 0U)
          {
            /* The driver button is the explicit OCCUPIED resolution input. */
            (void)TripGuard_Supervisor_PostOccupancyResolved();
          }
          TripGuard_QueueAudio(TRIPGUARD_AUDIO_STOP);
          TripGuard_QueueTelemetry(TRIPGUARD_TELEMETRY_SAFETY_EVENT);
        }
      }
      else
      {
        (void)osEventFlagsClear(systemEventsHandle,
                                TRIPGUARD_EVENT_ACK_PRESSED);
      }
    }
  }
  /* USER CODE END StartSafetyTask */
}

static void TripGuard_ServoWritePulse(uint16_t pulse_us)
{
  if (pulse_us < 1000U)
  {
    pulse_us = 1000U;
  }
  else if (pulse_us > 2000U)
  {
    pulse_us = 2000U;
  }

  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse_us);
  tripguard_servo_pulse_us = pulse_us;
}

static void TripGuard_ServoStop(void)
{
  TripGuard_ServoWritePulse(TRIPGUARD_SERVO_STOP_PULSE_US);
  tripguard_servo_moving = 0U;
}

static void TripGuard_ServoInit(void)
{
  servo_stop_deadline_ms = 0U;
  tripguard_servo_angle_deg = TRIPGUARD_SERVO_HOME_ANGLE_DEG;
  TripGuard_ServoStop();
}

static void TripGuard_ServoStart(uint16_t pulse_us,
                                 uint16_t logical_angle_deg,
                                 uint32_t now_ms)
{
  TripGuard_ServoWritePulse(pulse_us);
  tripguard_servo_angle_deg = logical_angle_deg;
  tripguard_servo_moving = 1U;
  tripguard_servo_move_count++;
  servo_stop_deadline_ms = now_ms + TRIPGUARD_SERVO_MOVE_TIME_MS;
}

static void TripGuard_ServoMoveToRadar(uint32_t now_ms)
{
  TripGuard_ServoStart(TRIPGUARD_SERVO_TO_RADAR_PULSE_US,
                       TRIPGUARD_SERVO_RADAR_ANGLE_DEG,
                       now_ms);
}

static void TripGuard_ServoMoveToHome(uint32_t now_ms)
{
  TripGuard_ServoStart(TRIPGUARD_SERVO_TO_HOME_PULSE_US,
                       TRIPGUARD_SERVO_HOME_ANGLE_DEG,
                       now_ms);
}

static void TripGuard_ServoUpdate(uint32_t now_ms)
{
  if ((tripguard_servo_moving != 0U) &&
      (TripGuard_TimeReached(now_ms, servo_stop_deadline_ms) != 0U))
  {
    TripGuard_ServoStop();
  }
}

/* USER CODE BEGIN Header_StartCellularTask */
/**
* @brief Function implementing the CellularTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCellularTask */
void StartCellularTask(void *argument)
{
  /* USER CODE BEGIN StartCellularTask */
  uint32_t telemetry_event;
  TripGuard_TelemetryEvent_t telemetry_kind = TRIPGUARD_TELEMETRY_PERIODIC;
  uint32_t last_diagnostics_ms = 0U;
  uint32_t last_publish_attempt_ms = 0U;
  uint32_t cellular_rail_epoch = tripguard_power_rail_epoch;
  TripGuard_SupervisorRpcResult_t supervisor_rpc_result;
  uint8_t supervisor_rpc_result_pending = 0U;

  (void)argument;
  for(;;)
  {
#if (TRIPGUARD_CONNECTIVITY_OWNER_PI != 0U)
    uint32_t thread_flags = osThreadFlagsGet();

    /* Pi owns SIM/ThingsBoard.  STM32 only queues durable UART requests. */
    if ((thread_flags & osFlagsError) == 0U)
    {
      if ((thread_flags & TRIPGUARD_CELLULAR_FLAG_SOS) != 0U)
      {
        (void)osThreadFlagsClear(TRIPGUARD_CELLULAR_FLAG_SOS);
        tripguard_last_publish_queued =
            PiLink_SendAlertRequest((uint8_t)TRIPGUARD_ALERT_SOS_SMS_CALL,
                                    0U);
        tripguard_alert_last_kind =
            (uint8_t)TRIPGUARD_ALERT_SOS_SMS_CALL;
        tripguard_alert_last_success = tripguard_last_publish_queued;
        if (tripguard_last_publish_queued == 0U)
        {
          tripguard_alert_error_count++;
        }
      }
      if ((thread_flags & TRIPGUARD_CELLULAR_FLAG_RADAR_SMS) != 0U)
      {
        (void)osThreadFlagsClear(TRIPGUARD_CELLULAR_FLAG_RADAR_SMS);
        tripguard_last_publish_queued =
            PiLink_SendAlertRequest((uint8_t)TRIPGUARD_ALERT_RADAR_SMS,
                                    tripguard_person_alert_value);
        tripguard_alert_last_kind = (uint8_t)TRIPGUARD_ALERT_RADAR_SMS;
        tripguard_alert_last_success = tripguard_last_publish_queued;
        if (tripguard_last_publish_queued == 0U)
        {
          tripguard_alert_error_count++;
        }
      }
    }

    while (osMessageQueueGet(telemetryEventQueueHandle,
                             &telemetry_event,
                             NULL,
                             0U) == osOK)
    {
      TripGuard_TelemetryEvent_t received_event =
          (TripGuard_TelemetryEvent_t)telemetry_event;

      if (tripguard_telemetry_pending != 0U)
      {
        tripguard_telemetry_coalesced_count++;
      }
      if ((received_event == TRIPGUARD_TELEMETRY_REAR_CONFIRM) ||
          (tripguard_telemetry_pending == 0U) ||
          ((received_event == TRIPGUARD_TELEMETRY_SAFETY_EVENT) &&
           (telemetry_kind == TRIPGUARD_TELEMETRY_PERIODIC)))
      {
        telemetry_kind = received_event;
      }
      tripguard_telemetry_pending = 1U;
    }

    if ((tripguard_telemetry_pending != 0U) &&
        (PiLink_IsOnline() != 0U) &&
        (PiLink_IsReady() != 0U) &&
        ((uint32_t)(HAL_GetTick() - last_publish_attempt_ms) >=
         TRIPGUARD_TELEMETRY_RETRY_MS))
    {
      size_t telemetry_length;

      last_publish_attempt_ms = HAL_GetTick();
      TripGuard_BuildTelemetry(rtos_telemetry_json,
                               sizeof(rtos_telemetry_json),
                               telemetry_kind);
      telemetry_length = strlen(rtos_telemetry_json);
      tripguard_last_publish_queued =
          PiLink_QueueTelemetryJson(
              rtos_telemetry_json,
              (uint16_t)((telemetry_length > UINT16_MAX) ?
                         UINT16_MAX : telemetry_length));
      if (tripguard_last_publish_queued != 0U)
      {
        tripguard_telemetry_pending = 0U;
        telemetry_kind = TRIPGUARD_TELEMETRY_PERIODIC;
      }
    }

    osDelay(5U);
    continue;
#endif

    if (TripGuard_Power_IsModemOn() == 0U)
    {
      osDelay(20U);
      continue;
    }
    if (cellular_rail_epoch != tripguard_power_rail_epoch)
    {
      (void)HAL_UART_AbortReceive(&huart2);
      A7670_Init(&huart2);
      TB_MQTT_Init();
      cellular_rail_epoch = tripguard_power_rail_epoch;
    }
    A7670_Task();

    /*
     * MQTT phai tiep tuc chay trong giai doan ALERT_WAIT_NETWORK de hoan tat
     * transaction dang do va cleanup an toan. Khi alert da lay exclusive
     * session thi MQTT dung han.
     */
    if (A7670_IsExclusiveSessionActive() == 0U)
    {
     TB_MQTT_Task();
    }

    if ((supervisor_rpc_result_pending == 0U) &&
        (TripGuard_Supervisor_TakeRpcResult(
            &supervisor_rpc_result) != 0U))
    {
      supervisor_rpc_result_pending = 1U;
    }
    if ((supervisor_rpc_result_pending != 0U) &&
        (alert_kind == TRIPGUARD_ALERT_NONE) &&
        (A7670_IsExclusiveSessionActive() == 0U) &&
        (TB_MQTT_PublishRpcResponse(supervisor_rpc_result.request_id,
                                    supervisor_rpc_result.json) != 0U))
    {
      TripGuard_SetRpcResult("supervisor",
          (supervisor_rpc_result.success != 0U) ?
          "RPC_RESPONSE_OK" : "RPC_RESPONSE_REJECTED");
      supervisor_rpc_result_pending = 0U;
    }
    TripGuard_HandleRpcCommand();
    TripGuard_AlertStep();

    if ((HAL_GetTick() - last_diagnostics_ms) >= TRIPGUARD_DIAGNOSTICS_PERIOD_MS)
    {
      last_diagnostics_ms = HAL_GetTick();
      TripGuard_UpdateDiagnostics();
    }

    while (osMessageQueueGet(telemetryEventQueueHandle,
                             &telemetry_event,
                             NULL,
                             0U) == osOK)
    {
      TripGuard_TelemetryEvent_t received_event =
          (TripGuard_TelemetryEvent_t)telemetry_event;

      if (tripguard_telemetry_pending != 0U)
      {
        tripguard_telemetry_coalesced_count++;
      }

      /* REAR_CONFIRM has the highest priority; periodic is the lowest. */
      if ((received_event == TRIPGUARD_TELEMETRY_REAR_CONFIRM) ||
          (tripguard_telemetry_pending == 0U) ||
          ((received_event == TRIPGUARD_TELEMETRY_SAFETY_EVENT) &&
           (telemetry_kind == TRIPGUARD_TELEMETRY_PERIODIC)))
      {
        telemetry_kind = received_event;
      }
      tripguard_telemetry_pending = 1U;
    }

    /* Retain one coalesced snapshot until MQTT can actually accept it. */
    if ((tripguard_telemetry_pending != 0U) &&
        (alert_kind == TRIPGUARD_ALERT_NONE) &&
        (A7670_IsExclusiveSessionActive() == 0U) &&
        (TB_MQTT_IsConnected() != 0U) &&
        ((uint32_t)(HAL_GetTick() - last_publish_attempt_ms) >=
         TRIPGUARD_TELEMETRY_RETRY_MS))
    {
      last_publish_attempt_ms = HAL_GetTick();
      TripGuard_BuildTelemetry(rtos_telemetry_json,
                               sizeof(rtos_telemetry_json),
                               telemetry_kind);
      tripguard_last_publish_queued =
          TB_MQTT_PublishTelemetry(rtos_telemetry_json);
      if (tripguard_last_publish_queued != 0U)
      {
        tripguard_telemetry_pending = 0U;
        telemetry_kind = TRIPGUARD_TELEMETRY_PERIODIC;
      }
    }

    osDelay(5U);
  }
  /* USER CODE END StartCellularTask */
}

/* USER CODE BEGIN Header_StartGpsTask */
/**
* @brief Function implementing the GpsTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGpsTask */
void StartGpsTask(void *argument)
{
  /* USER CODE BEGIN StartGpsTask */
  uint32_t gps_rail_epoch = tripguard_power_rail_epoch;

  (void)argument;
  for(;;)
  {
    if (TripGuard_Power_IsGpsOn() == 0U)
    {
      osDelay(20U);
      continue;
    }
    if (gps_rail_epoch != tripguard_power_rail_epoch)
    {
      (void)HAL_UART_AbortReceive(&huart1);
      GPS_Init(&huart1);
      gps_rail_epoch = tripguard_power_rail_epoch;
    }
    if (osMutexAcquire(gpsDataMutexHandle, 20U) == osOK)
    {
      GPS_Task();
      (void)osMutexRelease(gpsDataMutexHandle);
    }
    osDelay(5U);
  }
  /* USER CODE END StartGpsTask */
}

/* USER CODE BEGIN Header_StartTelemetryTask */
/**
* @brief Function implementing the TelemetryTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTelemetryTask */
void StartTelemetryTask(void *argument)
{
  /* USER CODE BEGIN StartTelemetryTask */
  uint32_t next_publish_tick;

  (void)argument;
  next_publish_tick = osKernelGetTickCount() + 1000U;
  for(;;)
  {
    (void)osDelayUntil(next_publish_tick);
    TripGuard_QueueTelemetry(TRIPGUARD_TELEMETRY_PERIODIC);
    next_publish_tick += tripguard_telemetry_period_ms;
  }
  /* USER CODE END StartTelemetryTask */
}

/* USER CODE BEGIN Header_StartAudioTask */
/**
* @brief Function implementing the AudioTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartAudioTask */
void StartAudioTask(void *argument)
{
  /* USER CODE BEGIN StartAudioTask */
  uint32_t audio_command;
  uint32_t audio_rail_epoch = tripguard_power_rail_epoch;

  (void)argument;
  for(;;)
  {
    if (TripGuard_Power_IsAudioOn() == 0U)
    {
      osDelay(20U);
      continue;
    }
    if (audio_rail_epoch != tripguard_power_rail_epoch)
    {
      JQ8900_Init(JQ8900_IO1_GPIO_Port, JQ8900_IO1_Pin);
      audio_rail_epoch = tripguard_power_rail_epoch;
    }
    if (osMessageQueueGet(audioQueueHandle,
                          &audio_command,
                          NULL,
                          osWaitForever) == osOK)
    {
      uint8_t command_ok = 0U;

      if (audio_command == (uint32_t)TRIPGUARD_AUDIO_STOP)
      {
        command_ok = JQ8900_Stop();
        if (command_ok != 0U)
        {
          tripguard_audio_track = 0U;
        }
      }
      else
      {
        command_ok = JQ8900_PlayTrack((uint16_t)audio_command);
        if (command_ok != 0U)
        {
          tripguard_audio_track = (uint16_t)audio_command;
        }
      }

      if (command_ok == 0U)
      {
        tripguard_audio_error_count++;
      }
    }
  }
  /* USER CODE END StartAudioTask */
}

/* USER CODE BEGIN Header_StartHealthTask */
/**
* @brief Function implementing the HealthTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartHealthTask */
void StartHealthTask(void *argument)
{
  /* USER CODE BEGIN StartHealthTask */
  (void)argument;
  for(;;)
  {
    const GPS_Data_t *gps_data;

    tripguard_audio_busy_raw =
        (TripGuard_Power_IsAudioOn() != 0U) ?
        JQ8900_IsBusyRaw() : 0U;
    if (osMutexAcquire(gpsDataMutexHandle, 20U) == osOK)
    {
      gps_data = GPS_GetData();
      tripguard_gps_state = gps_data->state;
      tripguard_gps_comm_state = gps_data->comm_state;
      tripguard_speed_kmh = gps_data->speed_kmh;
      (void)TripGuard_Supervisor_PostGps(
          (float)gps_data->latitude,
          (float)gps_data->longitude,
          gps_data->speed_kmh,
          (uint8_t)(gps_data->fix_valid && TripGuard_Power_IsGpsOn()),
          gps_data->location_age_ms);

      if ((gps_data->state == GPS_FIXED) ||
          (gps_data->state == GPS_LAST_VALID))
      {
        tripguard_driving_state =
            (gps_data->speed_kmh >= 3.0f) ?
            TRIPGUARD_DRIVING_MOVING : TRIPGUARD_DRIVING_STOPPED;
      }
      else
      {
        tripguard_driving_state = TRIPGUARD_DRIVING_UNKNOWN;
      }
      (void)osMutexRelease(gpsDataMutexHandle);
    }

    osDelay(250U);
  }
  /* USER CODE END StartHealthTask */
}

/* USER CODE BEGIN Header_StartGatewayLinkTask */
/**
* @brief Function implementing the SPI/BLE gateway transport owner.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGatewayLinkTask */
void StartGatewayLinkTask(void *argument)
{
  /* USER CODE BEGIN StartGatewayLinkTask */
  if (tripguard_gateway_init_ok != 0U)
  {
    TripGuard_Gateway_Task(argument);
  }

  /* Fail safe: GSM/SMS/MQTT and all local safety functions keep running. */
  for (;;)
  {
    osDelay(1000U);
  }
  /* USER CODE END StartGatewayLinkTask */
}

void StartPowerManagerTask(void *argument)
{
  TripGuard_Power_Task(argument);
}

/**
* @brief Single owner of the TripGuard logical activation state machine.
*/
void StartTripGuardSupervisorTask(void *argument)
{
  TripGuard_Supervisor_Task(argument);
}

/**
* @brief Single owner of Raspberry Pi UART parser and TX queue.
*/
void StartPiLinkTask(void *argument)
{
  (void)argument;
  for (;;)
  {
    if (PiLink_Init(&huart6, xTaskGetCurrentTaskHandle()) != 0U)
    {
      PiLink_Task(NULL);
    }
    osDelay(1000U);
  }
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

static uint8_t TripGuard_TimeReached(uint32_t now_ms, uint32_t deadline_ms)
{
  return (uint8_t)(((int32_t)(now_ms - deadline_ms)) >= 0);
}

static void TripGuard_CopyVolatileText(volatile char *destination,
                                       size_t destination_size,
                                       const char *source)
{
  size_t index = 0U;

  if ((destination == NULL) || (destination_size == 0U))
  {
    return;
  }
  if (source != NULL)
  {
    while ((source[index] != '\0') && ((index + 1U) < destination_size))
    {
      destination[index] = source[index];
      index++;
    }
  }
  destination[index] = '\0';
}

static void TripGuard_SetRpcResult(const char *method, const char *result)
{
  TripGuard_CopyVolatileText(tripguard_last_rpc_method,
                             sizeof(tripguard_last_rpc_method), method);
  TripGuard_CopyVolatileText(tripguard_last_rpc_result,
                             sizeof(tripguard_last_rpc_result), result);
}

static uint8_t TripGuard_RpcBool(const char *params, uint8_t *value)
{
  if ((params == NULL) || (value == NULL))
  {
    return 0U;
  }
  if ((strstr(params, "true") != NULL) ||
      (strstr(params, "TRUE") != NULL) ||
      (strcmp(params, "1") == 0))
  {
    *value = 1U;
    return 1U;
  }
  if ((strstr(params, "false") != NULL) ||
      (strstr(params, "FALSE") != NULL) ||
      (strcmp(params, "0") == 0))
  {
    *value = 0U;
    return 1U;
  }
  return 0U;
}

static uint8_t TripGuard_RpcUint32(const char *params, uint32_t *value)
{
  const char *cursor;
  char *end_pointer;
  unsigned long parsed;

  if ((params == NULL) || (value == NULL))
  {
    return 0U;
  }
  cursor = params;
  while ((*cursor != '\0') && ((*cursor < '0') || (*cursor > '9')))
  {
    cursor++;
  }
  if (*cursor == '\0')
  {
    return 0U;
  }
  parsed = strtoul(cursor, &end_pointer, 10);
  if (end_pointer == cursor)
  {
    return 0U;
  }
  *value = (uint32_t)parsed;
  return 1U;
}

static void TripGuard_HandleRpcCommand(void)
{
  TB_RPC_Command_t command;
  uint32_t parameter;
  uint8_t enabled;

  if (TB_MQTT_TakeRpcCommand(&command) == 0U)
  {
    return;
  }
  tripguard_remote_command_count++;

  if (strcmp(command.method, "setSystemArmed") == 0)
  {
    if (TripGuard_RpcBool(command.params, &enabled) == 0U)
    {
      tripguard_remote_rejected_count++;
      (void)TripGuard_Supervisor_QueueRpcError(
          command.id, "invalid_boolean_params");
      TripGuard_SetRpcResult(command.method, "INVALID_BOOL_PARAMS");
    }
    else if (TripGuard_Supervisor_PostRpcSetArmed(
                 command.id, enabled) == 0U)
    {
      tripguard_remote_rejected_count++;
      (void)TripGuard_Supervisor_QueueRpcError(
          command.id, "supervisor_queue_full");
      TripGuard_SetRpcResult(command.method, "QUEUE_FULL");
    }
    else
    {
      TripGuard_SetRpcResult(command.method, "QUEUED_FOR_APPLY");
    }
  }
  else if (strcmp(command.method, "cancelArm") == 0)
  {
    if (TripGuard_Supervisor_PostRpcCancelArm(command.id) == 0U)
    {
      tripguard_remote_rejected_count++;
      (void)TripGuard_Supervisor_QueueRpcError(
          command.id, "supervisor_queue_full");
      TripGuard_SetRpcResult(command.method, "QUEUE_FULL");
    }
    else
    {
      TripGuard_SetRpcResult(command.method, "QUEUED_FOR_APPLY");
    }
  }
  else if (strcmp(command.method, "getSystemState") == 0)
  {
    if (TripGuard_Supervisor_PostRpcGetState(command.id) == 0U)
    {
      tripguard_remote_rejected_count++;
      (void)TripGuard_Supervisor_QueueRpcError(
          command.id, "supervisor_queue_full");
      TripGuard_SetRpcResult(command.method, "QUEUE_FULL");
    }
    else
    {
      TripGuard_SetRpcResult(command.method, "QUEUED_FOR_READ");
    }
  }
  else if (strcmp(command.method, "playAlarm") == 0)
  {
    if ((TripGuard_RpcUint32(command.params, &parameter) == 0U) ||
        (parameter != (uint32_t)TRIPGUARD_AUDIO_RADAR_WARNING))
    {
      parameter = (uint32_t)TRIPGUARD_AUDIO_SOS_WARNING;
    }
    TripGuard_QueueAudio((TripGuard_AudioCommand_t)parameter);
    TripGuard_SetRpcResult(command.method, "OK_AUDIO_QUEUED");
  }
  else if (strcmp(command.method, "stopAudio") == 0)
  {
    TripGuard_QueueAudio(TRIPGUARD_AUDIO_STOP);
    TripGuard_SetRpcResult(command.method, "OK_AUDIO_STOP_QUEUED");
  }
  else if (strcmp(command.method, "setRadarAlerts") == 0)
  {
    if (TripGuard_RpcBool(command.params, &enabled) != 0U)
    {
      tripguard_radar_alert_enabled = enabled;
      TripGuard_SetRpcResult(command.method,
                             (enabled != 0U) ? "OK_ENABLED" : "OK_DISABLED");
    }
    else
    {
      tripguard_remote_rejected_count++;
      TripGuard_SetRpcResult(command.method, "INVALID_BOOL_PARAMS");
    }
  }
  else if (strcmp(command.method, "acknowledgeSOS") == 0)
  {
    tripguard_sos_pending = 0U;
    TripGuard_QueueAudio(TRIPGUARD_AUDIO_STOP);
    TripGuard_SetRpcResult(command.method, "OK_ACKNOWLEDGED");
  }
  else if (strcmp(command.method, "sendRadarSms") == 0)
  {
    if (alert_kind == TRIPGUARD_ALERT_NONE)
    {
      if (CellularTaskHandle != NULL)
      {
        (void)osThreadFlagsSet(CellularTaskHandle,
                               TRIPGUARD_CELLULAR_FLAG_RADAR_SMS);
      }
      TripGuard_SetRpcResult(command.method, "OK_SMS_QUEUED");
    }
    else
    {
      tripguard_remote_rejected_count++;
      TripGuard_SetRpcResult(command.method, "BUSY_ALERT_ACTIVE");
    }
  }
  else if (strcmp(command.method, "triggerSOS") == 0)
  {
    if (alert_kind != TRIPGUARD_ALERT_SOS_SMS_CALL)
    {
      tripguard_sos_count++;
      tripguard_sos_pending = 1U;
      TripGuard_QueueAudio(TRIPGUARD_AUDIO_SOS_WARNING);
      if (CellularTaskHandle != NULL)
      {
        (void)osThreadFlagsSet(CellularTaskHandle,
                               TRIPGUARD_CELLULAR_FLAG_SOS);
      }
      TripGuard_SetRpcResult(command.method, "OK_SOS_QUEUED");
    }
    else
    {
      tripguard_remote_rejected_count++;
      TripGuard_SetRpcResult(command.method, "BUSY_SOS_ACTIVE");
    }
  }
  else if (strcmp(command.method, "setTelemetryPeriod") == 0)
  {
    if ((TripGuard_RpcUint32(command.params, &parameter) != 0U) &&
        (parameter >= TRIPGUARD_TELEMETRY_PERIOD_MIN_MS) &&
        (parameter <= TRIPGUARD_TELEMETRY_PERIOD_MAX_MS))
    {
      tripguard_telemetry_period_ms = parameter;
      TripGuard_SetRpcResult(command.method, "OK_PERIOD_UPDATED");
    }
    else
    {
      tripguard_remote_rejected_count++;
      TripGuard_SetRpcResult(command.method, "INVALID_PERIOD_2000_60000");
    }
  }
  else if (strcmp(command.method, "refreshTelemetry") == 0)
  {
    TripGuard_SetRpcResult(command.method, "OK_REFRESH_QUEUED");
  }
  else if (strcmp(command.method, "cameraToRadar") == 0)
  {
    if (TripGuard_Supervisor_IsDetectionActive() != 0U)
    {
      TripGuard_ServoMoveToRadar(HAL_GetTick());
      tripguard_servo_tracking_radar = 1U;
      TripGuard_SetRpcResult(command.method, "OK_CAMERA_TO_RADAR");
    }
    else
    {
      tripguard_remote_rejected_count++;
      TripGuard_SetRpcResult(command.method, "SYSTEM_NOT_ACTIVE");
    }
  }
  else if (strcmp(command.method, "cameraHome") == 0)
  {
    if (TripGuard_Supervisor_IsDetectionActive() != 0U)
    {
      TripGuard_ServoMoveToHome(HAL_GetTick());
      tripguard_servo_tracking_radar = 0U;
      TripGuard_SetRpcResult(command.method, "OK_CAMERA_HOME");
    }
    else
    {
      tripguard_remote_rejected_count++;
      TripGuard_SetRpcResult(command.method, "SYSTEM_NOT_ACTIVE");
    }
  }
  else if (strcmp(command.method, "stopServo") == 0)
  {
    TripGuard_ServoStop();
    TripGuard_SetRpcResult(command.method, "OK_SERVO_STOPPED");
  }
  else
  {
    tripguard_remote_rejected_count++;
    TripGuard_SetRpcResult(command.method, "UNKNOWN_METHOD");
  }

  TripGuard_QueueTelemetry(TRIPGUARD_TELEMETRY_SAFETY_EVENT);
}

/*
 * Dung dung kieu SMS text mode da chay on dinh trong project test rieng:
 * charset GSM, so dien thoai dang thuong va noi dung ASCII khong dau.
 */
static const char *TripGuard_GetAlertSmsText(void)
{
  return (alert_kind == TRIPGUARD_ALERT_SOS_SMS_CALL) ?
         TRIPGUARD_SOS_SMS_TEXT : TRIPGUARD_RADAR_SMS_TEXT;
}

static void TripGuard_AlertCopyResponse(const char *source)
{
  char *destination = (char *)(uintptr_t)tripguard_alert_last_response;

  if (source == NULL)
  {
    destination[0] = '\0';
    return;
  }

  strncpy(destination, source, sizeof(tripguard_alert_last_response) - 1U);
  destination[sizeof(tripguard_alert_last_response) - 1U] = '\0';
}

static uint8_t TripGuard_AlertTakeResult(A7670_CommandResult_t *result)
{
  A7670_CommandResult_t current = A7670_GetCommandResult();

  if ((current == A7670_COMMAND_IDLE) ||
      (current == A7670_COMMAND_PENDING))
  {
    return 0U;
  }

  tripguard_alert_last_result = current;
  TripGuard_AlertCopyResponse(A7670_GetCommandResponse());
  if (result != NULL)
  {
    *result = current;
  }
  A7670_ClearCommandResult();
  return 1U;
}

static void TripGuard_AlertBegin(TripGuard_AlertKind_t kind)
{
  /* MQTT se tu phuc hoi; alert chiem transaction AT ngay lap tuc. */
  TB_MQTT_AbortForEmergency();
  alert_kind = kind;
  alert_phase = ALERT_WAIT_NETWORK;
  alert_sequence_ok = 1U;
  alert_deadline_ms = HAL_GetTick() + TRIPGUARD_ALERT_NETWORK_WAIT_MS;
  tripguard_alert_active = (uint8_t)kind;
  tripguard_alert_phase = (uint8_t)alert_phase;
  TripGuard_AlertCopyResponse("");
}

static void TripGuard_AlertFinish(void)
{
  TripGuard_AlertKind_t completed_kind = alert_kind;

  /* Neu chua vao exclusive thi MQTT van phai quiesce xong truoc khi resume. */
  if ((A7670_IsExclusiveSessionActive() == 0U) &&
      (TB_MQTT_IsQuiesced() == 0U))
  {
    return;
  }

  if (A7670_IsExclusiveSessionActive() != 0U)
  {
    A7670_EndExclusiveSession();
    if (A7670_IsExclusiveSessionActive() != 0U)
    {
      /* Van con transaction AT: khong resume MQTT/doi owner luc nay. */
      return;
    }
  }

  tripguard_alert_last_kind = (uint8_t)completed_kind;
  tripguard_alert_last_success = alert_sequence_ok;
  tripguard_alert_active = (uint8_t)TRIPGUARD_ALERT_NONE;
  tripguard_alert_phase = (uint8_t)ALERT_IDLE;
  alert_kind = TRIPGUARD_ALERT_NONE;
  alert_phase = ALERT_IDLE;

  /*
   * SOS den trong luc radar dang SMS khong duoc A7670_CancelCommand().
   * Hoan thanh giao dich hien tai, sau do nang cap sang chuoi SOS.
   */
  if ((alert_sos_pending != 0U) &&
      (completed_kind != TRIPGUARD_ALERT_SOS_SMS_CALL))
  {
    alert_sos_pending = 0U;
    TripGuard_AlertBegin(TRIPGUARD_ALERT_SOS_SMS_CALL);
    return;
  }

  alert_sos_pending = 0U;
  TB_MQTT_ResumeAfterEmergency();

  /* Cap nhat ThingsBoard sau khi MQTT tu ket noi lai. */
  TripGuard_QueueTelemetry(TRIPGUARD_TELEMETRY_SAFETY_EVENT);
}

static void TripGuard_AlertSmsDone(uint8_t success)
{
  if (success != 0U)
  {
    tripguard_sms_sent_count++;
  }
  else
  {
    alert_sequence_ok = 0U;
    tripguard_alert_error_count++;
  }

  /* Luon ve GSM truoc. SOS se goi sau buoc restore; radar ket thuc tai do. */
  alert_phase = ALERT_RESTORE_CHARSET_START;
}

static void TripGuard_AlertStep(void)
{
  uint32_t thread_flags;
  uint32_t now_ms;
  A7670_CommandResult_t result;
  const char *sms_text;
  static const uint8_t sms_end = 0x1AU;

  thread_flags = osThreadFlagsGet();
  if ((thread_flags & osFlagsError) == 0U)
  {
    /*
     * SOS uu tien cao, nhung khong duoc xoa transaction AT dang PENDING.
     * Neu radar dang SMS, ghi nho SOS va chuyen sang SOS ngay khi chuoi AT
     * hien tai ket thuc an toan.
     */
    if ((thread_flags & TRIPGUARD_CELLULAR_FLAG_SOS) != 0U)
    {
      (void)osThreadFlagsClear(TRIPGUARD_CELLULAR_FLAG_SOS);
      if (alert_kind == TRIPGUARD_ALERT_NONE)
      {
        TripGuard_AlertBegin(TRIPGUARD_ALERT_SOS_SMS_CALL);
      }
      else if (alert_kind != TRIPGUARD_ALERT_SOS_SMS_CALL)
      {
        alert_sos_pending = 1U;
      }
    }
    else if ((alert_kind == TRIPGUARD_ALERT_NONE) &&
             ((thread_flags & TRIPGUARD_CELLULAR_FLAG_RADAR_SMS) != 0U))
    {
      (void)osThreadFlagsClear(TRIPGUARD_CELLULAR_FLAG_RADAR_SMS);
      TripGuard_AlertBegin(TRIPGUARD_ALERT_RADAR_SMS);
    }
  }

  if (alert_kind == TRIPGUARD_ALERT_NONE)
  {
    return;
  }

  now_ms = HAL_GetTick();
  switch (alert_phase)
  {
    case ALERT_WAIT_NETWORK:
      if ((A7670_IsRegistered() != 0U) &&
          (TB_MQTT_IsQuiesced() != 0U) &&
          (A7670_BeginExclusiveSession() != 0U))
      {
        alert_phase = ALERT_SMS_MODE_START;
      }
      else if ((TB_MQTT_IsQuiesced() != 0U) &&
               (TripGuard_TimeReached(now_ms, alert_deadline_ms) != 0U))
      {
        tripguard_alert_last_result = A7670_COMMAND_TIMEOUT;
        TripGuard_AlertCopyResponse("ALERT: network/MQTT quiesce timeout");
        alert_sequence_ok = 0U;
        tripguard_alert_error_count++;
        TripGuard_AlertFinish();
      }
      break;

    case ALERT_SMS_MODE_START:
      if (A7670_CommandStart("AT+CMGF=1", NULL,
                            TRIPGUARD_ALERT_COMMAND_TIMEOUT_MS) != 0U)
      {
        alert_phase = ALERT_SMS_MODE_WAIT;
      }
      break;

    case ALERT_SMS_MODE_WAIT:
      if (TripGuard_AlertTakeResult(&result) != 0U)
      {
        alert_phase = (result == A7670_COMMAND_SUCCESS) ?
                      ALERT_SMS_CHARSET_START : alert_phase;
        if (result != A7670_COMMAND_SUCCESS)
        {
          TripGuard_AlertSmsDone(0U);
        }
      }
      break;

    case ALERT_SMS_CHARSET_START:
      if (A7670_CommandStart("AT+CSCS=\"GSM\"", NULL,
                            TRIPGUARD_ALERT_COMMAND_TIMEOUT_MS) != 0U)
      {
        alert_phase = ALERT_SMS_CHARSET_WAIT;
      }
      break;

    case ALERT_SMS_CHARSET_WAIT:
      if (TripGuard_AlertTakeResult(&result) != 0U)
      {
        alert_phase = (result == A7670_COMMAND_SUCCESS) ?
                      ALERT_SMS_DCS_START : alert_phase;
        if (result != A7670_COMMAND_SUCCESS)
        {
          TripGuard_AlertSmsDone(0U);
        }
      }
      break;

    case ALERT_SMS_DCS_START:
      /* DCS=0: noi dung GSM 7-bit, giong project test dang gui duoc. */
      if (A7670_CommandStart("AT+CSMP=17,167,0,0", NULL,
                            TRIPGUARD_ALERT_COMMAND_TIMEOUT_MS) != 0U)
      {
        alert_phase = ALERT_SMS_DCS_WAIT;
      }
      break;

    case ALERT_SMS_DCS_WAIT:
      if (TripGuard_AlertTakeResult(&result) != 0U)
      {
        alert_phase = (result == A7670_COMMAND_SUCCESS) ?
                      ALERT_SMS_PROMPT_START : alert_phase;
        if (result != A7670_COMMAND_SUCCESS)
        {
          TripGuard_AlertSmsDone(0U);
        }
      }
      break;

    case ALERT_SMS_PROMPT_START:
      (void)snprintf(alert_command,
                     sizeof(alert_command),
                     "AT+CMGS=\"%s\"",
                     TRIPGUARD_EMERGENCY_NUMBER);
      if (A7670_CommandStart(alert_command, ">",
                            TRIPGUARD_ALERT_SMS_TIMEOUT_MS) != 0U)
      {
        alert_phase = ALERT_SMS_PROMPT_WAIT;
      }
      break;

    case ALERT_SMS_PROMPT_WAIT:
      if (TripGuard_AlertTakeResult(&result) != 0U)
      {
        alert_phase = (result == A7670_COMMAND_SUCCESS) ?
                      ALERT_SMS_BODY_START : alert_phase;
        if (result != A7670_COMMAND_SUCCESS)
        {
          TripGuard_AlertSmsDone(0U);
        }
      }
      break;

    case ALERT_SMS_BODY_START:
      sms_text = TripGuard_GetAlertSmsText();
      if ((sms_text == NULL) || (sms_text[0] == '\0'))
      {
        /* Config khong co noi dung SMS hop le: khong mo +CMGS transaction. */
        TripGuard_AlertSmsDone(0U);
        break;
      }

      if (A7670_WaitResponse("+CMGS:",
                             TRIPGUARD_ALERT_SMS_TIMEOUT_MS) != 0U)
      {
        if ((A7670_SendRaw(
                 (const uint8_t *)sms_text,
                 (uint16_t)strlen(sms_text)) != 0U) &&
            (A7670_SendRaw(&sms_end, 1U) != 0U))
        {
          alert_phase = ALERT_SMS_BODY_WAIT;
        }
        else
        {
          /*
           * Da mo transaction +CMGS thi khong xoa state STM32 neu TX raw loi.
           * De transaction cho terminal response/timeout; neu xoa ngay, lenh
           * restore tiep theo co the chen vao luc modem van dang doi SMS body.
           */
          alert_sequence_ok = 0U;
          alert_phase = ALERT_SMS_BODY_WAIT;
        }
      }
      break;

    case ALERT_SMS_BODY_WAIT:
      if (TripGuard_AlertTakeResult(&result) != 0U)
      {
        TripGuard_AlertSmsDone(
            (uint8_t)(result == A7670_COMMAND_SUCCESS));
      }
      break;

    case ALERT_CALL_START:
      (void)snprintf(alert_command,
                     sizeof(alert_command),
                     "ATD%s;",
                     TRIPGUARD_EMERGENCY_NUMBER);
      if (A7670_CommandStart(alert_command, NULL,
                            TRIPGUARD_ALERT_DIAL_TIMEOUT_MS) != 0U)
      {
        alert_phase = ALERT_CALL_WAIT;
      }
      break;

    case ALERT_CALL_WAIT:
      if (TripGuard_AlertTakeResult(&result) != 0U)
      {
        if (result == A7670_COMMAND_SUCCESS)
        {
          tripguard_call_started_count++;
          alert_deadline_ms = now_ms + TRIPGUARD_SOS_CALL_DURATION_MS;
          alert_phase = ALERT_CALL_ACTIVE;
        }
        else
        {
          alert_sequence_ok = 0U;
          tripguard_alert_error_count++;
          TripGuard_AlertFinish();
        }
      }
      break;

    case ALERT_CALL_ACTIVE:
      if (TripGuard_TimeReached(now_ms, alert_deadline_ms) != 0U)
      {
        alert_phase = ALERT_HANGUP_START;
      }
      break;

    case ALERT_HANGUP_START:
      if (A7670_CommandStart("ATH", NULL,
                            TRIPGUARD_ALERT_COMMAND_TIMEOUT_MS) != 0U)
      {
        alert_phase = ALERT_HANGUP_WAIT;
      }
      break;

    case ALERT_HANGUP_WAIT:
      if (TripGuard_AlertTakeResult(&result) != 0U)
      {
        if (result != A7670_COMMAND_SUCCESS)
        {
          alert_sequence_ok = 0U;
          tripguard_alert_error_count++;
        }
        TripGuard_AlertFinish();
      }
      break;

    case ALERT_RESTORE_CHARSET_START:
      if (A7670_CommandStart("AT+CSCS=\"GSM\"", NULL,
                            TRIPGUARD_ALERT_COMMAND_TIMEOUT_MS) != 0U)
      {
        alert_phase = ALERT_RESTORE_CHARSET_WAIT;
      }
      break;

    case ALERT_RESTORE_CHARSET_WAIT:
      if (TripGuard_AlertTakeResult(&result) != 0U)
      {
        if (result != A7670_COMMAND_SUCCESS)
        {
          alert_sequence_ok = 0U;
          tripguard_alert_error_count++;
        }
        if (alert_kind == TRIPGUARD_ALERT_SOS_SMS_CALL)
        {
          /* Chi SOS di tiep den voice call; radar chi SMS. */
          alert_phase = ALERT_CALL_START;
        }
        else
        {
          TripGuard_AlertFinish();
        }
      }
      break;

    case ALERT_IDLE:
    default:
      alert_sequence_ok = 0U;
      tripguard_alert_error_count++;
      TripGuard_AlertFinish();
      break;
  }

  tripguard_alert_phase = (uint8_t)alert_phase;
}

static void TripGuard_QueueAudio(TripGuard_AudioCommand_t command)
{
  uint32_t message = (uint32_t)command;

  (void)osMessageQueuePut(audioQueueHandle, &message, 0U, 0U);
}

static void TripGuard_QueueTelemetry(TripGuard_TelemetryEvent_t event)
{
  uint32_t message = (uint32_t)event;

  if (osMessageQueuePut(telemetryEventQueueHandle,
                        &message,
                        0U,
                        0U) != osOK)
  {
    tripguard_telemetry_queue_full_count++;
  }
}

static uint8_t TripGuard_IsRearDuplicate(uint8_t node_id,
                                         uint16_t sequence)
{
  uint8_t index;

  for (index = 0U; index < rear_history_count; index++)
  {
    if ((rear_history_node[index] == node_id) &&
        (rear_history_sequence[index] == sequence))
    {
      return 1U;
    }
  }
  return 0U;
}

static void TripGuard_RememberRear(uint8_t node_id, uint16_t sequence)
{
  rear_history_node[rear_history_next] = node_id;
  rear_history_sequence[rear_history_next] = sequence;
  rear_history_next = (uint8_t)((rear_history_next + 1U) %
                                TRIPGUARD_REAR_HISTORY_SIZE);
  if (rear_history_count < TRIPGUARD_REAR_HISTORY_SIZE)
  {
    rear_history_count++;
  }
}

static uint8_t TripGuard_IsRemoteSosDuplicate(uint8_t node_id,
                                               uint16_t sequence)
{
  uint8_t index;

  for (index = 0U; index < remote_sos_history_count; index++)
  {
    if ((remote_sos_history_node[index] == node_id) &&
        (remote_sos_history_sequence[index] == sequence))
    {
      return 1U;
    }
  }
  return 0U;
}

static void TripGuard_RememberRemoteSos(uint8_t node_id, uint16_t sequence)
{
  remote_sos_history_node[remote_sos_history_next] = node_id;
  remote_sos_history_sequence[remote_sos_history_next] = sequence;
  remote_sos_history_next = (uint8_t)((remote_sos_history_next + 1U) %
                                      TRIPGUARD_REAR_HISTORY_SIZE);
  if (remote_sos_history_count < TRIPGUARD_REAR_HISTORY_SIZE)
  {
    remote_sos_history_count++;
  }
}

static void TripGuard_ProcessGatewayEvents(void)
{
  tripguard_event_t event;
  tripguard_ack_t ack;
  uint8_t recognized;
  uint8_t pi_event_code;

  while (osMessageQueueGet(tripguardEventQueueHandle,
                           &event,
                           NULL,
                           0U) == osOK)
  {
    recognized = 0U;
    pi_event_code = 0U;
    if (event.source_node != TG_NODE_REMOTE)
    {
      tripguard_rear_invalid_count++;
    }
    else if ((event.type == TG_EVT_REAR_CONFIRM) && (event.value == 1U))
    {
      recognized = 1U;
      if (TripGuard_IsRearDuplicate(event.source_node,
                                    event.sequence) == 0U)
      {
        tripguard_rear_confirmed = 1U;
        tripguard_rear_confirm_node = event.source_node;
        tripguard_rear_confirm_sequence = event.sequence;
        tripguard_rear_confirm_count++;
        TripGuard_RememberRear(event.source_node, event.sequence);
        (void)TripGuard_Supervisor_PostRearConfirm(event.source_node,
                                                   event.sequence);
        TripGuard_Power_NotifyRemoteCommitted();
        TripGuard_QueueTelemetry(TRIPGUARD_TELEMETRY_REAR_CONFIRM);
        (void)PiLink_SendEvent(TG_EVENT_REAR_CONFIRM, event.value, 1U);
      }
      else
      {
        tripguard_rear_duplicate_count++;
      }
    }
    else if ((event.type == TG_EVT_REMOTE_SOS) && (event.value == 1U))
    {
      recognized = 1U;
      if (TripGuard_IsRemoteSosDuplicate(event.source_node,
                                         event.sequence) == 0U)
      {
        tripguard_remote_sos_received = 1U;
        tripguard_remote_sos_sequence = event.sequence;
        tripguard_remote_sos_count++;
        TripGuard_RememberRemoteSos(event.source_node, event.sequence);

        tripguard_sos_count++;
        tripguard_sos_pending = 1U;
        TripGuard_Power_NotifyRemoteCommitted();
        TripGuard_QueueAudio(TRIPGUARD_AUDIO_SOS_WARNING);
        TripGuard_QueueTelemetry(TRIPGUARD_TELEMETRY_SAFETY_EVENT);
        (void)PiLink_SendEvent(TG_EVENT_REMOTE_SOS, event.value, 1U);
        if (CellularTaskHandle != NULL)
        {
          (void)osThreadFlagsSet(CellularTaskHandle,
                                 TRIPGUARD_CELLULAR_FLAG_SOS);
        }
      }
      else
      {
        tripguard_remote_sos_duplicate_count++;
      }
    }
    else if ((event.type == TG_EVT_PERSON) ||
             (event.type == TG_EVT_RADAR_CLEAR) ||
             (event.type == TG_EVT_RADAR_HEALTH))
    {
      /* Gateway has already validated and applied sequence-safe radar state. */
      recognized = 1U;
      if (event.type == TG_EVT_PERSON)
      {
        pi_event_code = TG_EVENT_PERSON;
      }
      else if (event.type == TG_EVT_RADAR_CLEAR)
      {
        pi_event_code = TG_EVENT_RADAR_CLEAR;
      }
      else
      {
        pi_event_code = TG_EVENT_RADAR_HEALTH;
      }
      (void)PiLink_SendEvent(pi_event_code, event.value, 1U);
      TripGuard_QueueTelemetry(TRIPGUARD_TELEMETRY_SAFETY_EVENT);
    }
    else
    {
      tripguard_rear_invalid_count++;
    }

    if (recognized != 0U)
    {
      /* Local commit token rearms SPI RX; no MISO/READY is used. */
      ack.node_id = event.source_node;
      ack.sequence = event.sequence;
      ack.value = TG_VALUE_ACK_OK;
      if (osMessageQueuePut(gatewayAckQueueHandle, &ack, 0U, 0U) != osOK)
      {
        tripguard_gateway_ack_queue_full_count++;
      }
    }
  }
}

static void TripGuard_BuildTelemetry(char *buffer, size_t buffer_size,
                                     TripGuard_TelemetryEvent_t event)
{
  static uint32_t telemetry_sequence = 0U;
  const char *event_name = "PERIODIC";
  uint8_t event_code = 0U;
  size_t length;
  size_t remaining;
  int written;
  TripGuard_SupervisorSnapshot_t supervisor;
  PiLink_Snapshot_t pi;

  if ((buffer == NULL) || (buffer_size == 0U))
  {
    return;
  }

  buffer[0] = '\0';

  if (osMutexAcquire(gpsDataMutexHandle, 50U) == osOK)
  {
    GPS_BuildThingsBoardJSON(buffer, buffer_size);
    (void)osMutexRelease(gpsDataMutexHandle);
  }

  length = strlen(buffer);

  if ((length == 0U) || (buffer[length - 1U] != '}'))
  {
    (void)snprintf(buffer,
                   buffer_size,
                   "{\"gps_state\":\"UNAVAILABLE\","
                   "\"gps_link_alive\":false,"
                   "\"position_available\":false,"
                   "\"position_current\":false}");
    length = strlen(buffer);
  }

  telemetry_sequence++;
  TripGuard_Supervisor_GetSnapshot(&supervisor);
  PiLink_GetSnapshot(&pi);

  if (event == TRIPGUARD_TELEMETRY_REAR_CONFIRM)
  {
    event_name = "REAR_CONFIRM";
    event_code = 3U;
  }
  else if (event == TRIPGUARD_TELEMETRY_SAFETY_EVENT)
  {
    event_name = "SAFETY_STATE_CHANGE";
    event_code = 2U;
  }

  if ((length > 0U) && (length < buffer_size))
  {
    remaining = buffer_size - length + 1U;

    written = snprintf(
        &buffer[length - 1U],
        remaining,

        ",\"event\":\"%s\","
        "\"event_code\":%u,"
        "\"telemetry_seq\":%lu,"

        "\"systemState\":\"%s\","
        "\"systemArmed\":%s,"
        "\"activationSource\":\"%s\","
        "\"activationCountdown\":%lu,"
        "\"rearConfirmWaitRemainingSec\":%lu,"
        "\"connectivityOwner\":\"PI\","
        "\"rearConfirmed\":%s,"
        "\"powerPresent\":%s,"
        "\"stationDetected\":%s,"
        "\"gpsFixValid\":%s,"
        "\"espGatewayOnline\":%s,"
        "\"radarEnabled\":%s,"
        "\"cameraEnabled\":%s,"
        "\"lastActivationReason\":\"%s\","
        "\"lastError\":\"%s\","

        "\"armRemainingSec\":%lu,"
        "\"piOnline\":%s,"
        "\"piReady\":%s,"
        "\"piRxFrames\":%lu,"
        "\"piTxFrames\":%lu,"
        "\"piCrcErrors\":%lu,"
        "\"piUartErrors\":%lu,"
        "\"piLastPacketAgeMs\":%lu,"
        "\"cameraPerson\":%s,"
        "\"cameraConfidencePermille\":%u,"
        "\"piScanSessionId\":%lu,"
        "\"piScanCommandSentCount\":%lu,"
        "\"piScanCommandAckCount\":%lu,"
        "\"piScanCompleteCount\":%lu,"
        "\"piScanLastResult\":%u,"
        "\"cameraAlertArmed\":%s,"
        "\"cameraAlertCount\":%lu,"
        "\"selfTestMask\":%u,"
        "\"selfTestComplete\":%s,"
        "\"selfTestRetries\":%lu,"

        "\"radar_online\":%s,"
        "\"person_detected\":%s,"
        "\"radar_state\":%u,"
        "\"radar_distance_cm\":%u,"
        "\"radar_moving_distance_cm\":%u,"
        "\"radar_still_distance_cm\":%u,"
        "\"radar_moving_energy\":%u,"
        "\"radar_still_energy\":%u,"
        "\"radar_alert_armed\":%s,"
        "\"radar_alert_enabled\":%s,"
        "\"radar_alert_count\":%lu,"
        "\"radar_valid_frames\":%lu,"
        "\"radar_invalid_frames\":%lu,"
        "\"radar_uart_errors\":%lu,"

        "\"servo_angle_deg\":%u,"
        "\"servo_pulse_us\":%u,"
        "\"servo_moving\":%s,"
        "\"servo_tracking_radar\":%s,"
        "\"servo_move_count\":%lu,"

        "\"sos_pending\":%s,"
        "\"sos_button_latched\":%s,"
        "\"sos_button_raw\":%s,"
        "\"sos_count\":%lu,"
        "\"driver_ack_count\":%lu,"
        "\"ack_button_raw\":%s,"

        "\"rear_confirmed\":%s,"
        "\"rear_confirm_node\":%u,"
        "\"rear_confirm_sequence\":%u,"
        "\"rear_confirm_count\":%lu,"
        "\"rear_duplicate_count\":%lu,"
        "\"rear_invalid_count\":%lu,"
        "\"gateway_spi_ok\":%s,"

        "\"audio_busy_raw\":%u,"
        "\"audio_track\":%u,"
        "\"audio_error_count\":%lu,"
        "\"driving_state\":%u,"

        "\"alert_active\":%s,"
        "\"alert_kind\":%u,"
        "\"alert_phase\":%u,"
        "\"alert_last_kind\":%u,"
        "\"alert_last_success\":%s,"
        "\"sms_sent_count\":%lu,"
        "\"call_started_count\":%lu,"
        "\"alert_error_count\":%lu,"

        "\"modem_state\":%u,"
        "\"modem_ready\":%s,"
        "\"network_registered\":%s,"
        "\"cereg_status\":%d,"
        "\"pdp_active\":%s,"

        "\"mqtt_connected\":%s,"
        "\"mqtt_state\":%u,"
        "\"mqtt_publish_count\":%lu,"
        "\"mqtt_error_count\":%lu,"
        "\"rpc_command_count\":%lu,"
        "\"rpc_receive_error_count\":%lu,"
        "\"remote_command_count\":%lu,"
        "\"remote_rejected_count\":%lu,"
        "\"last_rpc_method\":\"%s\","
        "\"last_rpc_result\":\"%s\","
        "\"telemetry_period_ms\":%lu,"
        "\"telemetry_dropped_count\":0,"
        "\"uptime_s\":%lu}",

        event_name,
        (unsigned int)event_code,
        (unsigned long)telemetry_sequence,

        TripGuard_Supervisor_StateName(supervisor.state),
        (supervisor.system_armed != 0U) ? "true" : "false",
        TripGuard_Supervisor_SourceName(supervisor.activation_source),
        (unsigned long)supervisor.activation_countdown_s,
        (unsigned long)supervisor.rear_confirm_wait_remaining_s,
        (supervisor.rear_confirmed != 0U) ? "true" : "false",
        (supervisor.power_present != 0U) ? "true" : "false",
        (supervisor.station_detected != 0U) ? "true" : "false",
        (supervisor.gps_fix_valid != 0U) ? "true" : "false",
        (supervisor.esp_gateway_online != 0U) ? "true" : "false",
        (supervisor.radar_enabled != 0U) ? "true" : "false",
        (supervisor.camera_enabled != 0U) ? "true" : "false",
        supervisor.last_activation_reason,
        supervisor.last_error,

        (unsigned long)supervisor.activation_countdown_s,
        (pi.online != 0U) ? "true" : "false",
        (pi.ready != 0U) ? "true" : "false",
        (unsigned long)pi.rx_frames,
        (unsigned long)pi.tx_frames,
        (unsigned long)pi.crc_errors,
        (unsigned long)pi.uart_errors,
        (unsigned long)pi.last_packet_age_ms,
        (pi.camera_person != 0U) ? "true" : "false",
        (unsigned int)pi.camera_confidence_permille,
        (unsigned long)pi_scan_session_id,
        (unsigned long)pi_scan_command_sent_count,
        (unsigned long)pi_scan_command_ack_count,
        (unsigned long)pi_scan_complete_count,
        (unsigned int)pi_scan_last_result,
        (tripguard_camera_alert_armed != 0U) ? "true" : "false",
        (unsigned long)tripguard_camera_alert_count,
        (unsigned int)supervisor.self_test_pass_mask,
        (supervisor.self_test_complete != 0U) ? "true" : "false",
        (unsigned long)supervisor.self_test_retry_count,

        (tripguard_radar_online != 0U) ? "true" : "false",
        (tripguard_person_detected != 0U) ? "true" : "false",
        (unsigned int)tripguard_radar_state,
        (unsigned int)tripguard_radar_distance_cm,
        (unsigned int)tripguard_radar_moving_distance_cm,
        (unsigned int)tripguard_radar_still_distance_cm,
        (unsigned int)tripguard_radar_moving_energy,
        (unsigned int)tripguard_radar_still_energy,
        (tripguard_radar_alert_armed != 0U) ? "true" : "false",
        (tripguard_radar_alert_enabled != 0U) ? "true" : "false",
        (unsigned long)tripguard_radar_alert_count,
        (unsigned long)tripguard_radar_valid_frame_count,
        (unsigned long)tripguard_radar_invalid_frame_count,
        (unsigned long)tripguard_radar_uart_error_count,

        (unsigned int)tripguard_servo_angle_deg,
        (unsigned int)tripguard_servo_pulse_us,
        (tripguard_servo_moving != 0U) ? "true" : "false",
        (tripguard_servo_tracking_radar != 0U) ? "true" : "false",
        (unsigned long)tripguard_servo_move_count,

        (tripguard_sos_pending != 0U) ? "true" : "false",
        (tripguard_sos_button_latched != 0U) ? "true" : "false",
        (tripguard_sos_button_raw != 0U) ? "true" : "false",
        (unsigned long)tripguard_sos_count,
        (unsigned long)tripguard_driver_ack_count,
        (tripguard_ack_button_raw != 0U) ? "true" : "false",

        (tripguard_rear_confirmed != 0U) ? "true" : "false",
        (unsigned int)tripguard_rear_confirm_node,
        (unsigned int)tripguard_rear_confirm_sequence,
        (unsigned long)tripguard_rear_confirm_count,
        (unsigned long)tripguard_rear_duplicate_count,
        (unsigned long)tripguard_rear_invalid_count,
        (tripguard_gateway_init_ok != 0U) ? "true" : "false",

        (unsigned int)tripguard_audio_busy_raw,
        (unsigned int)tripguard_audio_track,
        (unsigned long)tripguard_audio_error_count,
        (unsigned int)tripguard_driving_state,

        (tripguard_alert_active != 0U) ? "true" : "false",
        (unsigned int)alert_kind,
        (unsigned int)tripguard_alert_phase,
        (unsigned int)tripguard_alert_last_kind,
        (tripguard_alert_last_success != 0U) ? "true" : "false",
        (unsigned long)tripguard_sms_sent_count,
        (unsigned long)tripguard_call_started_count,
        (unsigned long)tripguard_alert_error_count,

        (unsigned int)A7670_GetState(),
        (A7670_IsNetworkReady() != 0U) ? "true" : "false",
        (A7670_IsRegistered() != 0U) ? "true" : "false",
        (int)A7670_GetRegistrationStatus(),
        (A7670_IsPdpActive() != 0U) ? "true" : "false",

        (TB_MQTT_IsConnected() != 0U) ? "true" : "false",
        (unsigned int)TB_MQTT_GetState(),
        (unsigned long)TB_MQTT_GetPublishCount(),
        (unsigned long)TB_MQTT_GetErrorCount(),
        (unsigned long)TB_MQTT_GetRpcCommandCount(),
        (unsigned long)TB_MQTT_GetRpcErrorCount(),
        (unsigned long)tripguard_remote_command_count,
        (unsigned long)tripguard_remote_rejected_count,
        (const char *)(uintptr_t)tripguard_last_rpc_method,
        (const char *)(uintptr_t)tripguard_last_rpc_result,
        (unsigned long)tripguard_telemetry_period_ms,
        (unsigned long)(HAL_GetTick() / 1000U));

    if ((written < 0) || ((size_t)written >= remaining))
    {
      (void)snprintf(buffer,
                     buffer_size,
                     "{\"event\":\"TELEMETRY_TRUNCATED\","
                     "\"event_code\":255,"
                     "\"telemetry_seq\":%lu,"
                     "\"uptime_s\":%lu}",
                     (unsigned long)telemetry_sequence,
                     (unsigned long)(HAL_GetTick() / 1000U));
    }
  }

  buffer[buffer_size - 1U] = '\0';
}
/* USER CODE END Application */
