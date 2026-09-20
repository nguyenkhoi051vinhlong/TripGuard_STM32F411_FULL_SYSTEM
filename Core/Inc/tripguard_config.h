#ifndef INC_TRIPGUARD_CONFIG_H_
#define INC_TRIPGUARD_CONFIG_H_

/*
 * Cau hinh trien khai TripGuard.
 *
 * Thay APN theo SIM dang su dung. Neu nha mang khong yeu cau user/password,
 * giu hai chuoi tuong ung rong.
 */
#define TRIPGUARD_CELLULAR_APN          "v-internet"
#define TRIPGUARD_CELLULAR_USER         ""
#define TRIPGUARD_CELLULAR_PASSWORD     ""

/*
 * ThingsBoard MQTT over TCP. HOST/PORT phai la TCP tunnel toi cong MQTT
 * 1883 cua ThingsBoard, khong phai URL HTTPS cua Cloudflare Quick Tunnel.
 *
 * Dung hostname DNS-only rieng cho MQTT. Khong dung hostname "tb" dang
 * Cloudflare-proxy vi proxy web thong thuong khong chuyen MQTT TCP.
 *
 * HOST/PORT ben duoi la endpoint Pinggy gan nhat nguoi dung da cung cap.
 * Pinggy free het han sau 60 phut; neu PowerShell in endpoint moi thi phai
 * thay dung hai gia tri nay, Clean/Build va nap lai firmware.
 */
#define TRIPGUARD_TB_HOST               "mbvyt-58-187-48-135.run.pinggy-free.link"
#define TRIPGUARD_TB_PORT               36225U
#define TRIPGUARD_TB_ACCESS_TOKEN       "Trankhoinguyen0110"
#define TRIPGUARD_TB_CLIENT_ID          "tripguard-stm32f411"
#define TRIPGUARD_TB_TELEMETRY_TOPIC    "v1/devices/me/telemetry"
#define TRIPGUARD_TB_KEEPALIVE_SECONDS  60U
#define TRIPGUARD_TB_CLEAN_SESSION      1U
#define TRIPGUARD_TB_PUBLISH_QOS        1U

/* Telemetry mac dinh 2 giay, co the doi tu app trong khoang 2..60 giay. */
#define TRIPGUARD_TELEMETRY_PERIOD_DEFAULT_MS   2000U
#define TRIPGUARD_TELEMETRY_PERIOD_MIN_MS       2000U
#define TRIPGUARD_TELEMETRY_PERIOD_MAX_MS      60000U

/*
 * Connectivity ownership in the production architecture.
 * 1: Raspberry Pi owns the 4G modem and ThingsBoard connection.  STM32 only
 *    sends CRC-protected telemetry/alert frames over USART6.
 * 0: retained legacy mode where STM32 owns USART2/A7670/MQTT.
 */
#define TRIPGUARD_CONNECTIVITY_OWNER_PI              1U

/*
 * Servo 360 do (continuous rotation), dieu khien bang toc do + thoi gian.
 * 1500 us = dung; lon hon 1500 us quay ve radar; nho hon 1500 us quay ve nha.
 * Tuy tung servo, hay chinh STOP trong khoang 1470..1530 us neu van bi bo.
 * MOVE_TIME quyet dinh goc quay thuc te; tang/giam no de camera dung dung huong.
 */
#define TRIPGUARD_SERVO_STOP_PULSE_US          1500U
#define TRIPGUARD_SERVO_TO_RADAR_PULSE_US      1700U
#define TRIPGUARD_SERVO_TO_HOME_PULSE_US       1300U
#define TRIPGUARD_SERVO_MOVE_TIME_MS            700U

/* Chi la vi tri logic de hien thi tren ThingsBoard, khong phai feedback goc. */
#define TRIPGUARD_SERVO_HOME_ANGLE_DEG            0U
#define TRIPGUARD_SERVO_RADAR_ANGLE_DEG          90U

/*
 * SMS text mode GSM da duoc xac nhan chay on dinh trong project test rieng.
 * Noi dung de ASCII khong dau de tranh phu thuoc UCS2/DCS cua nha mang.
 */
#define TRIPGUARD_EMERGENCY_NUMBER             "0838740019"

/* Radar chi gui SMS; nut SOS gui SMS rieng va sau do goi dien. */
#define TRIPGUARD_RADAR_SMS_TEXT                "PHAT HIEN CO NGUOI"
#define TRIPGUARD_SOS_SMS_TEXT                  "CUU TOI"
#define TRIPGUARD_SOS_CALL_DURATION_MS          20000U
#define TRIPGUARD_ALERT_NETWORK_WAIT_MS         60000U

/*
 * Production interlock. Set to 1 only after PA8 has a protected 3.3 V bus
 * detector, PB3/PB5/PB6/PB11 drive real load-switch EN inputs, PA1/PB7 are
 * connected to the Pi. With 0, physical rail-off remains inhibited for bench
 * safety. Logical standby never depends on this switch.
 */
#define TRIPGUARD_POWER_HARDWARE_READY              0U

/*
 * Logical standby keeps FreeRTOS, SPI, GPS, A7670 and MQTT alive.  STOP mode
 * from the previous power prototype is intentionally disabled in this build.
 */
#define TRIPGUARD_LOGICAL_STANDBY_ENABLED            1U
#define TRIPGUARD_USE_STOP_MODE                      0U

/* PA8 receives only a protected 0..3.3 V digital vehicle power signal. */
#define TRIPGUARD_POWER_ACTIVE_LEVEL                 1U
#define TRIPGUARD_POWER_QUALIFY_HIGH_MS          30000U
#define TRIPGUARD_POWER_LOSS_DEBOUNCE_MS          3000U

/* Logical activation timings (all evaluated with wrap-safe tick subtraction). */
#define TRIPGUARD_ARM_DELAY_MS                    60000U
#define TRIPGUARD_SENSOR_WARMUP_MS                10000U

/* One end-trip camera session commanded by STM32 lasts exactly 15 minutes. */
#define TRIPGUARD_CAMERA_SCAN_DURATION_SECONDS       900U

/*
 * After GPS has continuously verified the arrival geofence, wait at most five
 * minutes for the driver's rear-confirm.  STM32 owns this timer; GPS only
 * supplies the verified arrival condition and cannot wake a powered-off MCU by
 * itself.
 */
#define TRIPGUARD_REAR_CONFIRM_TIMEOUT_MS        300000U

/* ESP32 Gateway -> STM32 is receive-only: NSS/SCK/MOSI, no MISO/READY ACK. */
#define TRIPGUARD_SPI_ONE_WAY                        1U
#define TRIPGUARD_SPI_ACCEPT_LEGACY_FRAME            1U

/* Station coordinates are deployment data, not safe defaults. */
#define TRIPGUARD_GEOFENCE_ENABLED                   0U
#define TRIPGUARD_STATION_LAT                       0.0f
#define TRIPGUARD_STATION_LON                       0.0f
#define TRIPGUARD_STATION_LON_METERS_PER_DEG    111320.0f
#define TRIPGUARD_STATION_RADIUS_M                  50.0f
#define TRIPGUARD_GEOFENCE_DWELL_MS              15000U
#define TRIPGUARD_GPS_MAX_AGE_MS                   3000U
#define TRIPGUARD_MAX_ARRIVAL_SPEED_KMH              3.0f

/* Compatibility aliases for the retained legacy power module. */
#define TRIPGUARD_STATION_GEOFENCE_ENABLED TRIPGUARD_GEOFENCE_ENABLED
#define TRIPGUARD_STATION_LATITUDE         TRIPGUARD_STATION_LAT
#define TRIPGUARD_STATION_LONGITUDE        TRIPGUARD_STATION_LON
#define TRIPGUARD_STATION_MAX_SPEED_KMH    TRIPGUARD_MAX_ARRIVAL_SPEED_KMH
#define TRIPGUARD_STATION_CONFIRM_MS       TRIPGUARD_GEOFENCE_DWELL_MS

#endif /* INC_TRIPGUARD_CONFIG_H_ */
