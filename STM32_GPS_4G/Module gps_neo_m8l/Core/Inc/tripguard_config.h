#ifndef INC_TRIPGUARD_CONFIG_H_
#define INC_TRIPGUARD_CONFIG_H_

/*
 * Cau hinh trien khai TripGuard.
 *
 * Thay APN theo SIM dang su dung. Neu nha mang khong yeu cau user/password,
 * giu hai chuoi tuong ung rong.
 */
#define TRIPGUARD_CELLULAR_APN          "internet"
#define TRIPGUARD_CELLULAR_USER         ""
#define TRIPGUARD_CELLULAR_PASSWORD     ""

/* ThingsBoard MQTT over TCP. */
#define TRIPGUARD_TB_HOST               "demo.thingsboard.io"
#define TRIPGUARD_TB_PORT               1883U
#define TRIPGUARD_TB_ACCESS_TOKEN       "REPLACE_WITH_DEVICE_ACCESS_TOKEN"
#define TRIPGUARD_TB_CLIENT_ID          "tripguard-stm32f411"
#define TRIPGUARD_TB_TELEMETRY_TOPIC    "v1/devices/me/telemetry"

#endif /* INC_TRIPGUARD_CONFIG_H_ */
