# TripGuard — hệ thống giám sát và cảnh báo

Repository chứa firmware và cấu hình các thiết bị trong hệ thống TripGuard.

## Thành phần

- Firmware STM32F411 đầy đủ ở thư mục gốc; hướng dẫn hệ thống tại [README_FULL_SYSTEM_VI.md](README_FULL_SYSTEM_VI.md).
- Firmware STM32 GPS + 4G độc lập tại [STM32_GPS_4G/Module gps_neo_m8l](STM32_GPS_4G/Module%20gps_neo_m8l), có project STM32CubeIDE và hướng dẫn tại `TRIPGUARD_README.md`.
- Remote SOS ESP32-C3 tại [ESP32_C3_SOS_Remote](ESP32_C3_SOS_Remote).
- Rear Pod ESP32-C3 + LD2410 tại [ESP32_Rear_Pod_LD2410](ESP32_Rear_Pod_LD2410).
- Gateway ESP32-WROOM tại [ESP32_WROOM_Gateway](ESP32_WROOM_Gateway).
- Dashboard ThingsBoard và file import tại [ThingsBoard_Dashboard](ThingsBoard_Dashboard).

## Build firmware

- Mở `TripGuard_STM32F411_RTOS.ioc` ở thư mục gốc hoặc `Module gps_neo_m8l.ioc` trong project GPS + 4G bằng STM32CubeIDE.
- Với các firmware ESP32, dùng ESP-IDF 6.1. Mở ESP-IDF PowerShell để có `IDF_PATH`, vào thư mục thiết bị rồi chạy `\.\build.ps1`. Script tạo thư mục build tạm và đặt file nạp cục bộ trong `firmware/`.
- File `.bin` và cấu hình build sinh ra không được commit; build lại từ source để tạo chúng. Hướng dẫn nạp nằm trong `firmware/HUONG_DAN_NAP.md` ở các project có tệp này.
- Import dashboard từ `ThingsBoard_Dashboard/TripGuard_Full_System_Dashboard.json`; xem `README_IMPORT.md` trong cùng thư mục.

## Cấu hình thiết bị

Trước khi nạp firmware, cập nhật APN, broker, access token và số điện thoại trong file cấu hình tương ứng. Các giá trị nhạy cảm cần giữ ở máy local; không commit token đang sử dụng hoặc số điện thoại cá nhân.
