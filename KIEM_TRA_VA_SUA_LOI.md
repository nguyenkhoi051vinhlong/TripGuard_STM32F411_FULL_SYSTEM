# Cac muc da kiem tra va sua

## Da sua

1. Xoa duong dan source tro toi `Codex_Backup_...` trong `.cproject`.
2. Dong bo ten folder, Eclipse project va file `.ioc` thanh
   `TripGuard_STM32F411_RTOS` de Clean/Build khong bi lech duong dan.
3. Loai bo `.metadata`, cac thu muc backup, file build Debug cu va launch cu.
4. Giu nguyen toan bo chan phan cung cua project goc.
5. Sua TIM3 CH1 tren PA6 thanh PWM 50 Hz thuc su, pulse khoi tao 1500 us.
6. Doi so canh bao thanh `+84838740019`, gui SMS GSM/ASCII de tang tinh tuong thich.
7. Tach noi dung SMS:
   - Radar: `PHAT HIEN CO NGUOI`, chi SMS.
   - SOS: `CUU TUI`, SMS roi goi dien.
8. Kiem tra lai callback UART, DMA, EXTI va TIM10 timebase.
9. Kiem tra cu phap tat ca file `Core/Src/*.c`: khong co loi cu phap.

## Can cap nhat khi trien khai

- Host/port Pinggy mien phi co the het han hoac thay doi. Firmware van test
  radar, loa, nut va SMS khi MQTT offline, nhung website se khong co du lieu
  cho den khi `TRIPGUARD_TB_HOST` va `TRIPGUARD_TB_PORT` dung.
- APN hien tai la `v-internet`, user/password de rong (Viettel).
- Access token dang nam trong source. Neu tung chia se project cong khai, nen
  tao token ThingsBoard moi.

## Gioi han cua kiem tra tu dong

Project da duoc kiem tra cau truc import, XML, pin mapping va cu phap C. Viec
nap len STM32, dang ky mang, gui SMS/cuoc goi va ket noi broker phai duoc xac
nhan tren bo mach that vi phu thuoc day noi, nguon A7670, SIM va tunnel hien tai.
