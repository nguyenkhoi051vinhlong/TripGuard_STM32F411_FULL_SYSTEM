# Chi can Generate Code

1. Giai nen ZIP vao mot thu muc ngan, khong dau. Vi du: `C:\STM32\TripGuard_STM32F411_RTOS`.
2. Mo STM32CubeIDE, chon `File > Import > Existing Projects into Workspace`.
3. Chon thu muc `TripGuard_STM32F411_RTOS`, sau do bam `Finish`.
4. Mo file `TripGuard_STM32F411_RTOS.ioc`.
5. Project da ha ve CubeMX 6.13.0 va STM32Cube FW_F4 V1.28.0 de khop IDE cua ban. Neu IDE hoi migrate firmware, chon `Migrate`/`Continue` va giu `STM32Cube FW_F4 V1.28.x` dang co.
6. Bam `GENERATE CODE` va chon `Yes` khi IDE hoi mo C/C++ perspective.
7. Bam `Project > Clean...`, chon dung `TripGuard_STM32F411_RTOS`, sau do `Build Project`.
8. Nap chuong trinh bang nut `Run` hoac `Debug`.

Khong doi pin trong CubeMX. Cau hinh da san:

- SYSCLK 96 MHz; APB1 48 MHz; APB2 96 MHz.
- GPS USART1: PA9/PA10, 9600 baud.
- A7670 USART2: PA2/PA3, 9600 baud, RX DMA circular.
- Raspberry Pi USART6: PA11/TX -> GPIO15/RXD pin 10; PA12/RX <- GPIO14/TXD
  pin 8; 115200 baud, DMA RX-to-IDLE.
- LD2410 nam trong ESP32 Remote va den STM32 qua Gateway SPI2.
- Loa JQ8900: PB0; BUSY PB12.
- Nut SOS: PB1 keo xuong GND; nut ACK: PB10 keo xuong GND.
- Servo: PA6, TIM3_CH1, PWM 50 Hz.
- So SMS/goi: +84838740019.

Luu y: PA11/PA12 dang dung cho PiLink, vi vay khong bat USB_OTG_FS. Khi
generate lai, giu cac khoi `USER CODE` va xac nhan `pi_link.c` van nam trong
build.
