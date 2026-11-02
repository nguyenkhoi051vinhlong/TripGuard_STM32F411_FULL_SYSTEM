# Thiết kế Rear Pod dùng ESP32-C3 Mini

## 1. Kiến trúc đã chốt

Rear Pod đặt phía sau xe chỉ gồm ESP32-C3 Mini, nút `REAR CONFIRM`, radar
LD2410C và mạch cấp nguồn radar. Pod không có đèn, còi hoặc JQ8900. Pod truyền
dữ liệu bằng Bluetooth Mesh tới gateway ESP32-WROOM-32. Gateway đặt gần STM32
và nối với STM32 bằng UART TTL 3,3 V.

```text
REAR CONFIRM ─┐
              ├─> ESP32-C3 Mini ── Bluetooth Mesh ── ESP32-WROOM-32
LD2410C ──────┘      Mesh LPN                            Mesh Friend
                                                            |
                                                     UART TTL 3,3 V
                                                            |
                                                          STM32
```

Thiết kế demo không lắp mạch chuyển đổi đường truyền vi sai. GPIO của gateway
không được nối trực tiếp vào UART mức 5 V.

## 2. Chế độ hoạt động

1. Khi chưa provision Mesh, C3 giữ hoạt động để điện thoại hoặc provisioner
   thêm node vào mạng.
2. Khi đã provision và đang chờ, nguồn LD2410C bị ngắt. ESP32-C3 hoạt động ở
   chế độ Mesh Low Power Node (LPN), bật Bluetooth modem sleep và automatic
   light sleep.
3. Người dùng bấm nút `REAR CONFIRM` một lần: C3 thức, gửi sự kiện lên
   gateway, bật nguồn radar và quét liên tục 5 phút.
4. Trong 5 phút, C3 đọc UART và chân `OUT` của LD2410C, rồi gửi trạng thái về
   gateway. Mỗi lệnh/sự kiện quan trọng phải có transaction ID, ACK, retry và
   chống xử lý trùng.
5. Hết 5 phút, C3 dừng radar, đưa các chân UART radar về trạng thái an toàn,
   ngắt nguồn radar và quay lại LPN + light sleep.

Không dùng deep sleep trong trạng thái chờ vì deep sleep tắt Bluetooth; gateway
sẽ không thể gửi lệnh hoặc ACK tới C3. “Tight sleep” được triển khai là light
sleep kết hợp Mesh LPN. Gateway WROOM phải luôn được cấp nguồn và cấu hình là
Friend Node để lưu gói tin cho LPN.

## 3. Danh sách khối phần cứng

| Khối | Phần tử đề xuất | Chức năng |
|---|---|---|
| MCU | ESP32-C3 Mini/SuperMini, flash 4 MB | BLE Mesh, nút, radar và báo trạng thái |
| Radar | HLK-LD2410C | Phát hiện mục tiêu tĩnh/chuyển động |
| Công tắc nguồn radar | TPS22919 hoặc load switch 5 V active-high tương đương | Tắt hẳn LD2410C khi chờ |
| Nút | LA38, tiếp điểm thường hở 23–24 | REAR CONFIRM |
| Nguồn | Buck phù hợp nguồn xe, đầu ra 5 V/1 A | Cấp C3 và radar |
| Gateway | ESP32-WROOM-32 | Mesh Friend/Proxy và cầu UART tới STM32 |

Nếu dùng trên nguồn xe thật, phía đầu vào buck cần cầu chì tự phục hồi, chống
đảo cực, TVS và tụ lọc phù hợp. Không cấp trực tiếp 12 V hoặc 24 V vào chân 5 V
của ESP32-C3 Mini.

## 4. Phân chân ESP32-C3 Mini

| GPIO | Tên firmware | Hướng | Kết nối |
|---:|---|---|---|
| GPIO0 | `RADAR_OUT` | Input | `OUT` của LD2410C qua 1 kΩ |
| GPIO1 | `SPARE_1` | Dự phòng | Không nối |
| GPIO3 | `REAR_BUTTON` | Input pull-up | Nút LA38, tiếp điểm 23–24 |
| GPIO4 | `RADAR_RX` | UART RX | Nhận từ `TX` LD2410C qua 220 Ω |
| GPIO5 | `RADAR_TX` | UART TX | Gửi tới `RX` LD2410C qua 220 Ω |
| GPIO6 | `SPARE_6` | Dự phòng | Không nối |
| GPIO7 | `SPARE_7` | Dự phòng | Không nối |
| GPIO10 | `RADAR_POWER_EN` | Output | EN load switch; kéo xuống GND 100 kΩ |
| GPIO20 | `TEST_RX` | Dự phòng | Test point, không nối tải cố định |
| GPIO21 | `TEST_TX` | Dự phòng | Test point, không nối tải cố định |

GPIO9 là chân BOOT của nhiều board C3 Mini/SuperMini: không gắn nút chức năng
vào chân này. GPIO18/GPIO19 thường được board dùng cho USB native, cũng không
dùng cho ngoại vi của pod. Kiểm tra đúng pinout của board thực tế trước khi hàn.

## 5. Đấu nối chi tiết

### 5.1 Nguồn

```text
Nguồn xe -> cầu chì/PTC -> chống đảo cực -> TVS -> buck 5 V
                                                     |-> chân 5V ESP32-C3 Mini
                                                     |-> VIN load switch radar
GND nguồn ----------- GND C3 -------- GND LD2410C -------- GND gateway/STM32
```

Đặt gần C3 một tụ 100 µF và một tụ 100 nF giữa 5 V–GND. Đặt gần LD2410C một
tụ 100 µF và một tụ 100 nF ở phía sau load switch. Tất cả các khối phải chung
GND; chỉ đường truyền giữa Rear Pod và gateway là không dây.

### 5.2 Nguồn điều khiển LD2410C

```text
5 V ---------------- VIN  load switch  VOUT -------------- VCC LD2410C
GPIO10 --- 100 Ω ---- EN
                   |
                  100 kΩ
                   |
                  GND
```

Chọn load switch hoạt động được ở 5 V và chân EN chấp nhận logic 3,3 V. Điện
trở kéo xuống giữ radar tắt trong lúc reset/boot. Không nối GPIO10 trực tiếp vào
đường cấp nguồn radar.

Trình tự firmware:

- Bật radar: đặt UART, kéo `RADAR_POWER_EN` lên 1, chờ radar khởi động tối thiểu
  2 giây rồi mới tin dữ liệu.
- Tắt radar: ngừng UART, đặt GPIO4/GPIO5 input high-impedance, sau đó kéo
  `RADAR_POWER_EN` xuống 0. Việc này tránh cấp nguồn ngược qua chân UART.

### 5.3 Giao tiếp LD2410C

| LD2410C | ESP32-C3 Mini |
|---|---|
| VCC | 5 V sau load switch |
| GND | GND chung |
| TX | GPIO4 qua 220 Ω |
| RX | GPIO5 qua 220 Ω |
| OUT | GPIO0 qua 1 kΩ |

Mặc định firmware dùng UART `256000, 8N1` cho LD2410C. Phải kiểm tra datasheet
hoặc cấu hình thực tế của module nếu nhà cung cấp đã đổi baud rate. Logic UART
của module và C3 phải tương thích 3,3 V; không đưa tín hiệu 5 V vào GPIO C3.

### 5.4 Nút LA38 REAR CONFIRM

Hai chân `23–24` là một cặp tiếp điểm thường hở, không phân cực:

```text
LA38 chân 23 ---------------- GPIO3
LA38 chân 24 ---------------- GND
GPIO3 -------- 10 kΩ -------- 3V3     (khuyến nghị thêm)
GPIO3 -------- 100 nF ------- GND     (lọc nhiễu, đặt gần C3)
```

Có thể đổi chỗ chân 23 và 24. Tuyệt đối không cấp 5 V hoặc điện áp của đèn nút
vào GPIO3. Firmware cấu hình GPIO3 là `INPUT_PULLUP`: nhả nút đọc HIGH, nhấn
đọc LOW. Xung LOW đầu tiên tạo REAR CONFIRM; firmware khóa các xung
dội và chỉ mở lại sau khi HIGH ổn định 80 ms. Nếu nút LA38 có
hai chân đèn riêng thì để hở, không nối vào C3 hoặc nguồn.

## 6. Gateway ESP32-WROOM-32 nối STM32

Gateway nhận Mesh và chuyển sự kiện qua UART TTL 3,3 V:

| ESP32-WROOM-32 gateway | STM32 | Ý nghĩa |
|---|---|---|
| GPIO17 / UART2 TX | PA10 / USART1 RX | Gateway gửi sự kiện tới STM32 |
| GPIO16 / UART2 RX | PA9 / USART1 TX | STM32 gửi ACK/lệnh tới gateway |
| GND | GND | Bắt buộc chung mass |

Cấu hình ban đầu: `115200, 8N1`, không flow control. Nếu bo STM32 đang dùng
PA9/PA10 cho GPS hoặc ngoại vi khác, phải chuyển ngoại vi đó sang UART khác trước;
không nối hai ngõ TX cùng vào một đường dây.

Hai board đặt gần nhau nên UART TTL là đủ. Dây TX/RX/GND nên ngắn, đấu chéo TX
sang RX và kiểm tra cả hai phía đều dùng mức logic 3,3 V.

## 7. Quy tắc tiết kiệm điện

- C3 là Mesh LPN; gateway WROOM là Friend Node và luôn bật.
- Bật `CONFIG_PM_ENABLE`, dynamic frequency scaling, Bluetooth modem sleep và
  automatic light sleep trong ESP-IDF.
- Không gọi deep sleep trong khi cần nhận Mesh.
- Radar tắt nguồn hoàn toàn ở trạng thái chờ; đây là phần tiết kiệm điện lớn nhất.
- Trong cửa sổ quét 5 phút, UART/radar hoạt động nên C3 có thể ngủ ít hơn; sau khi
  tắt radar mới quay lại chế độ chờ tiết kiệm điện.
- Khi chưa provision, giữ thiết bị thức để ghép mạng. Sau khi provision mới bật
  LPN/light sleep.
- GPIO3 của nút được cấu hình làm nguồn đánh thức khi light sleep;
  firmware chống dội bằng khóa sự kiện đến khi HIGH ổn định 80 ms.

## 8. Quy tắc an toàn phần mềm

- Mỗi lần nhấn chỉ tạo một sự kiện; phải thả nút trước lần nhấn kế tiếp.
- Mọi sự kiện quan trọng có transaction ID; gateway khử trùng và ACK.
- Nếu không nhận ACK, C3 retry theo khoảng tăng dần và gửi trạng thái lỗi về gateway.
- Lệnh `START_SCAN` khi radar đã chạy chỉ gia hạn/khởi tạo lại bộ đếm có chủ đích,
  không tạo nhiều timer song song.
- Timer 5 phút dùng `esp_timer` hoặc timer đơn điệu, không phụ thuộc thời gian
  hệ thống.
- Khi LD2410C mất phản hồi, gửi cờ lỗi nhưng không giả định “không có người”.
- Gateway chỉ ACK sự kiện cho C3 sau khi đã chuyển thành công tới STM32 hoặc đã
  lưu bền để gửi lại.

## 9. BOM tối thiểu cho một Rear Pod

| Nhóm | Linh kiện | Số lượng |
|---|---|---:|
| Điều khiển | ESP32-C3 Mini/SuperMini 4 MB | 1 |
| Cảm biến | LD2410C | 1 |
| Nguồn radar | Load switch 5 V active-high + R 100 Ω + R 100 kΩ | 1 bộ |
| Nút | LA38 SPDT | 1 |
| Lọc nút | R kéo lên 10 kΩ và tụ lọc 100 nF | 1 bộ |
| Lọc nguồn | 100 µF ×2, 100 nF ×2 | 1 bộ |
| Nguồn | Buck 5 V/1 A có bảo vệ đầu vào phù hợp | 1 |

Gateway cần thêm ESP32-WROOM-32 và ba dây UART `TX`, `RX`, `GND` sang STM32.

## 10. Kiểm thử bắt buộc

1. Kiểm tra 5 V và 3,3 V trước khi cắm C3/radar.
2. Provision Rear Pod vào Mesh; xác nhận node vào LPN và gateway có Friendship.
3. Đo GPIO10: radar phải tắt trong lúc chờ và bật khi bắt đầu quét.
4. Bấm REAR CONFIRM một lần; gateway nhận đúng một transaction và STM32 trả ACK.
5. Xác nhận radar chỉ được tin sau thời gian khởi động.
6. Xác nhận scan tự dừng sau 5 phút rồi radar mất nguồn.
7. Trong light sleep, gửi lệnh `START_SCAN` từ gateway; Rear Pod phải nhận qua
   Friendship và bắt đầu quét.
8. Rút TX/RX LD2410C; hệ thống phải báo sensor fault, không báo vùng sau an toàn.
9. Mất gateway rồi khôi phục; C3 retry có giới hạn, không reset hoặc watchdog.
10. Chạy thử tối thiểu 2 giờ, kiểm tra Mesh ACK, UART ACK, timer và nhiệt độ nguồn.

Giao thức chi tiết nằm trong [GIAO_THUC_MESH_UART.md](./GIAO_THUC_MESH_UART.md).
