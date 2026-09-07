# Hướng dẫn LD2450 với Tang Nano 20K và PicoRV32

Project sử dụng duy nhất kit Tang Nano 20K, chạy hệ thống PicoRV32 ở 27 MHz.
LD2450 giao tiếp qua một UART phần cứng riêng ở 256000 baud, có FIFO nhận
64 byte. Firmware cấu hình radar ở chế độ theo dõi tối đa ba mục tiêu, đọc tọa
độ X/Y và đưa từng mục tiêu qua một bộ lọc Kalman độc lập.

Kết quả gửi lên máy tính gồm khoảng cách và vận tốc được tính từ state Kalman:

```text
LD2450 targets:
  T1: distance=1.532 m, velocity=0.742 m/s
  T2: distance=2.104 m, velocity=0.318 m/s (predicted)
  T3: not detected
```

Project chưa sử dụng `Target speed` hoặc `Distance resolution` của LD2450 để
update Kalman. Ba slot T1/T2/T3 hiện được ánh xạ trực tiếp sang ba track; đây
chưa phải data association theo ID người cố định.

## 1. Cách xác định vị trí chân trên Tang Nano 20K

Trong file constraint, các số như `73`, `74`, `69` và `70` là số chân package
của FPGA GW2AR-18, không phải số thứ tự của chân header.

Đặt board theo hướng:

- Cổng USB-C và hai nút S1/S2 ở phía trên.
- Cổng HDMI ở phía dưới.
- FPGA pin 73 là chân trên cùng của hàng header bên trái.
- FPGA pin 74 là chân thứ hai của hàng header bên trái.
- Chân 5V và GND là hai chân trên cùng của hàng header bên phải.

Ánh xạ chính thức của project nằm trong
[`src/picorv32_20k.cst`](src/picorv32_20k.cst).

## 2. Đấu dây LD2450

| Chân LD2450 | Tang Nano 20K | Tín hiệu trong project | Hướng dữ liệu |
| --- | --- | --- | --- |
| `5V` | Chân `5V` trên header | Nguồn radar | Tang Nano → LD2450 |
| `GND` | Một chân `GND` bất kỳ | Mass chung | Chung |
| `TX` | FPGA pin `73`, đầu hàng trái | `radar_rx` | LD2450 → PicoRV32 |
| `RX` | FPGA pin `74`, chân thứ hai hàng trái | `radar_tx` | PicoRV32 → LD2450 |

Lưu ý an toàn:

- LD2450 cần nguồn 5 V với khả năng cấp dòng lớn hơn 200 mA.
- UART của LD2450 sử dụng mức logic 3,3 V, phù hợp với I/O 3,3 V của FPGA.
- Không đưa tín hiệu logic 5 V trực tiếp vào FPGA.
- Nếu dùng nguồn 5 V riêng cho radar, bắt buộc nối chung GND với Tang Nano 20K.
- Nên ngắt nguồn trước khi cắm hoặc thay đổi dây.
- UART phải nối chéo: `TX của LD2450 → radar_rx`, `RX của LD2450 ← radar_tx`.

## 3. UART debug lên máy tính

UART debug khác với UART dành cho LD2450:

| Tín hiệu | FPGA pin | Kết nối |
| --- | ---: | --- |
| `uart_tx` | 69 | Nối nội bộ tới RX của chip USB-UART BL616 |
| `uart_rx` | 70 | Nối nội bộ tới TX của chip USB-UART BL616 |

Không cần cắm dây ngoài vào pin 69/70. Chỉ cần kết nối cổng USB-C của Tang Nano
20K với máy tính và mở cổng COM do Windows cấp.

Cấu hình Serial Debug Assistant:

| Tham số | Giá trị |
| --- | --- |
| Baud rate | `115200` |
| Data bits | `8` |
| Stop bits | `1` |
| Parity | `None` |
| Flow control | `None` |

Trên máy đã kiểm tra, UART debug xuất hiện ở FTDI channel B. Số COM có thể thay
đổi giữa các máy; hãy chọn `USB Serial Port` tương ứng rồi nhấn S1 để reset CPU
và xem thông báo khởi động.

## 4. Bảng chân đang được project sử dụng

| Chức năng | Tín hiệu | FPGA pin | Vị trí/kết nối trên board |
| --- | --- | ---: | --- |
| Clock hệ thống | `clk_in` | 4 | Clock 27 MHz có sẵn trên board |
| Reset PicoRV32 | `reset_button` | 88 | Nút S1 |
| Nút người dùng | `button2` | 87 | Nút S2 |
| UART debug TX | `uart_tx` | 69 | Nối nội bộ với USB-UART |
| UART debug RX | `uart_rx` | 70 | Nối nội bộ với USB-UART |
| LD2450 RX của FPGA | `radar_rx` | 73 | Header trái, chân trên cùng |
| LD2450 TX của FPGA | `radar_tx` | 74 | Header trái, chân thứ hai |
| OLED SCL | `oled_scl` | 29 | Header trái |
| OLED SDA | `oled_sda` | 30 | Header trái |
| WS2812 | `ws2812b_din` | 79 | LED RGB có sẵn trên board |
| SD clock | `sd_clk` | 83 | Khe thẻ TF có sẵn |
| SD MOSI | `sd_mosi` | 82 | Khe thẻ TF có sẵn |
| SD chip select | `sd_cs` | 81 | Khe thẻ TF có sẵn |
| SD MISO | `sd_miso` | 84 | Khe thẻ TF có sẵn |
| LED0..LED5 | `leds[0]..leds[5]` | 15..20 | Sáu LED có sẵn, active-low |

Không tự ý dùng lại một chân trong bảng cho ngoại vi khác nếu chưa sửa file
`.cst` và kiểm tra xung đột chân trong báo cáo Place & Route.

## 5. Thanh ghi UART LD2450

| Địa chỉ | Thanh ghi | Mô tả |
| --- | --- | --- |
| `0x80000070` | `STATUS` | bit 0: RX có dữ liệu; bit 1: overflow; bit 2: TX busy; bit 14:8: số byte trong FIFO |
| `0x80000074` | `RXDATA` | Đọc byte RX tiếp theo và loại byte đó khỏi FIFO |
| `0x80000078` | `TXDATA` | Ghi một byte TX; bus PicoRV32 chờ nếu UART đang bận |
| `0x8000007C` | `CONTROL` | Ghi bit 0 để xóa FIFO; bit 1 để xóa cờ overflow |

## 6. Build firmware C

Mở MSYS2 MINGW64 terminal:

```sh
cd "/c/Users/Mr T/Desktop/TangNano20K_Test/c_code"
make clean
make
```

Lệnh `make` biên dịch firmware RISC-V và tạo lại:

```text
src/mem_init0.ini
src/mem_init1.ini
src/mem_init2.ini
src/mem_init3.ini
src/sys_parameters.v
```

Chỉ cần Make lại khi code C thay đổi.

## 7. Build FPGA bằng Gowin

Luôn mở project qua đường dẫn ASCII sau để tránh lỗi mã hóa ký tự `Đ`:

```text
C:\Users\Mr T\Desktop\TangNano20K_Test\picorv32_20k.gprj
```

Sau khi Make firmware, chọn **Run All** trong Gowin. Bitstream được tạo tại:

```text
impl/pnr/picorv32_20k.fs
```

Cấu hình đã được kiểm tra:

- Kit: Tang Nano 20K, `GW2AR-LV18QN88C8/I7`.
- Clock hệ thống: 27 MHz.
- LD2450 UART: 256000 baud.
- UART debug: 115200 baud.
- Setup violations: 0.
- Hold violations: 0.

## 8. Nạp bitstream

Để kiểm tra nhanh:

1. Cắm USB-C của Tang Nano 20K vào máy tính.
2. Mở Gowin Programmer và Scan Device.
3. Chọn `SRAM Mode` và `SRAM Program`.
4. Chọn file `impl/pnr/picorv32_20k.fs`.
5. Bấm Program/Configure và chờ báo thành công.
6. Mở Serial Debug Assistant ở 115200 8N1.
7. Nhấn S1 để reset và theo dõi dữ liệu.

SRAM Program sẽ mất chương trình khi ngắt nguồn. Sau khi kiểm tra ổn định, có
thể chọn `External Flash Mode` và `Generic Flash` để chương trình tự chạy khi
cấp nguồn.

## 9. Tham số Kalman cần hiệu chỉnh

Các giá trị thử nghiệm nằm trong `c_code/main.c`:

```c
#define KALMAN_DEFAULT_DT           0.1f
#define KALMAN_SIGMA_A_MM_S2     1000.0f
#define KALMAN_R_X_MM2           2500.0f
#define KALMAN_R_Y_MM2           2500.0f
#define MAX_MISSED_FRAMES             5
```

`R_X` và `R_Y` là phương sai theo đơn vị mm². Nên thu dữ liệu của mục tiêu đứng
yên để tính phương sai thực tế trước khi hiệu chỉnh. Sau khi đổi các tham số
này, cần chạy lại Make và Run All để tạo bitstream mới.
