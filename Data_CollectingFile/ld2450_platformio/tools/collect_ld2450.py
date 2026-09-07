import serial
import csv
import time
from datetime import datetime
from pathlib import Path
# Config
PORT = "COM3"
BAUD = 115200

# ----- Thông tin case thu -----

ACTIVITY = "WALK"          # STILL / WALK / FAST
DIRECTION = "APPROACH"     # NA / APPROACH / AWAY / L2R / R2L / DIAG_L / DIAG_R

PEOPLE_COUNT = 3           # 1 / 2 / 3 người

# ID người tham gia trong session
# Ví dụ:
# 1 người: ["P01"]
# 2 người: ["P01", "P02"]
# 3 người: ["P01", "P02", "P03"]
PARTICIPANTS = ["P01", "P02", "P03"]

# Mỗi file nên thu liên tục khoảng 30-60 giây
RECORD_TIME = 60           # giây
COUNTDOWN = 3              # đếm ngược trước khi thu

ENVIRONMENT = "LAB01"

OUTPUT_FOLDER = "dataset"


# =========================================================
# CẤU HÌNH LD2450
# =========================================================

HEADER = bytes.fromhex("AA FF 03 00")
TAIL = bytes.fromhex("55 CC")
FRAME_SIZE = 30


# =========================================================
# HÀM DECODE LD2450
# =========================================================

def decode_signed(lo, hi):
    """
    LD2450:
      bit15 = 1 -> số dương
      bit15 = 0 -> số âm
      bit14..0 = độ lớn
    """
    raw = lo | (hi << 8)
    value = raw & 0x7FFF

    if raw & 0x8000:
        return value

    return -value


def decode_u16(lo, hi):
    return lo | (hi << 8)


def parse_target(data):
    valid = 1 if any(data) else 0

    x = decode_signed(data[0], data[1])
    y = decode_signed(data[2], data[3])
    speed = decode_signed(data[4], data[5])
    resolution = decode_u16(data[6], data[7])

    return {
        "valid": valid,
        "x_mm": x,
        "y_mm": y,
        "speed_cm_s": speed,
        "resolution_mm": resolution
    }


def parse_frame(frame):

    if len(frame) != FRAME_SIZE:
        return None

    if frame[:4] != HEADER:
        return None

    if frame[-2:] != TAIL:
        return None

    targets = []

    for slot in range(3):

        start = 4 + slot * 8
        block = frame[start:start + 8]

        target = parse_target(block)

        target["target_slot"] = slot

        targets.append(target)

    return targets


def parse_esp32_line(line):
    """
    ESP32 gửi dạng:

    F,frame_id,esp_time_us,AAFF....55CC
    """

    if not line.startswith("F,"):
        return None

    parts = line.split(",", 3)

    if len(parts) != 4:
        return None

    try:
        frame_id = int(parts[1])
        esp_time_us = int(parts[2])
        frame = bytes.fromhex(parts[3])

    except ValueError:
        return None

    return frame_id, esp_time_us, frame


# =========================================================
# KIỂM TRA CẤU HÌNH
# =========================================================

def check_config():

    if PEOPLE_COUNT not in [1, 2, 3]:
        raise ValueError("PEOPLE_COUNT phải là 1, 2 hoặc 3")

    if len(PARTICIPANTS) != PEOPLE_COUNT:
        raise ValueError(
            f"PEOPLE_COUNT = {PEOPLE_COUNT} nhưng PARTICIPANTS có "
            f"{len(PARTICIPANTS)} người"
        )

    activity_list = ["STILL", "WALK", "FAST"]

    if ACTIVITY.upper() not in activity_list:
        raise ValueError(
            "ACTIVITY phải là STILL, WALK hoặc FAST"
        )


# =========================================================
# MAIN
# =========================================================

def main():

    check_config()

    activity = ACTIVITY.upper()
    direction = DIRECTION.upper()

    participants_text = "-".join(PARTICIPANTS)

    scenario = (
        f"{activity}_{PEOPLE_COUNT}P_{direction}"
    )

    session_id = datetime.now().strftime(
        "%Y%m%d_%H%M%S"
    )

    # -----------------------------------------------------
    # Hiển thị cấu hình
    # -----------------------------------------------------

    print()
    print("==================================================")
    print("       LD2450 MULTI-TARGET DATA COLLECTION")
    print("==================================================")
    print()

    print(f"PORT         : {PORT}")
    print(f"ACTIVITY     : {activity}")
    print(f"DIRECTION    : {direction}")
    print(f"PEOPLE COUNT : {PEOPLE_COUNT}")
    print(f"PARTICIPANTS : {participants_text}")
    print(f"SCENARIO     : {scenario}")
    print(f"TIME         : {RECORD_TIME} s")

    print()

    # -----------------------------------------------------
    # Tạo thư mục
    # -----------------------------------------------------

    output_dir = Path(OUTPUT_FOLDER)
    output_dir.mkdir(
        parents=True,
        exist_ok=True
    )

    filename = (
        f"{session_id}_"
        f"{scenario}_"
        f"{participants_text}.csv"
    )

    filepath = output_dir / filename

    # -----------------------------------------------------
    # Mở serial
    # -----------------------------------------------------

    try:

        ser = serial.Serial(
            PORT,
            BAUD,
            timeout=0.2
        )

    except Exception as e:

        print()
        print("KHÔNG MỞ ĐƯỢC SERIAL!")
        print(e)

        print()
        print("Kiểm tra:")
        print("1. PORT có đúng không?")
        print("2. PlatformIO Serial Monitor đã đóng chưa?")
        print("3. ESP32 có đang kết nối không?")

        input("\nNhấn Enter để thoát...")
        return

    print("Đã kết nối ESP32.")

    # ESP32 thường reset khi mở COM
    time.sleep(2)

    ser.reset_input_buffer()

    # -----------------------------------------------------
    # Countdown
    # -----------------------------------------------------

    print()

    for i in range(COUNTDOWN, 0, -1):

        print(f"Bắt đầu sau {i}...")
        time.sleep(1)

    print()
    print("==================================================")
    print("              BẮT ĐẦU THU DATA")
    print("==================================================")
    print()

    # -----------------------------------------------------
    # Biến thống kê
    # -----------------------------------------------------

    start_time = time.time()

    frame_count = 0

    malformed_frame_count = 0

    valid_count_by_slot = [0, 0, 0]

    frames_with_1_target = 0
    frames_with_2_targets = 0
    frames_with_3_targets = 0

    # -----------------------------------------------------
    # Ghi CSV
    # -----------------------------------------------------

    with open(
        filepath,
        "w",
        newline="",
        encoding="utf-8"
    ) as file:

        writer = csv.writer(file)

        writer.writerow([
            "session_id",
            "scenario",

            "pc_time_s",
            "esp_time_us",
            "frame_id",

            "activity",
            "direction",

            "people_count",
            "participants",

            "environment",

            "target_slot",
            "valid",

            "x_mm",
            "y_mm",
            "speed_cm_s",
            "resolution_mm"
        ])

        while (time.time() - start_time) < RECORD_TIME:

            raw_line = ser.readline()

            if not raw_line:
                continue

            line = raw_line.decode(
                "ascii",
                errors="ignore"
            ).strip()

            # ESP32 diagnostic
            if line.startswith("#"):
                continue

            data = parse_esp32_line(line)

            if data is None:
                continue

            frame_id, esp_time_us, frame = data

            targets = parse_frame(frame)

            if targets is None:

                malformed_frame_count += 1
                continue

            pc_time_s = (
                time.time() - start_time
            )

            # ---------------------------------------------
            # Đếm số target valid trong frame
            # ---------------------------------------------

            valid_targets_in_frame = sum(
                target["valid"]
                for target in targets
            )

            if valid_targets_in_frame == 1:
                frames_with_1_target += 1

            elif valid_targets_in_frame == 2:
                frames_with_2_targets += 1

            elif valid_targets_in_frame == 3:
                frames_with_3_targets += 1

            # ---------------------------------------------
            # Ghi 3 target slot
            # ---------------------------------------------

            for target in targets:

                slot = target["target_slot"]

                writer.writerow([
                    session_id,
                    scenario,

                    f"{pc_time_s:.3f}",
                    esp_time_us,
                    frame_id,

                    activity,
                    direction,

                    PEOPLE_COUNT,
                    participants_text,

                    ENVIRONMENT,

                    slot,
                    target["valid"],

                    target["x_mm"],
                    target["y_mm"],
                    target["speed_cm_s"],
                    target["resolution_mm"]
                ])

                if target["valid"]:

                    valid_count_by_slot[slot] += 1

                    print(
                        f"Frame {frame_id:5d} | "
                        f"T{slot} | "
                        f"X={target['x_mm']:6d} mm | "
                        f"Y={target['y_mm']:6d} mm | "
                        f"V={target['speed_cm_s']:5d} cm/s"
                    )

            frame_count += 1

    ser.close()

    # =====================================================
    # THỐNG KÊ
    # =====================================================

    print()
    print("==================================================")
    print("                 KẾT THÚC")
    print("==================================================")
    print()

    print(f"Session ID            : {session_id}")
    print(f"Scenario              : {scenario}")
    print(f"People count          : {PEOPLE_COUNT}")
    print()

    print(f"Frames received       : {frame_count}")
    print(f"Malformed frames      : {malformed_frame_count}")

    print()

    print(
        f"Valid target slot 0   : "
        f"{valid_count_by_slot[0]}"
    )

    print(
        f"Valid target slot 1   : "
        f"{valid_count_by_slot[1]}"
    )

    print(
        f"Valid target slot 2   : "
        f"{valid_count_by_slot[2]}"
    )

    print()

    print(
        f"Frames có 1 target    : "
        f"{frames_with_1_target}"
    )

    print(
        f"Frames có 2 targets   : "
        f"{frames_with_2_targets}"
    )

    print(
        f"Frames có 3 targets   : "
        f"{frames_with_3_targets}"
    )

    print()

    if frame_count > 0:

        fps = frame_count / RECORD_TIME

        print(
            f"Frame rate approx     : "
            f"{fps:.2f} Hz"
        )

        # Tỷ lệ radar nhìn đủ số người mong muốn
        expected_count_frames = 0

        if PEOPLE_COUNT == 1:
            expected_count_frames = frames_with_1_target

        elif PEOPLE_COUNT == 2:
            expected_count_frames = frames_with_2_targets

        elif PEOPLE_COUNT == 3:
            expected_count_frames = frames_with_3_targets

        detection_ratio = (
            expected_count_frames /
            frame_count *
            100.0
        )

        print(
            f"Đúng {PEOPLE_COUNT} target/frame : "
            f"{detection_ratio:.2f}%"
        )

    print()
    print(f"CSV saved:")
    print(filepath.resolve())

    print()
    input("Nhấn Enter để thoát...")


if __name__ == "__main__":
    main()
