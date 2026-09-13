from pathlib import Path

import numpy as np
import pandas as pd


# ============================================================
# CONFIG
# ============================================================

RAW_DIR = Path("dataset")
OUTPUT_DIR = Path("processed")

# Lọc X/Y
EMA_ALPHA = 0.35

# Median filter cho speed
SPEED_MEDIAN_WINDOW = 5

# Sliding window
WINDOW_FRAMES = 10
WINDOW_STRIDE = 5

# ============================================================
# ACTIVE / MOVEMENT
#
# Dùng CHUNG cho WALK và FAST.
# Không dùng threshold riêng theo class để tránh ép dataset.
# ============================================================

MOVING_THRESHOLD_MM_S = 250.0

# Window 10 frame phải có >= 70% frame đang chuyển động
ACTIVE_RATIO_MIN = 0.70

# Window WALK/FAST phải dịch chuyển tối thiểu 150 mm
MIN_MOVEMENT_DISPLACEMENT_MM = 150.0

# Xác nhận active start
ACTIVE_CONFIRM_FRAMES = 3

# ============================================================
# STILL
# ============================================================

STILL_TRIM_SEC = 1.0

# ============================================================
# TIMING
# ============================================================

MAX_DT_S = 0.25

# ============================================================
# ENDPOINT
# ============================================================

APPROACH_STOP_Y_MM = 1500.0
AWAY_STOP_Y_MM = 5000.0

L2R_STOP_X_MM = 1500.0
R2L_STOP_X_MM = -1500.0

STOP_CONFIRM_FRAMES = 3

VALID_CLASSES = {
    "STILL",
    "WALK",
    "FAST"
}


# ============================================================
# EMA
# ============================================================

def ema(data, alpha):
    data = np.asarray(
        data,
        dtype=float
    )

    if len(data) == 0:
        return data

    out = np.zeros_like(data)

    out[0] = data[0]

    for i in range(1, len(data)):
        out[i] = (
            alpha * data[i]
            +
            (1.0 - alpha) * out[i - 1]
        )

    return out


# ============================================================
# CHỌN TARGET CHO SESSION 1P
# ============================================================

def choose_target_slot(
    session_df,
    activity
):
    valid = session_df[
        session_df["valid"] == 1
    ].copy()

    slots = valid[
        "target_slot"
    ].unique()

    if len(slots) == 0:
        return None

    # Nếu đúng 1 target
    if len(slots) == 1:
        return slots[0]

    candidates = []

    for slot in slots:

        d = valid[
            valid["target_slot"] == slot
        ].sort_values(
            "esp_time_us"
        )

        if len(d) < 3:
            continue

        x = d[
            "x_mm"
        ].to_numpy(float)

        y = d[
            "y_mm"
        ].to_numpy(float)

        path = np.sum(
            np.sqrt(
                np.diff(x) ** 2
                +
                np.diff(y) ** 2
            )
        )

        displacement = np.sqrt(
            (x[-1] - x[0]) ** 2
            +
            (y[-1] - y[0]) ** 2
        )

        radar_speed = np.abs(
            d["speed_cm_s"]
            .to_numpy(float)
        )

        mean_radar_speed = (
            radar_speed.mean()
        )

        candidates.append(
            {
                "slot": slot,
                "count": len(d),
                "path": path,
                "displacement": displacement,
                "mean_speed": mean_radar_speed
            }
        )

    if len(candidates) == 0:
        return None

    # STILL:
    # ưu tiên target tồn tại lâu,
    # vận tốc radar nhỏ
    if activity == "STILL":

        candidates.sort(
            key=lambda z: (
                z["count"],
                -z["mean_speed"]
            ),
            reverse=True
        )

    # WALK / FAST:
    # ưu tiên target có trajectory chuyển động
    else:

        candidates.sort(
            key=lambda z: (
                z["displacement"],
                z["path"],
                z["count"]
            ),
            reverse=True
        )

    return candidates[0]["slot"]


# ============================================================
# MEDIAN FILTER
# ============================================================

def median_filter(
    data,
    window_size=5
):

    return (
        pd.Series(data)
        .rolling(
            window=window_size,
            center=True,
            min_periods=1
        )
        .median()
        .to_numpy()
    )


# ============================================================
# TÍNH KINEMATICS
# ============================================================

def add_kinematics(df):

    df = df.sort_values(
        "esp_time_us"
    ).copy()

    # ------------------------------------------
    # TIME
    # ------------------------------------------

    t = (
        df["esp_time_us"]
        .to_numpy(float)
        /
        1e6
    )

    # Đưa về time relative
    t = t - t[0]

    # ------------------------------------------
    # RAW POSITION
    # ------------------------------------------

    x_raw = df[
        "x_mm"
    ].to_numpy(float)

    y_raw = df[
        "y_mm"
    ].to_numpy(float)

    # ------------------------------------------
    # EMA POSITION
    # ------------------------------------------

    x = ema(
        x_raw,
        EMA_ALPHA
    )

    y = ema(
        y_raw,
        EMA_ALPHA
    )

    n = len(df)

    vx = np.zeros(n)
    vy = np.zeros(n)

    speed_raw = np.zeros(n)

    # ------------------------------------------
    # VELOCITY
    # ------------------------------------------

    for i in range(1, n):

        dt = (
            t[i]
            -
            t[i - 1]
        )

        if (
            dt <= 0
            or
            dt > MAX_DT_S
        ):
            continue

        vx[i] = (
            x[i]
            -
            x[i - 1]
        ) / dt

        vy[i] = (
            y[i]
            -
            y[i - 1]
        ) / dt

        speed_raw[i] = np.sqrt(
            vx[i] ** 2
            +
            vy[i] ** 2
        )

    # ------------------------------------------
    # MEDIAN FILTER SPEED
    #
    # Loại spike:
    # 800, 850, 7000, 900, 850
    # ------------------------------------------

    speed = median_filter(
        speed_raw,
        SPEED_MEDIAN_WINDOW
    )

    # ------------------------------------------
    # ACCELERATION
    #
    # Quan trọng:
    # tính từ speed đã median-filter
    # ------------------------------------------

    accel = np.zeros(n)

    for i in range(1, n):

        dt = (
            t[i]
            -
            t[i - 1]
        )

        if (
            dt <= 0
            or
            dt > MAX_DT_S
        ):
            continue

        accel[i] = (
            speed[i]
            -
            speed[i - 1]
        ) / dt

    # ------------------------------------------
    # MOVING MASK
    # ------------------------------------------

    moving = (
        speed
        >=
        MOVING_THRESHOLD_MM_S
    )

    moving_ratio = (
        pd.Series(
            moving.astype(float)
        )
        .rolling(
            window=5,
            center=True,
            min_periods=1
        )
        .mean()
        .to_numpy()
    )

    # ------------------------------------------
    # STORE
    # ------------------------------------------

    df["time_s"] = t

    df["x_filtered"] = x
    df["y_filtered"] = y

    df["vx_mm_s"] = vx
    df["vy_mm_s"] = vy

    df["speed_raw_mm_s"] = (
        speed_raw
    )

    df["speed_mm_s"] = speed

    df["accel_mm_s2"] = accel

    df["moving"] = moving.astype(int)

    df["moving_ratio"] = (
        moving_ratio
    )

    return df


# ============================================================
# TÌM ACTIVE START
# ============================================================

def find_active_start(df):

    moving = (
        df["speed_mm_s"]
        .to_numpy()
        >=
        MOVING_THRESHOLD_MM_S
    )

    counter = 0

    for i in range(len(df)):

        if moving[i]:
            counter += 1
        else:
            counter = 0

        if (
            counter
            >=
            ACTIVE_CONFIRM_FRAMES
        ):

            start = (
                i
                -
                ACTIVE_CONFIRM_FRAMES
                +
                1
            )

            # lấy thêm 1 frame trước
            return max(
                0,
                start - 1
            )

    return None


# ============================================================
# TÌM STOP THEO POSITION
# ============================================================

def find_position_stop(
    df,
    direction
):

    direction = str(
        direction
    ).upper()

    x = df[
        "x_filtered"
    ].to_numpy()

    y = df[
        "y_filtered"
    ].to_numpy()

    counter = 0

    for i in range(
        len(df)
    ):

        reached = False

        # --------------------------------
        # APPROACH
        # --------------------------------

        if direction == "APPROACH":

            reached = (
                y[i]
                <=
                APPROACH_STOP_Y_MM
            )

        # --------------------------------
        # AWAY
        # --------------------------------

        elif direction == "AWAY":

            reached = (
                y[i]
                >=
                AWAY_STOP_Y_MM
            )

        # --------------------------------
        # LEFT TO RIGHT
        # --------------------------------

        elif direction == "L2R":

            reached = (
                x[i]
                >=
                L2R_STOP_X_MM
            )

        # --------------------------------
        # RIGHT TO LEFT
        # --------------------------------

        elif direction == "R2L":

            reached = (
                x[i]
                <=
                R2L_STOP_X_MM
            )

        # Direction khác
        else:
            return len(df) - 1

        if reached:
            counter += 1
        else:
            counter = 0

        if (
            counter
            >=
            STOP_CONFIRM_FRAMES
        ):

            return (
                i
                -
                STOP_CONFIRM_FRAMES
                +
                1
            )

    return len(df) - 1


# ============================================================
# CROP SESSION
# ============================================================

def crop_session(
    df,
    activity,
    direction
):

    df = df.reset_index(
        drop=True
    )

    if (
        len(df)
        <
        WINDOW_FRAMES
    ):
        return df.iloc[0:0]

    # ========================================================
    # STILL
    # ========================================================

    if activity == "STILL":

        t0 = (
            df["time_s"]
            .iloc[0]
        )

        t1 = (
            df["time_s"]
            .iloc[-1]
        )

        mask = (
            (
                df["time_s"]
                >=
                t0 + STILL_TRIM_SEC
            )
            &
            (
                df["time_s"]
                <=
                t1 - STILL_TRIM_SEC
            )
        )

        out = df[
            mask
        ].copy()

        if (
            len(out)
            <
            WINDOW_FRAMES
        ):
            return df.copy()

        return out.reset_index(
            drop=True
        )

    # ========================================================
    # WALK / FAST
    # ========================================================

    start = find_active_start(
        df
    )

    if start is None:
        return df.iloc[0:0]

    active = (
        df.iloc[start:]
        .reset_index(drop=True)
    )

    # Stop theo vị trí
    stop = find_position_stop(
        active,
        direction
    )

    active = active.iloc[
        :stop + 1
    ].copy()

    return active.reset_index(
        drop=True
    )


# ============================================================
# FEATURE EXTRACTION
# ============================================================

def extract_features(
    window
):

    x = window[
        "x_filtered"
    ].to_numpy(float)

    y = window[
        "y_filtered"
    ].to_numpy(float)

    vx = window[
        "vx_mm_s"
    ].to_numpy(float)

    vy = window[
        "vy_mm_s"
    ].to_numpy(float)

    speed = window[
        "speed_mm_s"
    ].to_numpy(float)

    accel = window[
        "accel_mm_s2"
    ].to_numpy(float)

    # ------------------------------------------
    # POSITION CHANGE
    # ------------------------------------------

    dx = (
        x[-1]
        -
        x[0]
    )

    dy = (
        y[-1]
        -
        y[0]
    )

    # ------------------------------------------
    # VELOCITY
    # ------------------------------------------

    mean_vx = np.mean(
        vx
    )

    mean_vy = np.mean(
        vy
    )

    mean_speed = np.mean(
        speed
    )

    median_speed = np.median(
        speed
    )

    max_speed = np.max(
        speed
    )

    p90_speed = np.percentile(
        speed,
        90
    )

    std_speed = np.std(
        speed
    )

    # ------------------------------------------
    # ACCELERATION
    # ------------------------------------------

    abs_accel = np.abs(
        accel
    )

    mean_accel = np.mean(
        abs_accel
    )

    median_accel = np.median(
        abs_accel
    )

    max_accel = np.max(
        abs_accel
    )

    # ------------------------------------------
    # PATH
    # ------------------------------------------

    step_distance = np.sqrt(
        np.diff(x) ** 2
        +
        np.diff(y) ** 2
    )

    path_length = np.sum(
        step_distance
    )

    # ------------------------------------------
    # NET DISPLACEMENT
    # ------------------------------------------

    net_displacement = np.sqrt(
        dx ** 2
        +
        dy ** 2
    )

    # ------------------------------------------
    # STRAIGHTNESS
    # ------------------------------------------

    if path_length > 1e-6:

        straightness = (
            net_displacement
            /
            path_length
        )

    else:

        straightness = 0.0

    return {

        # 1
        "dx_mm":
            dx,

        # 2
        "dy_mm":
            dy,

        # 3
        "mean_vx_mm_s":
            mean_vx,

        # 4
        "mean_vy_mm_s":
            mean_vy,

        # 5
        "mean_speed_mm_s":
            mean_speed,

        # 6 - NEW
        "median_speed_mm_s":
            median_speed,

        # 7
        "max_speed_mm_s":
            max_speed,

        # 8 - NEW
        "p90_speed_mm_s":
            p90_speed,

        # 9
        "mean_accel_mm_s2":
            mean_accel,

        # 10 - NEW
        "median_accel_mm_s2":
            median_accel,

        # 11
        "max_accel_mm_s2":
            max_accel,

        # 12
        "std_speed_mm_s":
            std_speed,

        # 13
        "path_length_mm":
            path_length,

        # 14
        "net_displacement_mm":
            net_displacement,

        # 15
        "straightness":
            straightness
    }


# ============================================================
# PROCESS 1 SESSION
# ============================================================

def process_session(
    session_df
):

    activity = str(
        session_df[
            "activity"
        ].iloc[0]
    ).upper()

    direction = str(
        session_df[
            "direction"
        ].iloc[0]
    ).upper()

    if (
        activity
        not in
        VALID_CLASSES
    ):
        return []

    session_id = (
        session_df[
            "session_id"
        ].iloc[0]
    )

    scenario = (
        session_df[
            "scenario"
        ].iloc[0]
    )

    people_count = (
        session_df[
            "people_count"
        ].iloc[0]
    )

    participants = (
        session_df[
            "participants"
        ].iloc[0]
    )

    environment = (
        session_df[
            "environment"
        ].iloc[0]
    )

    # ========================================================
    # CHỌN TARGET
    # ========================================================

    slot = choose_target_slot(
        session_df,
        activity
    )

    if slot is None:

        print(
            f"[SKIP] {session_id}: "
            f"không có target valid"
        )

        return []

    target = session_df[
        (
            session_df[
                "target_slot"
            ] == slot
        )
        &
        (
            session_df[
                "valid"
            ] == 1
        )
    ].copy()

    target = target.sort_values(
        "esp_time_us"
    )

    target = target.drop_duplicates(
        subset=[
            "frame_id"
        ]
    )

    if (
        len(target)
        <
        WINDOW_FRAMES
    ):

        print(
            f"[SKIP] {session_id}: "
            f"quá ít frame"
        )

        return []

    # ========================================================
    # KINEMATICS
    # ========================================================

    target = add_kinematics(
        target
    )

    # ========================================================
    # CROP
    # ========================================================

    target = crop_session(
        target,
        activity,
        direction
    )

    if (
        len(target)
        <
        WINDOW_FRAMES
    ):

        print(
            f"[SKIP] {session_id}: "
            f"không đủ active frame"
        )

        return []

    # ========================================================
    # WINDOW
    # ========================================================

    rows = []

    window_id = 0

    rejected_motion = 0
    rejected_displacement = 0
    rejected_time = 0

    for start in range(
        0,
        len(target)
        -
        WINDOW_FRAMES
        +
        1,
        WINDOW_STRIDE
    ):

        end = (
            start
            +
            WINDOW_FRAMES
        )

        w = target.iloc[
            start:end
        ].copy()

        # ====================================================
        # CHECK TIMESTAMP
        # ====================================================

        dt = np.diff(
            w[
                "time_s"
            ].to_numpy()
        )

        if np.any(
            dt <= 0
        ):

            rejected_time += 1
            continue

        if np.any(
            dt > MAX_DT_S
        ):

            rejected_time += 1
            continue

        # ====================================================
        # MOVING RATIO
        #
        # Chỉ áp dụng WALK / FAST
        #
        # Không ép FAST > WALK.
        # Chỉ loại window thực chất đang đứng.
        # ====================================================

        if activity in {
            "WALK",
            "FAST"
        }:

            moving_ratio = (
                (
                    w[
                        "speed_mm_s"
                    ]
                    >=
                    MOVING_THRESHOLD_MM_S
                )
                .mean()
            )

            if (
                moving_ratio
                <
                ACTIVE_RATIO_MIN
            ):

                rejected_motion += 1
                continue

        else:

            moving_ratio = (
                (
                    w[
                        "speed_mm_s"
                    ]
                    >=
                    MOVING_THRESHOLD_MM_S
                )
                .mean()
            )

        # ====================================================
        # FEATURE
        # ====================================================

        f = extract_features(
            w
        )

        # ====================================================
        # MOVEMENT DISPLACEMENT
        # ====================================================

        if activity in {
            "WALK",
            "FAST"
        }:

            if (
                f[
                    "net_displacement_mm"
                ]
                <
                MIN_MOVEMENT_DISPLACEMENT_MM
            ):

                rejected_displacement += 1
                continue

        # ====================================================
        # SAVE WINDOW
        # ====================================================

        row = {

            "session_id":
                session_id,

            "scenario":
                scenario,

            "window_id":
                window_id,

            "start_frame":
                int(
                    w[
                        "frame_id"
                    ].iloc[0]
                ),

            "end_frame":
                int(
                    w[
                        "frame_id"
                    ].iloc[-1]
                ),

            "activity":
                activity,

            "direction":
                direction,

            "people_count":
                people_count,

            "participants":
                participants,

            "environment":
                environment,

            "selected_slot":
                int(slot),

            "moving_ratio":
                float(
                    moving_ratio
                )
        }

        row.update(
            f
        )

        rows.append(
            row
        )

        window_id += 1

    # ========================================================
    # PRINT SESSION
    # ========================================================

    print(
        f"[OK] "
        f"{session_id} | "
        f"{activity:<5} | "
        f"{direction:<10} | "
        f"slot={slot} | "
        f"frames={len(target):3d} | "
        f"windows={len(rows):3d} | "
        f"rej_motion={rejected_motion:2d} | "
        f"rej_disp={rejected_displacement:2d}"
    )

    return rows


# ============================================================
# MAIN
# ============================================================

def main():

    OUTPUT_DIR.mkdir(
        parents=True,
        exist_ok=True
    )

    files = (
        list(
            RAW_DIR.glob(
                "*.csv"
            )
        )
        +
        list(
            RAW_DIR.glob(
                "*.txt"
            )
        )
    )

    if len(files) == 0:

        print(
            "Không tìm thấy CSV/TXT "
            "trong thư mục dataset/"
        )

        return

    print(
        f"Tìm thấy "
        f"{len(files)} file\n"
    )

    all_frames = []

    # ========================================================
    # READ FILES
    # ========================================================

    for file in files:

        try:

            df = pd.read_csv(
                file
            )

            required = {

                "session_id",
                "scenario",

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

                "speed_cm_s"
            }

            if not required.issubset(
                df.columns
            ):

                print(
                    f"[SKIP FILE] "
                    f"{file.name}: "
                    f"thiếu column"
                )

                continue

            all_frames.append(
                df
            )

        except Exception as e:

            print(
                f"[ERROR] "
                f"{file.name}: "
                f"{e}"
            )

    if len(
        all_frames
    ) == 0:

        print(
            "Không có file hợp lệ."
        )

        return

    # ========================================================
    # CONCAT
    # ========================================================

    raw = pd.concat(
        all_frames,
        ignore_index=True
    )

    feature_rows = []

    # ========================================================
    # SESSION PROCESSING
    # ========================================================

    for (
        session_id,
        session_df
    ) in raw.groupby(
        "session_id",
        sort=False
    ):

        rows = process_session(
            session_df
        )

        feature_rows.extend(
            rows
        )

    # ========================================================
    # FEATURES DATAFRAME
    # ========================================================

    features = pd.DataFrame(
        feature_rows
    )

    if features.empty:

        print(
            "\nKhông tạo được "
            "window nào."
        )

        return

    output_file = (
        OUTPUT_DIR
        /
        "features.csv"
    )

    features.to_csv(
        output_file,
        index=False
    )

    # ========================================================
    # SUMMARY
    # ========================================================

    print(
        "\n"
        "======================================"
    )

    print(
        "KẾT QUẢ PREPROCESSING"
    )

    print(
        "======================================"
    )

    print(
        f"\nTotal windows: "
        f"{len(features)}"
    )

    print(
        "\nTheo activity:"
    )

    print(
        features[
            "activity"
        ].value_counts()
    )

    print(
        "\nTheo activity + direction:"
    )

    print(
        features.groupby(
            [
                "activity",
                "direction"
            ]
        ).size()
    )

    print(
        "\nTheo people_count:"
    )

    print(
        features[
            "people_count"
        ].value_counts()
    )

    # ========================================================
    # FEATURE SUMMARY
    # ========================================================

    print(
        "\n"
        "======================================"
    )

    print(
        "SPEED DISTRIBUTION"
    )

    print(
        "======================================"
    )

    for activity in [
        "STILL",
        "WALK",
        "FAST"
    ]:

        d = features[
            features[
                "activity"
            ] == activity
        ]

        if len(d) == 0:
            continue

        print(
            f"\n--- {activity} ---"
        )

        print(
            d[
                [
                    "mean_speed_mm_s",
                    "median_speed_mm_s",
                    "p90_speed_mm_s",
                    "path_length_mm",
                    "net_displacement_mm"
                ]
            ]
            .describe()
            .loc[
                [
                    "mean",
                    "std",
                    "25%",
                    "50%",
                    "75%"
                ]
            ]
        )

    print(
        f"\nSaved: "
        f"{output_file}"
    )


# ============================================================
# RUN
# ============================================================

if __name__ == "__main__":
    main()