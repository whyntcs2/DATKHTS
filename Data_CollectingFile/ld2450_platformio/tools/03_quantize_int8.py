from pathlib import Path
import json

import numpy as np
import pandas as pd

import torch
import torch.nn as nn
import torch.nn.functional as F

from sklearn.model_selection import train_test_split
from sklearn.metrics import (
    accuracy_score,
    f1_score,
    classification_report,
    confusion_matrix
)


# ============================================================
# CONFIG
# ============================================================

FEATURE_FILE = Path(
    "processed/features.csv"
)

MODEL_FILE = Path(
    "model_output/mlp_fp32_best.pt"
)

NORMALIZATION_FILE = Path(
    "model_output/normalization.json"
)

OUTPUT_DIR = Path(
    "model_int8"
)

RANDOM_SEED = 42

TEST_SIZE = 0.20
VAL_SIZE = 0.20


# ============================================================
# MODEL
# ============================================================

class MLP(nn.Module):

    def __init__(
        self,
        input_size,
        hidden1,
        hidden2,
        output_size
    ):

        super().__init__()

        self.net = nn.Sequential(

            nn.Linear(
                input_size,
                hidden1
            ),

            nn.ReLU(),

            nn.Linear(
                hidden1,
                hidden2
            ),

            nn.ReLU(),

            nn.Linear(
                hidden2,
                output_size
            )
        )

    def forward(self, x):
        return self.net(x)


# ============================================================
# SESSION SPLIT
#
# Phải giống 02_train_mlp.py
# ============================================================

def split_sessions(df):

    session_table = (
        df[
            [
                "session_id",
                "activity"
            ]
        ]
        .drop_duplicates(
            subset=["session_id"]
        )
        .reset_index(drop=True)
    )

    train_val, test = train_test_split(

        session_table,

        test_size=TEST_SIZE,

        random_state=
            RANDOM_SEED,

        stratify=
            session_table[
                "activity"
            ]
    )

    relative_val_size = (
        VAL_SIZE /
        (1.0 - TEST_SIZE)
    )

    train, val = train_test_split(

        train_val,

        test_size=
            relative_val_size,

        random_state=
            RANDOM_SEED,

        stratify=
            train_val[
                "activity"
            ]
    )

    train_sessions = set(
        train["session_id"]
    )

    val_sessions = set(
        val["session_id"]
    )

    test_sessions = set(
        test["session_id"]
    )

    train_df = df[
        df["session_id"].isin(
            train_sessions
        )
    ].copy()

    val_df = df[
        df["session_id"].isin(
            val_sessions
        )
    ].copy()

    test_df = df[
        df["session_id"].isin(
            test_sessions
        )
    ].copy()

    return (
        train_df,
        val_df,
        test_df
    )


# ============================================================
# SCALE
# ============================================================

def symmetric_scale(data):

    max_abs = np.max(
        np.abs(data)
    )

    if max_abs < 1e-12:
        return 1.0

    return float(
        max_abs / 127.0
    )


def positive_scale(data):

    max_value = np.max(
        data
    )

    if max_value < 1e-12:
        return 1.0

    return float(
        max_value / 127.0
    )


# ============================================================
# INT8 QUANTIZATION
# ============================================================

def quantize_signed(
    data,
    scale
):

    q = np.rint(
        data / scale
    )

    q = np.clip(
        q,
        -127,
        127
    )

    return q.astype(
        np.int8
    )


def quantize_weight(
    weight
):

    scale = symmetric_scale(
        weight
    )

    q = quantize_signed(
        weight,
        scale
    )

    return (
        q,
        scale
    )


# ============================================================
# BIAS QUANTIZATION
#
# Bias scale:
#
# Sbias = Sinput * Sweight
# ============================================================

def quantize_bias(
    bias,
    input_scale,
    weight_scale
):

    scale = (
        input_scale
        *
        weight_scale
    )

    q = np.rint(
        bias / scale
    ).astype(
        np.int64
    )

    # Check INT32
    if (
        q.min()
        <
        np.iinfo(
            np.int32
        ).min
        or
        q.max()
        >
        np.iinfo(
            np.int32
        ).max
    ):

        raise RuntimeError(
            "Bias overflow INT32"
        )

    return q.astype(
        np.int32
    )


# ============================================================
# INTEGER REQUANTIZATION
#
# y_int8 =
#
# round(
#     ACC *
#     real_multiplier
# )
#
# real_multiplier =
#
# Sacc / Sout
#
# Đưa về:
#
# ACC * M >> SHIFT
#
# ============================================================

def make_multiplier(
    real_multiplier
):

    if real_multiplier <= 0:

        raise ValueError(
            "Multiplier phải > 0"
        )

    # Chọn shift lớn nhất
    # nhưng M vẫn nằm trong signed 31-bit

    for shift in range(
        30,
        0,
        -1
    ):

        M = int(
            round(
                real_multiplier
                *
                (1 << shift)
            )
        )

        if (
            M > 0
            and
            M <= 0x7FFFFFFF
        ):

            return (
                M,
                shift
            )

    raise RuntimeError(
        "Không tìm được "
        "integer multiplier"
    )


# ============================================================
# REQUANT + RELU
#
# ACC sau ReLU >= 0
#
# nên rounding FPGA rất đơn giản:
#
# (x*M + 2^(S-1)) >> S
# ============================================================

def requant_relu(
    acc,
    multiplier,
    shift
):

    x = np.maximum(
        acc,
        0
    ).astype(
        np.int64
    )

    rounding = (
        1 << (shift - 1)
    )

    y = (
        x
        *
        np.int64(
            multiplier
        )
        +
        rounding
    ) >> shift

    y = np.clip(
        y,
        0,
        127
    )

    return y.astype(
        np.int8
    )


# ============================================================
# INT32 RANGE CHECK
# ============================================================

def check_int32(
    name,
    data
):

    min_v = int(
        np.min(data)
    )

    max_v = int(
        np.max(data)
    )

    print(
        f"{name}: "
        f"min={min_v}, "
        f"max={max_v}"
    )

    if (
        min_v
        <
        -2147483648
        or
        max_v
        >
        2147483647
    ):

        raise RuntimeError(
            f"{name} overflow INT32"
        )


# ============================================================
# INTEGER INFERENCE
# ============================================================

def int8_inference(
    X_normalized,
    qmodel
):

    # ========================================================
    # INPUT QUANTIZATION
    # ========================================================

    xq = quantize_signed(
        X_normalized,
        qmodel[
            "input_scale"
        ]
    )

    # ========================================================
    # LAYER 1
    #
    # INT8 * INT8
    # accumulate INT32
    # ========================================================

    acc1 = (
        xq.astype(
            np.int64
        )
        @
        qmodel[
            "w1"
        ].astype(
            np.int64
        ).T
    )

    acc1 += qmodel[
        "b1"
    ].astype(
        np.int64
    )

    # ReLU + requant
    a1q = requant_relu(

        acc1,

        qmodel[
            "m1"
        ],

        qmodel[
            "shift1"
        ]
    )

    # ========================================================
    # LAYER 2
    # ========================================================

    acc2 = (
        a1q.astype(
            np.int64
        )
        @
        qmodel[
            "w2"
        ].astype(
            np.int64
        ).T
    )

    acc2 += qmodel[
        "b2"
    ].astype(
        np.int64
    )

    a2q = requant_relu(

        acc2,

        qmodel[
            "m2"
        ],

        qmodel[
            "shift2"
        ]
    )

    # ========================================================
    # LAYER 3
    #
    # Không cần requant.
    #
    # Vì tất cả output cùng scale.
    #
    # argmax(ACC3)
    #
    # chính là class prediction.
    # ========================================================

    acc3 = (
        a2q.astype(
            np.int64
        )
        @
        qmodel[
            "w3"
        ].astype(
            np.int64
        ).T
    )

    acc3 += qmodel[
        "b3"
    ].astype(
        np.int64
    )

    pred = np.argmax(
        acc3,
        axis=1
    )

    return {
        "xq": xq,
        "a1q": a1q,
        "a2q": a2q,
        "acc1": acc1,
        "acc2": acc2,
        "acc3": acc3,
        "pred": pred
    }


# ============================================================
# HEX EXPORT
# ============================================================

def write_int8_mem(
    filename,
    array
):

    with open(
        filename,
        "w"
    ) as f:

        flat = array.reshape(-1)

        for value in flat:

            v = (
                int(value)
                &
                0xFF
            )

            f.write(
                f"{v:02X}\n"
            )


def write_int32_mem(
    filename,
    array
):

    with open(
        filename,
        "w"
    ) as f:

        flat = array.reshape(-1)

        for value in flat:

            v = (
                int(value)
                &
                0xFFFFFFFF
            )

            f.write(
                f"{v:08X}\n"
            )


# ============================================================
# MAIN
# ============================================================

def main():

    OUTPUT_DIR.mkdir(
        parents=True,
        exist_ok=True
    )

    print(
        "======================================"
    )

    print(
        "LOAD FP32 MODEL"
    )

    print(
        "======================================"
    )

    checkpoint = torch.load(

        MODEL_FILE,

        map_location="cpu"
    )

    feature_columns = checkpoint[
        "feature_columns"
    ]

    label_map = checkpoint[
        "label_map"
    ]

    hidden1 = checkpoint[
        "hidden1"
    ]

    hidden2 = checkpoint[
        "hidden2"
    ]

    input_size = len(
        feature_columns
    )

    output_size = len(
        label_map
    )

    print(
        "Architecture:",
        input_size,
        "->",
        hidden1,
        "->",
        hidden2,
        "->",
        output_size
    )

    model = MLP(
        input_size,
        hidden1,
        hidden2,
        output_size
    )

    model.load_state_dict(
        checkpoint[
            "model_state_dict"
        ]
    )

    model.eval()

    # ========================================================
    # LOAD NORMALIZATION
    # ========================================================

    with open(
        NORMALIZATION_FILE,
        "r"
    ) as f:

        normalization = json.load(
            f
        )

    norm_mean = np.array(
        normalization[
            "mean"
        ],
        dtype=np.float64
    )

    norm_scale = np.array(
        normalization[
            "scale"
        ],
        dtype=np.float64
    )

    # ========================================================
    # DATA
    # ========================================================

    df = pd.read_csv(
        FEATURE_FILE
    )

    df = df[
        df[
            "activity"
        ].isin(
            label_map.keys()
        )
    ].copy()

    df["label"] = (
        df["activity"]
        .map(
            label_map
        )
    )

    (
        train_df,
        val_df,
        test_df
    ) = split_sessions(
        df
    )

    print(
        "\nCalibration windows:",
        len(train_df)
    )

    print(
        "Test windows:",
        len(test_df)
    )

    # ========================================================
    # NORMALIZED DATA
    # ========================================================

    X_train_raw = train_df[
        feature_columns
    ].to_numpy(
        dtype=np.float64
    )

    X_test_raw = test_df[
        feature_columns
    ].to_numpy(
        dtype=np.float64
    )

    X_train = (
        X_train_raw
        -
        norm_mean
    ) / norm_scale

    X_test = (
        X_test_raw
        -
        norm_mean
    ) / norm_scale

    y_test = test_df[
        "label"
    ].to_numpy(
        dtype=int
    )

    # ========================================================
    # EXTRACT FP32 WEIGHTS
    # ========================================================

    state = model.state_dict()

    w1 = state[
        "net.0.weight"
    ].numpy().astype(
        np.float64
    )

    b1 = state[
        "net.0.bias"
    ].numpy().astype(
        np.float64
    )

    w2 = state[
        "net.2.weight"
    ].numpy().astype(
        np.float64
    )

    b2 = state[
        "net.2.bias"
    ].numpy().astype(
        np.float64
    )

    w3 = state[
        "net.4.weight"
    ].numpy().astype(
        np.float64
    )

    b3 = state[
        "net.4.bias"
    ].numpy().astype(
        np.float64
    )

    # ========================================================
    # FP32 CALIBRATION
    #
    # Lấy activation range từ TRAIN.
    # Không lấy TEST.
    # ========================================================

    with torch.no_grad():

        xt = torch.tensor(
            X_train,
            dtype=torch.float32
        )

        z1 = F.linear(
            xt,
            torch.tensor(
                w1,
                dtype=torch.float32
            ),
            torch.tensor(
                b1,
                dtype=torch.float32
            )
        )

        a1 = F.relu(
            z1
        )

        z2 = F.linear(
            a1,
            torch.tensor(
                w2,
                dtype=torch.float32
            ),
            torch.tensor(
                b2,
                dtype=torch.float32
            )
        )

        a2 = F.relu(
            z2
        )

    a1_np = (
        a1.numpy()
        .astype(
            np.float64
        )
    )

    a2_np = (
        a2.numpy()
        .astype(
            np.float64
        )
    )

    # ========================================================
    # QUANTIZATION SCALES
    # ========================================================

    input_scale = (
        symmetric_scale(
            X_train
        )
    )

    a1_scale = (
        positive_scale(
            a1_np
        )
    )

    a2_scale = (
        positive_scale(
            a2_np
        )
    )

    print(
        "\n======================================"
    )

    print(
        "ACTIVATION SCALES"
    )

    print(
        "======================================"
    )

    print(
        "Input scale:",
        input_scale
    )

    print(
        "A1 scale:",
        a1_scale
    )

    print(
        "A2 scale:",
        a2_scale
    )

    # ========================================================
    # WEIGHTS INT8
    # ========================================================

    qw1, sw1 = (
        quantize_weight(
            w1
        )
    )

    qw2, sw2 = (
        quantize_weight(
            w2
        )
    )

    qw3, sw3 = (
        quantize_weight(
            w3
        )
    )

    print(
        "\nWeight scale W1:",
        sw1
    )

    print(
        "Weight scale W2:",
        sw2
    )

    print(
        "Weight scale W3:",
        sw3
    )

    # ========================================================
    # BIAS INT32
    # ========================================================

    qb1 = quantize_bias(
        b1,
        input_scale,
        sw1
    )

    qb2 = quantize_bias(
        b2,
        a1_scale,
        sw2
    )

    qb3 = quantize_bias(
        b3,
        a2_scale,
        sw3
    )

    # ========================================================
    # REQUANT MULTIPLIERS
    # ========================================================

    real_m1 = (
        input_scale
        *
        sw1
        /
        a1_scale
    )

    real_m2 = (
        a1_scale
        *
        sw2
        /
        a2_scale
    )

    m1, shift1 = (
        make_multiplier(
            real_m1
        )
    )

    m2, shift2 = (
        make_multiplier(
            real_m2
        )
    )

    print(
        "\n======================================"
    )

    print(
        "REQUANT"
    )

    print(
        "======================================"
    )

    print(
        f"L1 real multiplier = "
        f"{real_m1}"
    )

    print(
        f"L1 M={m1}, "
        f"SHIFT={shift1}"
    )

    print(
        f"L2 real multiplier = "
        f"{real_m2}"
    )

    print(
        f"L2 M={m2}, "
        f"SHIFT={shift2}"
    )

    # ========================================================
    # QUANT MODEL
    # ========================================================

    qmodel = {

        "input_scale":
            input_scale,

        "w1":
            qw1,

        "b1":
            qb1,

        "a1_scale":
            a1_scale,

        "m1":
            m1,

        "shift1":
            shift1,

        "w2":
            qw2,

        "b2":
            qb2,

        "a2_scale":
            a2_scale,

        "m2":
            m2,

        "shift2":
            shift2,

        "w3":
            qw3,

        "b3":
            qb3
    }

    # ========================================================
    # FP32 TEST
    # ========================================================

    with torch.no_grad():

        logits_fp32 = model(

            torch.tensor(
                X_test,
                dtype=torch.float32
            )
        )

        pred_fp32 = (
            torch.argmax(
                logits_fp32,
                dim=1
            )
            .numpy()
        )

    fp32_acc = accuracy_score(
        y_test,
        pred_fp32
    )

    fp32_f1 = f1_score(
        y_test,
        pred_fp32,
        average="macro"
    )

    # ========================================================
    # INT8 TEST
    # ========================================================

    result = int8_inference(
        X_test,
        qmodel
    )

    pred_int8 = result[
        "pred"
    ]

    int8_acc = accuracy_score(
        y_test,
        pred_int8
    )

    int8_f1 = f1_score(
        y_test,
        pred_int8,
        average="macro"
    )

    agreement = np.mean(
        pred_fp32
        ==
        pred_int8
    )

    # ========================================================
    # ACCUMULATOR RANGE
    # ========================================================

    print(
        "\n======================================"
    )

    print(
        "ACCUMULATOR RANGE"
    )

    print(
        "======================================"
    )

    check_int32(
        "ACC1",
        result[
            "acc1"
        ]
    )

    check_int32(
        "ACC2",
        result[
            "acc2"
        ]
    )

    check_int32(
        "ACC3",
        result[
            "acc3"
        ]
    )

    # ========================================================
    # RESULT
    # ========================================================

    print(
        "\n======================================"
    )

    print(
        "FP32 vs INT8"
    )

    print(
        "======================================"
    )

    print(
        f"FP32 Accuracy : "
        f"{fp32_acc:.4f}"
    )

    print(
        f"INT8 Accuracy : "
        f"{int8_acc:.4f}"
    )

    print(
        f"FP32 Macro F1 : "
        f"{fp32_f1:.4f}"
    )

    print(
        f"INT8 Macro F1 : "
        f"{int8_f1:.4f}"
    )

    print(
        f"Prediction agreement: "
        f"{agreement * 100:.2f}%"
    )

    print(
        "\nINT8 Classification report:\n"
    )

    # Map index -> name
    class_names = [
        None
    ] * len(
        label_map
    )

    for name, index in (
        label_map.items()
    ):

        class_names[
            index
        ] = name

    print(
        classification_report(
            y_test,
            pred_int8,
            target_names=
                class_names,
            digits=4,
            zero_division=0
        )
    )

    cm = confusion_matrix(
        y_test,
        pred_int8
    )

    print(
        "INT8 Confusion Matrix:"
    )

    print(
        cm
    )

    # ========================================================
    # SAVE NPZ
    # ========================================================

    np.savez(

        OUTPUT_DIR /
        "mlp_int8.npz",

        w1=qw1,
        b1=qb1,

        w2=qw2,
        b2=qb2,

        w3=qw3,
        b3=qb3
    )

    # ========================================================
    # EXPORT .MEM FOR VERILOG
    # ========================================================

    write_int8_mem(

        OUTPUT_DIR /
        "w1.mem",

        qw1
    )

    write_int32_mem(

        OUTPUT_DIR /
        "b1.mem",

        qb1
    )

    write_int8_mem(

        OUTPUT_DIR /
        "w2.mem",

        qw2
    )

    write_int32_mem(

        OUTPUT_DIR /
        "b2.mem",

        qb2
    )

    write_int8_mem(

        OUTPUT_DIR /
        "w3.mem",

        qw3
    )

    write_int32_mem(

        OUTPUT_DIR /
        "b3.mem",

        qb3
    )

    # ========================================================
    # CONFIG JSON
    # ========================================================

    config = {

        "architecture": [
            input_size,
            hidden1,
            hidden2,
            output_size
        ],

        "labels":
            label_map,

        "features":
            feature_columns,

        "input_scale":
            input_scale,

        "w1_scale":
            sw1,

        "a1_scale":
            a1_scale,

        "w2_scale":
            sw2,

        "a2_scale":
            a2_scale,

        "w3_scale":
            sw3,

        "requant1": {

            "real_multiplier":
                real_m1,

            "multiplier":
                m1,

            "shift":
                shift1
        },

        "requant2": {

            "real_multiplier":
                real_m2,

            "multiplier":
                m2,

            "shift":
                shift2
        },

        "fp32_accuracy":
            float(
                fp32_acc
            ),

        "int8_accuracy":
            float(
                int8_acc
            ),

        "fp32_macro_f1":
            float(
                fp32_f1
            ),

        "int8_macro_f1":
            float(
                int8_f1
            ),

        "prediction_agreement":
            float(
                agreement
            )
    }

    with open(

        OUTPUT_DIR /
        "quant_config.json",

        "w"
    ) as f:

        json.dump(
            config,
            f,
            indent=4
        )

    # ========================================================
    # GOLDEN VECTORS
    #
    # Đây sẽ dùng làm testbench FPGA.
    # ========================================================

    golden = test_df[
        [
            "session_id",
            "scenario",
            "window_id",
            "activity",
            "direction"
        ]
    ].copy()

    golden = golden.reset_index(
        drop=True
    )

    # INT8 input
    for i in range(
        input_size
    ):

        golden[
            f"x{i}"
        ] = result[
            "xq"
        ][
            :,
            i
        ]

    # Final INT32 logits
    for i in range(
        output_size
    ):

        golden[
            f"acc3_{i}"
        ] = result[
            "acc3"
        ][
            :,
            i
        ]

    golden[
        "true_class"
    ] = y_test

    golden[
        "fp32_pred"
    ] = pred_fp32

    golden[
        "int8_pred"
    ] = pred_int8

    golden.to_csv(

        OUTPUT_DIR /
        "golden_vectors.csv",

        index=False
    )

    print(
        "\n======================================"
    )

    print(
        "FILES EXPORTED"
    )

    print(
        "======================================"
    )

    print(
        OUTPUT_DIR /
        "mlp_int8.npz"
    )

    print(
        OUTPUT_DIR /
        "quant_config.json"
    )

    print(
        OUTPUT_DIR /
        "w1.mem"
    )

    print(
        OUTPUT_DIR /
        "b1.mem"
    )

    print(
        OUTPUT_DIR /
        "w2.mem"
    )

    print(
        OUTPUT_DIR /
        "b2.mem"
    )

    print(
        OUTPUT_DIR /
        "w3.mem"
    )

    print(
        OUTPUT_DIR /
        "b3.mem"
    )

    print(
        OUTPUT_DIR /
        "golden_vectors.csv"
    )


# ============================================================
# RUN
# ============================================================

if __name__ == "__main__":
    main()