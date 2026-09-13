from pathlib import Path
import json
import random

import numpy as np
import pandas as pd

import matplotlib.pyplot as plt

from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
from sklearn.metrics import (
    accuracy_score,
    f1_score,
    classification_report,
    confusion_matrix
)

import torch
import torch.nn as nn
from torch.utils.data import (
    TensorDataset,
    DataLoader
)


# ============================================================
# CONFIG
# ============================================================

FEATURE_FILE = Path(
    "processed/features.csv"
)

OUTPUT_DIR = Path(
    "model_output"
)

RANDOM_SEED = 42

TEST_SIZE = 0.20
VAL_SIZE = 0.20

HIDDEN1 = 24
HIDDEN2 = 12

EPOCHS = 300
BATCH_SIZE = 64

LEARNING_RATE = 1e-3
WEIGHT_DECAY = 1e-5

PATIENCE = 30

DEVICE = (
    "cuda"
    if torch.cuda.is_available()
    else "cpu"
)


FEATURE_COLUMNS = [

    "dx_mm",
    "dy_mm",

    "mean_vx_mm_s",
    "mean_vy_mm_s",

    "mean_speed_mm_s",

    "median_speed_mm_s",

    "max_speed_mm_s",

    "p90_speed_mm_s",

    "mean_accel_mm_s2",

    "median_accel_mm_s2",

    "max_accel_mm_s2",

    "std_speed_mm_s",

    "path_length_mm",
    "net_displacement_mm",

    "straightness"
]


LABEL_MAP = {
    "STILL": 0,
    "WALK": 1,
    "FAST": 2
}

CLASS_NAMES = [
    "STILL",
    "WALK",
    "FAST"
]


# ============================================================
# RANDOM SEED
# ============================================================

def set_seed(seed):
    random.seed(seed)
    np.random.seed(seed)

    torch.manual_seed(seed)

    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)


# ============================================================
# MODEL
# ============================================================

class MLP(nn.Module):

    def __init__(self):
        super().__init__()

        self.net = nn.Sequential(

            nn.Linear(
                len(FEATURE_COLUMNS),
                HIDDEN1
            ),

            nn.ReLU(),

            nn.Linear(
                HIDDEN1,
                HIDDEN2
            ),

            nn.ReLU(),

            nn.Linear(
                HIDDEN2,
                len(CLASS_NAMES)
            )
        )

    def forward(self, x):
        return self.net(x)


# ============================================================
# SESSION-LEVEL SPLIT
# ============================================================

def split_sessions(df):
    session_table = (
        df[
            ["session_id", "activity"]
        ]
        .drop_duplicates(
            subset=["session_id"]
        )
        .reset_index(drop=True)
    )

    print(
        "\nSố session từng class:"
    )

    print(
        session_table[
            "activity"
        ].value_counts()
    )

    train_val, test = train_test_split(
        session_table,
        test_size=TEST_SIZE,
        random_state=RANDOM_SEED,
        stratify=session_table["activity"]
    )

    relative_val_size = (
        VAL_SIZE /
        (1.0 - TEST_SIZE)
    )

    train, val = train_test_split(
        train_val,
        test_size=relative_val_size,
        random_state=RANDOM_SEED,
        stratify=train_val["activity"]
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
# DATASET
# ============================================================

def make_loader(
    X,
    y,
    shuffle
):

    X_tensor = torch.tensor(
        X,
        dtype=torch.float32
    )

    y_tensor = torch.tensor(
        y,
        dtype=torch.long
    )

    dataset = TensorDataset(
        X_tensor,
        y_tensor
    )

    return DataLoader(
        dataset,
        batch_size=BATCH_SIZE,
        shuffle=shuffle
    )


# ============================================================
# EVALUATE
# ============================================================

def evaluate(
    model,
    loader,
    criterion
):

    model.eval()

    total_loss = 0
    ys = []
    preds = []

    with torch.no_grad():

        for X, y in loader:

            X = X.to(DEVICE)
            y = y.to(DEVICE)

            logits = model(X)

            loss = criterion(
                logits,
                y
            )

            total_loss += (
                loss.item() *
                len(y)
            )

            pred = torch.argmax(
                logits,
                dim=1
            )

            ys.extend(
                y.cpu().numpy()
            )

            preds.extend(
                pred.cpu().numpy()
            )

    mean_loss = (
        total_loss /
        len(loader.dataset)
    )

    accuracy = accuracy_score(
        ys,
        preds
    )

    macro_f1 = f1_score(
        ys,
        preds,
        average="macro"
    )

    return (
        mean_loss,
        accuracy,
        macro_f1,
        np.array(ys),
        np.array(preds)
    )


# ============================================================
# MAIN
# ============================================================

def main():

    set_seed(
        RANDOM_SEED
    )

    OUTPUT_DIR.mkdir(
        parents=True,
        exist_ok=True
    )

    print(
        f"Device: {DEVICE}"
    )

    df = pd.read_csv(
        FEATURE_FILE
    )

    df = df[
        df["activity"].isin(
            LABEL_MAP.keys()
        )
    ].copy()

    df["label"] = (
        df["activity"]
        .map(LABEL_MAP)
    )

    print("\nWindows:")
    print(
        df["activity"]
        .value_counts()
    )

    # ========================================================
    # SPLIT THEO SESSION
    # ========================================================

    (
        train_df,
        val_df,
        test_df
    ) = split_sessions(df)

    print("\nSplit windows:")

    print(
        f"Train = {len(train_df)}"
    )

    print(
        f"Val   = {len(val_df)}"
    )

    print(
        f"Test  = {len(test_df)}"
    )

    print("\nTRAIN:")
    print(
        train_df[
            "activity"
        ].value_counts()
    )

    print("\nVAL:")
    print(
        val_df[
            "activity"
        ].value_counts()
    )

    print("\nTEST:")
    print(
        test_df[
            "activity"
        ].value_counts()
    )

    # ========================================================
    # X / Y
    # ========================================================

    X_train = train_df[
        FEATURE_COLUMNS
    ].to_numpy(float)

    X_val = val_df[
        FEATURE_COLUMNS
    ].to_numpy(float)

    X_test = test_df[
        FEATURE_COLUMNS
    ].to_numpy(float)

    y_train = train_df[
        "label"
    ].to_numpy(int)

    y_val = val_df[
        "label"
    ].to_numpy(int)

    y_test = test_df[
        "label"
    ].to_numpy(int)

    # ========================================================
    # STANDARDIZATION
    # ========================================================

    scaler = StandardScaler()

    X_train = scaler.fit_transform(
        X_train
    )

    X_val = scaler.transform(
        X_val
    )

    X_test = scaler.transform(
        X_test
    )

    scaler_info = {
        "features":
            FEATURE_COLUMNS,

        "mean":
            scaler.mean_.tolist(),

        "scale":
            scaler.scale_.tolist()
    }

    with open(
        OUTPUT_DIR /
        "normalization.json",
        "w"
    ) as f:

        json.dump(
            scaler_info,
            f,
            indent=4
        )

    # ========================================================
    # DATA LOADERS
    # ========================================================

    train_loader = make_loader(
        X_train,
        y_train,
        True
    )

    val_loader = make_loader(
        X_val,
        y_val,
        False
    )

    test_loader = make_loader(
        X_test,
        y_test,
        False
    )

    # ========================================================
    # CLASS WEIGHTS
    # ========================================================

    counts = np.bincount(
        y_train,
        minlength=len(CLASS_NAMES)
    )

    class_weights = (
        len(y_train) /
        (
            len(CLASS_NAMES) *
            counts
        )
    )

    class_weights = torch.tensor(
        class_weights,
        dtype=torch.float32,
        device=DEVICE
    )

    print(
        "\nClass weights:",
        class_weights
    )

    # ========================================================
    # MODEL
    # ========================================================

    model = MLP().to(
        DEVICE
    )

    criterion = nn.CrossEntropyLoss(
        weight=class_weights
    )

    optimizer = torch.optim.Adam(
        model.parameters(),
        lr=LEARNING_RATE,
        weight_decay=WEIGHT_DECAY
    )

    # ========================================================
    # TRAIN
    # ========================================================

    best_val_loss = float("inf")
    patience_counter = 0

    train_losses = []
    val_losses = []

    best_file = (
        OUTPUT_DIR /
        "mlp_fp32_best.pt"
    )

    for epoch in range(
        1,
        EPOCHS + 1
    ):

        model.train()

        total_train_loss = 0

        for X, y in train_loader:

            X = X.to(DEVICE)
            y = y.to(DEVICE)

            optimizer.zero_grad()

            logits = model(X)

            loss = criterion(
                logits,
                y
            )

            loss.backward()

            optimizer.step()

            total_train_loss += (
                loss.item() *
                len(y)
            )

        train_loss = (
            total_train_loss /
            len(train_loader.dataset)
        )

        (
            val_loss,
            val_acc,
            val_f1,
            _,
            _
        ) = evaluate(
            model,
            val_loader,
            criterion
        )

        train_losses.append(
            train_loss
        )

        val_losses.append(
            val_loss
        )

        if epoch == 1 or epoch % 10 == 0:

            print(
                f"Epoch {epoch:3d} | "
                f"train_loss={train_loss:.4f} | "
                f"val_loss={val_loss:.4f} | "
                f"val_acc={val_acc:.4f} | "
                f"val_f1={val_f1:.4f}"
            )

        # ====================================================
        # EARLY STOPPING
        # ====================================================

        if val_loss < best_val_loss:

            best_val_loss = val_loss

            patience_counter = 0

            torch.save(
                {
                    "model_state_dict":
                        model.state_dict(),

                    "feature_columns":
                        FEATURE_COLUMNS,

                    "label_map":
                        LABEL_MAP,

                    "hidden1":
                        HIDDEN1,

                    "hidden2":
                        HIDDEN2
                },
                best_file
            )

        else:

            patience_counter += 1

        if patience_counter >= PATIENCE:

            print(
                f"\nEarly stopping "
                f"epoch {epoch}"
            )

            break

    # ========================================================
    # LOAD BEST
    # ========================================================

    checkpoint = torch.load(
        best_file,
        map_location=DEVICE
    )

    model.load_state_dict(
        checkpoint[
            "model_state_dict"
        ]
    )

    # ========================================================
    # TEST
    # ========================================================

    (
        test_loss,
        test_acc,
        test_f1,
        y_true,
        y_pred
    ) = evaluate(
        model,
        test_loader,
        criterion
    )

    print("\n==============================")
    print("TEST RESULT")
    print("==============================")

    print(
        f"Loss     = {test_loss:.4f}"
    )

    print(
        f"Accuracy = {test_acc:.4f}"
    )

    print(
        f"Macro F1 = {test_f1:.4f}"
    )

    print("\nClassification report:\n")

    print(
        classification_report(
            y_true,
            y_pred,
            target_names=CLASS_NAMES,
            digits=4,
            zero_division=0
        )
    )

    # ========================================================
    # CONFUSION MATRIX
    # ========================================================

    cm = confusion_matrix(
        y_true,
        y_pred,
        labels=[
            0,
            1,
            2
        ]
    )

    cm_df = pd.DataFrame(
        cm,
        index=CLASS_NAMES,
        columns=CLASS_NAMES
    )

    cm_df.to_csv(
        OUTPUT_DIR /
        "confusion_matrix.csv"
    )

    fig, ax = plt.subplots(
        figsize=(6, 5)
    )

    im = ax.imshow(cm)

    ax.set_xticks(
        range(len(CLASS_NAMES))
    )

    ax.set_yticks(
        range(len(CLASS_NAMES))
    )

    ax.set_xticklabels(
        CLASS_NAMES
    )

    ax.set_yticklabels(
        CLASS_NAMES
    )

    ax.set_xlabel(
        "Predicted"
    )

    ax.set_ylabel(
        "True"
    )

    ax.set_title(
        "Confusion Matrix"
    )

    for i in range(
        len(CLASS_NAMES)
    ):
        for j in range(
            len(CLASS_NAMES)
        ):

            ax.text(
                j,
                i,
                cm[i, j],
                ha="center",
                va="center"
            )

    fig.tight_layout()

    fig.savefig(
        OUTPUT_DIR /
        "confusion_matrix.png",
        dpi=150
    )

    plt.close(fig)

    # ========================================================
    # TRAIN CURVE
    # ========================================================

    fig, ax = plt.subplots(
        figsize=(7, 4)
    )

    ax.plot(
        train_losses,
        label="Train"
    )

    ax.plot(
        val_losses,
        label="Validation"
    )

    ax.set_xlabel(
        "Epoch"
    )

    ax.set_ylabel(
        "Loss"
    )

    ax.legend()

    fig.tight_layout()

    fig.savefig(
        OUTPUT_DIR /
        "training_curve.png",
        dpi=150
    )

    plt.close(fig)

    # ========================================================
    # PREDICTIONS
    # ========================================================

    pred_df = test_df[
        [
            "session_id",
            "scenario",
            "window_id",
            "activity",
            "direction",
            "start_frame",
            "end_frame"
        ]
    ].copy()

    pred_df["true_label"] = [
        CLASS_NAMES[i]
        for i in y_true
    ]

    pred_df["pred_label"] = [
        CLASS_NAMES[i]
        for i in y_pred
    ]

    pred_df[
        "correct"
    ] = (
        pred_df["true_label"] ==
        pred_df["pred_label"]
    )

    pred_df.to_csv(
        OUTPUT_DIR /
        "test_predictions.csv",
        index=False
    )

    # ========================================================
    # METADATA
    # ========================================================

    metadata = {

        "architecture":
            [
                len(FEATURE_COLUMNS),
                HIDDEN1,
                HIDDEN2,
                len(CLASS_NAMES)
            ],

        "labels":
            LABEL_MAP,

        "features":
            FEATURE_COLUMNS,

        "test_accuracy":
            float(test_acc),

        "test_macro_f1":
            float(test_f1),

        "random_seed":
            RANDOM_SEED
    }

    with open(
        OUTPUT_DIR /
        "model_metadata.json",
        "w"
    ) as f:

        json.dump(
            metadata,
            f,
            indent=4
        )

    print(
        f"\nModel saved: {best_file}"
    )

    print(
        f"Output folder: {OUTPUT_DIR}"
    )


if __name__ == "__main__":
    main()