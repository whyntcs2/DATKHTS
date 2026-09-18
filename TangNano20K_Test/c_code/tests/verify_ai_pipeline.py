from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

import numpy as np
import pandas as pd

sys.dont_write_bytecode = True


PROJECT = Path(__file__).resolve().parents[2]
REFERENCE = PROJECT.parent / "Data_CollectingFile" / "ld2450_platformio"
FEATURES = [
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
    "straightness",
]


def load_preprocess():
    path = REFERENCE / "tools" / "01_preprocess.py"
    spec = importlib.util.spec_from_file_location("ld2450_preprocess", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def choose_trajectory(preprocess):
    for path in sorted((REFERENCE / "dataset").glob("*.csv")):
        frame = pd.read_csv(path)
        activity = str(frame["activity"].iloc[0]).upper()
        slot = preprocess.choose_target_slot(frame, activity)
        if slot is None:
            continue
        target = frame[(frame["target_slot"] == slot) & (frame["valid"] == 1)]
        target = target.sort_values("esp_time_us").drop_duplicates("frame_id")
        if len(target) >= 22:
            return path, target.reset_index(drop=True)
    raise RuntimeError("No reference trajectory has enough valid samples")


def python_windows(preprocess, target, mean, scale, input_scale):
    processed = preprocess.add_kinematics(target.copy()).reset_index(drop=True)
    expected = {}
    for start in range(0, len(processed) - 11, preprocess.WINDOW_STRIDE):
        window = processed.iloc[start : start + preprocess.WINDOW_FRAMES]
        dt = np.diff(window["time_s"].to_numpy(float))
        if np.any(dt <= 0.0) or np.any(dt > preprocess.MAX_DT_S):
            continue
        values = preprocess.extract_features(window)
        feature = np.array([values[name] for name in FEATURES], dtype=np.float64)
        normalized = (feature - mean) / scale
        quantized = np.clip(np.rint(normalized / input_scale), -127, 127).astype(np.int8)
        expected[start] = (feature, normalized, quantized)
    return expected


def run_c_pipeline(target):
    compiler = shutil.which("gcc")
    if compiler is None:
        raise RuntimeError("gcc not found")

    with tempfile.TemporaryDirectory(prefix="ld2450_ai_test_") as directory:
        executable = Path(directory) / "ai_pipeline_host.exe"
        subprocess.run(
            [
                compiler,
                "-std=c99",
                "-O2",
                "-Wall",
                "-Wextra",
                "-pedantic",
                str(PROJECT / "c_code" / "tests" / "ai_pipeline_host.c"),
                str(PROJECT / "c_code" / "motion_features.c"),
                "-o",
                str(executable),
            ],
            check=True,
        )
        time_s = target["esp_time_us"].to_numpy(np.float64) / 1e6
        dt = np.diff(time_s, prepend=time_s[0])
        rows = [
            f"{delta:.17g},{x:.17g},{y:.17g}"
            for delta, x, y in zip(dt, target["x_mm"], target["y_mm"])
        ]
        result = subprocess.run(
            [str(executable)],
            input="\n".join(rows) + "\n",
            text=True,
            capture_output=True,
            check=True,
        )

    output = {}
    for line in result.stdout.splitlines():
        fields = line.split(",")
        start = int(fields[0])
        feature = np.array(fields[1:16], dtype=np.float64)
        normalized = np.array(fields[16:31], dtype=np.float64)
        quantized = np.array(fields[31:46], dtype=np.int8)
        output[start] = (feature, normalized, quantized)
    return output


def verify_golden(mean, scale, input_scale):
    features = pd.read_csv(REFERENCE / "processed" / "features.csv")
    golden = pd.read_csv(REFERENCE / "model_int8" / "golden_vectors.csv")
    joined = golden.merge(
        features[["session_id", "window_id", *FEATURES]],
        on=["session_id", "window_id"],
        how="left",
        validate="many_to_one",
    )
    if joined[FEATURES].isna().any().any():
        raise AssertionError("golden vector metadata does not map to features.csv")
    normalized = (joined[FEATURES].to_numpy(np.float64) - mean) / scale
    expected = np.clip(np.rint(normalized / input_scale), -127, 127).astype(np.int8)
    actual = joined[[f"x{i}" for i in range(15)]].to_numpy(np.int8)
    return int(np.count_nonzero(expected == actual)), int(expected.size)


def main():
    preprocess = load_preprocess()
    normalization = json.loads(
        (REFERENCE / "model_output" / "normalization.json").read_text()
    )
    quantization = json.loads(
        (REFERENCE / "model_int8" / "quant_config.json").read_text()
    )
    assert normalization["features"] == FEATURES
    assert quantization["features"] == FEATURES

    mean = np.array(normalization["mean"], dtype=np.float64)
    scale = np.array(normalization["scale"], dtype=np.float64)
    input_scale = float(quantization["input_scale"])
    path, target = choose_trajectory(preprocess)
    expected = python_windows(preprocess, target, mean, scale, input_scale)
    actual = run_c_pipeline(target)
    if expected.keys() != actual.keys():
        raise AssertionError(
            f"window starts differ: Python={sorted(expected)} C={sorted(actual)}"
        )

    feature_errors = []
    normalized_errors = []
    int8_matches = 0
    int8_total = 0
    for start in expected:
        py_feature, py_normalized, py_input = expected[start]
        c_feature, c_normalized, c_input = actual[start]
        feature_errors.extend(np.abs(py_feature - c_feature))
        normalized_errors.extend(np.abs(py_normalized - c_normalized))
        int8_matches += int(np.count_nonzero(py_input == c_input))
        int8_total += py_input.size

    max_feature_error = float(np.max(feature_errors))
    max_normalized_error = float(np.max(normalized_errors))
    golden_matches, golden_total = verify_golden(mean, scale, input_scale)
    print(f"trajectory={path}")
    print(f"windows={len(expected)}")
    print(f"feature_max_abs_error={max_feature_error:.9g}")
    print(f"normalized_max_abs_error={max_normalized_error:.9g}")
    print(f"python_vs_c_int8={int8_matches}/{int8_total}")
    print(f"golden_artifact_int8={golden_matches}/{golden_total}")

    if max_feature_error > 0.1:
        raise AssertionError("feature error exceeds 0.1")
    if max_normalized_error > 0.001:
        raise AssertionError("normalized error exceeds 0.001")
    if int8_matches != int8_total:
        raise AssertionError("Python and C INT8 inputs differ")
    if golden_matches != golden_total:
        raise AssertionError("golden vectors differ from current artifacts")


if __name__ == "__main__":
    main()
