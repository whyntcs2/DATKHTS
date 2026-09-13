import pandas as pd

df = pd.read_csv("processed/features.csv")

fast = df[df["activity"] == "FAST"]

result = (
    fast.groupby("session_id")
    .agg(
        windows=("window_id", "count"),
        mean_speed=("mean_speed_mm_s", "mean"),
        median_speed=("mean_speed_mm_s", "median"),
        min_speed=("mean_speed_mm_s", "min"),
        max_speed=("mean_speed_mm_s", "max"),
        mean_path=("path_length_mm", "mean"),
        direction=("direction", "first")
    )
    .sort_values("median_speed")
)

print(result.to_string())