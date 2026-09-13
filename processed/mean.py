import pandas as pd

df = pd.read_csv("processed/features.csv")

features = [
    "mean_speed_mm_s",
    "max_speed_mm_s",
    "std_speed_mm_s",
    "mean_accel_mm_s2",
    "max_accel_mm_s2",
    "path_length_mm",
    "net_displacement_mm"
]

for cls in ["STILL", "WALK", "FAST"]:
    print("\n====================")
    print(cls)
    print("====================")
    
    d = df[df["activity"] == cls]

    print(
        d[features].describe().loc[
            ["mean", "std", "25%", "50%", "75%"]
        ]
    )