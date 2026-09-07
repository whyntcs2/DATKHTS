# LD2450 + ESP32 Data Collector — PlatformIO

## Project structure

```text
ld2450_platformio/
├── platformio.ini
├── src/
│   └── main.cpp
└── tools/
    ├── collect_ld2450.py
    └── requirements.txt
```

## Wiring

Default pins in `src/main.cpp`:

```text
LD2450 TX  -> ESP32 GPIO16 (RX2)
LD2450 RX  -> ESP32 GPIO17 (TX2) [optional for acquisition]
LD2450 GND -> ESP32 GND
LD2450 5V  -> stable 5V
```

If your wiring differs, change:

```cpp
static constexpr int RADAR_RX_PIN = 16;
static constexpr int RADAR_TX_PIN = 17;
```

## PlatformIO

Open this entire folder in VS Code + PlatformIO.

Build:

```bash
pio run
```

Upload:

```bash
pio run -t upload
```

Serial monitor:

```bash
pio device monitor
```

Expected output:

```text
# LD2450 acquisition bridge ready
# radar_baud=256000
# radar_rx_gpio=16
# radar_tx_gpio=17
F,0,1532043,AAFF0300...55CC
F,1,1631991,AAFF0300...55CC
```

## Python data logger

Install dependency once:

```bash
python -m pip install -r tools/requirements.txt
```

Close PlatformIO Serial Monitor before running the logger because both cannot
normally open the same COM port at the same time.

Example on Windows:

```bash
python tools/collect_ld2450.py ^
  --port COM3 ^
  --subject P01 ^
  --activity WALK ^
  --direction APPROACH ^
  --environment LAB01 ^
  --scenario S001 ^
  --repetition R01 ^
  --pre 2 ^
  --active 10 ^
  --post 2
```

The logger creates raw binary frames, frame CSV, target CSV, metadata and a QC
summary under `dataset/00_raw/<subject>/` relative to the terminal working
directory.
