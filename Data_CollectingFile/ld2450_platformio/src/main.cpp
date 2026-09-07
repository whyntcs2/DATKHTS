#include <Arduino.h>
#include "esp_timer.h"

/*
 * LD2450 -> ESP32 -> PC raw-frame bridge
 * PlatformIO target: ESP32 DOIT DevKit V1
 *
 * Default wiring:
 *   LD2450 TX  -> ESP32 GPIO16 (RX2)
 *   LD2450 RX  -> ESP32 GPIO17 (TX2)   [optional for acquisition only]
 *   LD2450 GND -> ESP32 GND
 *   LD2450 5V  -> stable 5V supply
 *
 * LD2450 UART: 256000 baud, 8N1
 * PC USB Serial: 115200 baud
 *
 * Output, one validated radar frame per line:
 *   F,<frame_id>,<esp_time_us>,<60 hex chars>
 */

static constexpr int RADAR_RX_PIN = 16;
static constexpr int RADAR_TX_PIN = 17;

static constexpr uint32_t RADAR_BAUD = 256000;
static constexpr uint32_t PC_BAUD = 115200;

static constexpr uint8_t HEADER[4] = {0xAA, 0xFF, 0x03, 0x00};
static constexpr uint8_t TAIL[2] = {0x55, 0xCC};
static constexpr size_t FRAME_SIZE = 30;

HardwareSerial RadarSerial(2);

static uint8_t frameBuffer[FRAME_SIZE];
static size_t framePos = 0;

enum class ParserState : uint8_t {
    WAIT_AA,
    WAIT_FF,
    WAIT_03,
    WAIT_00,
    READ_REST
};

static ParserState parserState = ParserState::WAIT_AA;
static uint32_t frameId = 0;
static uint32_t badTailCount = 0;

/*
 * LD2450 signed encoding is not normal two's complement:
 *   bit15 = 1 -> positive
 *   bit15 = 0 -> negative
 *   bits14..0 = magnitude
 */
static int16_t decodeLD2450Signed(uint8_t lo, uint8_t hi) {
    const uint16_t raw = static_cast<uint16_t>(lo) |
                         (static_cast<uint16_t>(hi) << 8);
    const int16_t magnitude = static_cast<int16_t>(raw & 0x7FFF);
    return (raw & 0x8000) ? magnitude : static_cast<int16_t>(-magnitude);
}

static uint16_t decodeU16(uint8_t lo, uint8_t hi) {
    return static_cast<uint16_t>(lo) |
           (static_cast<uint16_t>(hi) << 8);
}

static bool targetIsValid(const uint8_t *target8) {
    for (int i = 0; i < 8; ++i) {
        if (target8[i] != 0x00) {
            return true;
        }
    }
    return false;
}

void printHexByte(uint8_t value) {
    const char hexChars[] = "0123456789ABCDEF";

    Serial.write(hexChars[(value >> 4) & 0x0F]);
    Serial.write(hexChars[value & 0x0F]);
}

static void emitFrame(const uint8_t *frame) {
    const uint64_t timestampUs = static_cast<uint64_t>(esp_timer_get_time());

    Serial.print("F,");
    Serial.print(frameId);
    Serial.print(',');
    Serial.printf("%llu", static_cast<unsigned long long>(timestampUs));
    Serial.print(',');

    for (size_t i = 0; i < FRAME_SIZE; ++i) {
        printHexByte(frame[i]);
    }
    Serial.println();

    ++frameId;
}

/* Optional diagnostic helper. Keep disabled during dataset collection. */
static void debugDecodedFrame(const uint8_t *frame) {
    for (int slot = 0; slot < 3; ++slot) {
        const size_t base = 4 + static_cast<size_t>(slot) * 8;
        const uint8_t *p = &frame[base];

        if (!targetIsValid(p)) {
            continue;
        }

        const int16_t x = decodeLD2450Signed(p[0], p[1]);
        const int16_t y = decodeLD2450Signed(p[2], p[3]);
        const int16_t speed = decodeLD2450Signed(p[4], p[5]);
        const uint16_t resolution = decodeU16(p[6], p[7]);

        Serial.print("# target=");
        Serial.print(slot);
        Serial.print(" x_mm=");
        Serial.print(x);
        Serial.print(" y_mm=");
        Serial.print(y);
        Serial.print(" speed_cm_s=");
        Serial.print(speed);
        Serial.print(" resolution_mm=");
        Serial.println(resolution);
    }
}

static void resetParserWithPossibleAA(uint8_t currentByte) {
    framePos = 0;

    if (currentByte == 0xAA) {
        frameBuffer[0] = 0xAA;
        framePos = 1;
        parserState = ParserState::WAIT_FF;
    } else {
        parserState = ParserState::WAIT_AA;
    }
}

static void feedRadarByte(uint8_t b) {
    switch (parserState) {
        case ParserState::WAIT_AA:
            if (b == HEADER[0]) {
                frameBuffer[0] = b;
                framePos = 1;
                parserState = ParserState::WAIT_FF;
            }
            break;

        case ParserState::WAIT_FF:
            if (b == HEADER[1]) {
                frameBuffer[1] = b;
                framePos = 2;
                parserState = ParserState::WAIT_03;
            } else {
                resetParserWithPossibleAA(b);
            }
            break;

        case ParserState::WAIT_03:
            if (b == HEADER[2]) {
                frameBuffer[2] = b;
                framePos = 3;
                parserState = ParserState::WAIT_00;
            } else {
                resetParserWithPossibleAA(b);
            }
            break;

        case ParserState::WAIT_00:
            if (b == HEADER[3]) {
                frameBuffer[3] = b;
                framePos = 4;
                parserState = ParserState::READ_REST;
            } else {
                resetParserWithPossibleAA(b);
            }
            break;

        case ParserState::READ_REST:
            frameBuffer[framePos++] = b;

            if (framePos == FRAME_SIZE) {
                const bool tailOk =
                    frameBuffer[FRAME_SIZE - 2] == TAIL[0] &&
                    frameBuffer[FRAME_SIZE - 1] == TAIL[1];

                if (tailOk) {
                    emitFrame(frameBuffer);

                    // For manual debugging only. Disable during dataset logging.
                    // debugDecodedFrame(frameBuffer);
                } else {
                    ++badTailCount;
                    Serial.print("# bad_tail_count=");
                    Serial.println(badTailCount);
                }

                framePos = 0;
                parserState = ParserState::WAIT_AA;
            }
            break;
    }
}

void setup() {
    Serial.begin(PC_BAUD);
    delay(300);

    // Arduino-ESP32 requires RX buffer sizing before begin().
    RadarSerial.setRxBufferSize(1024);
    RadarSerial.begin(
        RADAR_BAUD,
        SERIAL_8N1,
        RADAR_RX_PIN,
        RADAR_TX_PIN
    );

    Serial.println("# LD2450 acquisition bridge ready");
    Serial.print("# radar_baud=");
    Serial.println(RADAR_BAUD);
    Serial.print("# radar_rx_gpio=");
    Serial.println(RADAR_RX_PIN);
    Serial.print("# radar_tx_gpio=");
    Serial.println(RADAR_TX_PIN);
    Serial.println("# output=F,frame_id,esp_time_us,raw_frame_hex");
}

void loop() {
    while (RadarSerial.available() > 0) {
        const uint8_t b = static_cast<uint8_t>(RadarSerial.read());
        feedRadarByte(b);
    }
}
