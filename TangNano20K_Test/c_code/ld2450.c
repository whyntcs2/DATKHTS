#include "std_int_types.h"
#include "readtime.h"
#include "ld2450.h"

#define LD2450_STATUS  (*(volatile uint32_t *) 0x80000070)
#define LD2450_RXDATA  (*(volatile uint32_t *) 0x80000074)
#define LD2450_TXDATA  (*(volatile uint32_t *) 0x80000078)
#define LD2450_CONTROL (*(volatile uint32_t *) 0x8000007c)

#define STATUS_RX_AVAILABLE 1u
#define STATUS_RX_OVERFLOW  2u

#define CONTROL_CLEAR_FIFO     1u
#define CONTROL_CLEAR_OVERFLOW 2u

#define ACK_TIMEOUT_CYCLES CLK_FREQ

static const uint8_t data_header[4] = {0xaa, 0xff, 0x03, 0x00};
static const uint8_t command_header[4] = {0xfd, 0xfc, 0xfb, 0xfa};
static const uint8_t command_footer[4] = {0x04, 0x03, 0x02, 0x01};

static const uint8_t enable_configuration[] = {
    0xfd, 0xfc, 0xfb, 0xfa, 0x04, 0x00, 0xff,
    0x00, 0x01, 0x00, 0x04, 0x03, 0x02, 0x01
};

static const uint8_t enable_multi_target[] = {
    0xfd, 0xfc, 0xfb, 0xfa, 0x02, 0x00, 0x90,
    0x00, 0x04, 0x03, 0x02, 0x01
};

static const uint8_t end_configuration[] = {
    0xfd, 0xfc, 0xfb, 0xfa, 0x02, 0x00, 0xfe,
    0x00, 0x04, 0x03, 0x02, 0x01
};

static uint16_t make_u16(uint8_t low, uint8_t high)
{
    return (uint16_t)low | ((uint16_t)high << 8);
}

static int16_t decode_sign_magnitude(uint16_t raw)
{
    int16_t magnitude = (int16_t)(raw & 0x7fff);

    return (raw & 0x8000) ? magnitude : -magnitude;
}

void ld2450_clear_rx(void)
{
    LD2450_CONTROL = CONTROL_CLEAR_FIFO | CONTROL_CLEAR_OVERFLOW;
}

static int uart_try_read(uint8_t *value)
{
    uint32_t status = LD2450_STATUS;
    uint32_t data;

    if (status & STATUS_RX_OVERFLOW) {
        ld2450_clear_rx();
        return 0;
    }

    if (!(status & STATUS_RX_AVAILABLE))
        return 0;

    data = LD2450_RXDATA;
    if (data == 0xffffffff)
        return 0;

    *value = (uint8_t)data;
    return 1;
}

static int uart_read_before(uint32_t start, uint32_t timeout, uint8_t *value)
{
    while ((uint32_t)(readtime() - start) < timeout) {
        if (uart_try_read(value))
            return 1;
    }

    return 0;
}

static uint8_t uart_read_blocking(void)
{
    uint8_t value;

    while (!uart_try_read(&value)) {}
    return value;
}

static void uart_write(const uint8_t *data, uint32_t length)
{
    uint32_t i;

    for (i = 0; i < length; i++)
        LD2450_TXDATA = data[i];
}

static int wait_for_ack(uint8_t command)
{
    uint8_t payload[64];
    uint8_t byte;
    uint32_t start = readtime();
    uint32_t match = 0;
    uint32_t length;
    uint32_t i;

    while ((uint32_t)(readtime() - start) < ACK_TIMEOUT_CYCLES) {
        if (!uart_read_before(start, ACK_TIMEOUT_CYCLES, &byte))
            return 0;

        if (byte == command_header[match]) {
            match++;
            if (match != 4)
                continue;
        } else {
            match = (byte == command_header[0]) ? 1 : 0;
            continue;
        }

        if (!uart_read_before(start, ACK_TIMEOUT_CYCLES, &byte))
            return 0;
        length = byte;
        if (!uart_read_before(start, ACK_TIMEOUT_CYCLES, &byte))
            return 0;
        length |= (uint32_t)byte << 8;

        if ((length < 4) || (length > sizeof(payload))) {
            match = 0;
            continue;
        }

        for (i = 0; i < length; i++)
            if (!uart_read_before(start, ACK_TIMEOUT_CYCLES, &payload[i]))
                return 0;

        for (i = 0; i < sizeof(command_footer); i++) {
            if (!uart_read_before(start, ACK_TIMEOUT_CYCLES, &byte))
                return 0;
            if (byte != command_footer[i])
                break;
        }

        if ((i == sizeof(command_footer)) &&
            (payload[0] == command) &&
            (payload[1] == 0x01) &&
            (payload[2] == 0x00) &&
            (payload[3] == 0x00))
            return 1;

        match = 0;
    }

    return 0;
}

int ld2450_configure_multi_target(void)
{
    ld2450_clear_rx();

    uart_write(enable_configuration, sizeof(enable_configuration));
    if (!wait_for_ack(0xff))
        return 0;

    uart_write(enable_multi_target, sizeof(enable_multi_target));
    if (!wait_for_ack(0x90))
        return 0;

    uart_write(end_configuration, sizeof(end_configuration));
    if (!wait_for_ack(0xfe))
        return 0;

    ld2450_clear_rx();
    return 1;
}

static void decode_target(ld2450_target_t *target, const uint8_t *data)
{
    uint32_t i;

    target->valid = 0;
    for (i = 0; i < 8; i++)
        if (data[i] != 0)
            target->valid = 1;

    target->x_mm = decode_sign_magnitude(make_u16(data[0], data[1]));
    target->y_mm = decode_sign_magnitude(make_u16(data[2], data[3]));
    target->speed_cm_s = decode_sign_magnitude(make_u16(data[4], data[5]));
    target->resolution_mm = make_u16(data[6], data[7]);
}

int ld2450_read_frame(ld2450_frame_t *frame)
{
    uint8_t data[30];
    uint8_t byte;
    uint32_t match = 0;
    uint32_t i;

    for (;;) {
        byte = uart_read_blocking();
        if (byte == data_header[match]) {
            data[match] = byte;
            match++;
            if (match == sizeof(data_header)) {
                for (i = sizeof(data_header); i < sizeof(data); i++)
                    data[i] = uart_read_blocking();

                if ((data[28] == 0x55) && (data[29] == 0xcc)) {
                    for (i = 0; i < LD2450_TARGET_COUNT; i++)
                        decode_target(&frame->target[i], &data[4 + 8*i]);
                    return 1;
                }

                match = 0;
            }
        } else {
            match = (byte == data_header[0]) ? 1 : 0;
            if (match)
                data[0] = byte;
        }
    }
}
