#include "std_int_types.h"
#include "i2c.h"

#define I2C_CTRL   (*(volatile uint32_t*)0x80000040)
#define I2C_STATUS (*(volatile uint32_t*)0x80000044)
#define I2C_ADDR   (*(volatile uint32_t*)0x80000048)
#define I2C_LEN    (*(volatile uint32_t*)0x8000004C)
#define I2C_TX0    (*(volatile uint32_t*)0x80000050)
#define I2C_TX1    (*(volatile uint32_t*)0x80000054)
#define I2C_TX2    (*(volatile uint32_t*)0x80000058)
#define I2C_TX3    (*(volatile uint32_t*)0x8000005C)

static void i2c_wait_done(void)
{
    while (I2C_STATUS & 1) {
    }
}

int i2c_write(uint8_t addr, uint8_t *data, uint8_t len)
{
    uint32_t w0 = 0;
    uint32_t w1 = 0;
    uint32_t w2 = 0;
    uint32_t w3 = 0;
    int i;

    if (len > 16) return -1;

    for (i = 0; i < len; i++) {
        uint32_t b = data[i];

        if (i < 4)
            w0 |= b << (24 - 8*i);
        else if (i < 8)
            w1 |= b << (24 - 8*(i-4));
        else if (i < 12)
            w2 |= b << (24 - 8*(i-8));
        else
            w3 |= b << (24 - 8*(i-12));
    }

    i2c_wait_done();

    I2C_ADDR = addr & 0x7F;
    I2C_TX0 = w0;
    I2C_TX1 = w1;
    I2C_TX2 = w2;
    I2C_TX3 = w3;
    I2C_LEN = len & 0xF;

    I2C_CTRL = 1; // start, write

    i2c_wait_done();

    if (I2C_STATUS & 2)
        return -2;

    return 0;
}