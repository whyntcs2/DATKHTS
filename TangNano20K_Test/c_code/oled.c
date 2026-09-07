#include "std_int_types.h"
#include "i2c.h"
#include "oled.h"
#include "countdown_timer.h"
#include "ptit_logo_oled.h"
#include "fee_logo_oled.h"
#define OLED_ADDR 0x3C

/*
 * Nếu OLED bị mất nét bên trái hoặc bị lệch,
 * thử đổi giá trị này:
 *
 * SSD1306 thường: 0
 * SH1106 thường : 2
 */
#define OLED_COL_OFFSET 2

static void oled_cmd(uint8_t cmd)
{
    uint8_t buf[2];

    buf[0] = 0x00;   // control byte: command
    buf[1] = cmd;

    i2c_write(OLED_ADDR, buf, 2);
}

static void oled_data(uint8_t data)
{
    uint8_t buf[2];

    buf[0] = 0x40;   // control byte: data
    buf[1] = data;

    i2c_write(OLED_ADDR, buf, 2);
}

void oled_set_cursor(unsigned char page, unsigned char col)
{
    col += OLED_COL_OFFSET;

    oled_cmd(0xB0 + page);                    // page address
    oled_cmd(0x00 + (col & 0x0F));            // lower column
    oled_cmd(0x10 + ((col >> 4) & 0x0F));     // higher column
}

void oled_init(void)
{
    /*
     * Delay một chút để OLED ổn định sau khi cấp nguồn.
     * Giá trị này có thể chỉnh nếu cần.
     */
    cdt_delay(1000000);

    oled_cmd(0xAE); // Display OFF

    oled_cmd(0x20); // Set memory addressing mode
    oled_cmd(0x00); // Horizontal addressing mode

    oled_cmd(0xB0); // Set page start address

    oled_cmd(0xC8); // COM output scan direction remapped

    oled_cmd(0x00); // Low column address
    oled_cmd(0x10); // High column address

    oled_cmd(0x40); // Set start line address

    oled_cmd(0x81); // Set contrast
    oled_cmd(0x7F);

    oled_cmd(0xA1); // Segment remap

    oled_cmd(0xA6); // Normal display

    oled_cmd(0xA8); // Set multiplex ratio
    oled_cmd(0x3F); // 1/64 duty

    oled_cmd(0xA4); // Display follows RAM content

    oled_cmd(0xD3); // Set display offset
    oled_cmd(0x00);

    oled_cmd(0xD5); // Set display clock divide ratio
    oled_cmd(0x80);

    oled_cmd(0xD9); // Set pre-charge period
    oled_cmd(0xF1);

    oled_cmd(0xDA); // Set COM pins hardware configuration
    oled_cmd(0x12);

    oled_cmd(0xDB); // Set VCOMH deselect level
    oled_cmd(0x40);

    oled_cmd(0x8D); // Charge pump setting
    oled_cmd(0x14); // Enable charge pump

    oled_cmd(0xAF); // Display ON
}

void oled_clear(void)
{
    int page;
    int col;

    /*
     * Một số OLED SH1106 có RAM 132 cột.
     * Ghi 132 cột giúp xóa sạch dải trắng ở mép.
     */
    for (page = 0; page < 8; page++) {
        oled_cmd(0xB0 + page);
        oled_cmd(0x00);
        oled_cmd(0x10);

        for (col = 0; col < 132; col++) {
            oled_data(0x00);
        }
    }
}

void oled_draw_bitmap_128x64(const unsigned char *bitmap)
{
    int page;
    int col;
    int index = 0;

    for (page = 0; page < 8; page++) {
        oled_set_cursor(page, 0);

        for (col = 0; col < 128; col++) {
            oled_data(bitmap[index]);
            index++;
        }
    }
}

void oled_show_ptit_logo(void)
{
    oled_clear();
    oled_draw_bitmap_128x64(ptit_logo_128x64);
}
void oled_show_fee_logo(void)
{
    oled_clear();
    oled_draw_bitmap_128x64(fee_logo_128x64);
}
