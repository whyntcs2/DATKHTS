#ifndef OLED_H
#define OLED_H

void oled_init(void);
void oled_clear(void);
void oled_set_cursor(unsigned char page, unsigned char col);
void oled_draw_bitmap_128x64(const unsigned char *bitmap);
void oled_show_ptit_logo(void);
void oled_show_fee_logo(void);
#endif