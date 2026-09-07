/* Copyright 2025 Grug Huhler.  License SPDX BSD-2-Clause.

   This is a Tang Nano 20K application for the PicoRV32-based SoC.
*/

#include "std_int_types.h"
#include "leds.h"
#include "uart.h"
#include "countdown_timer.h"
#include "readtime.h"
#include "sd_card_cgpt.h"
#include "oled.h"
#include "ws2812b.h"
#include "ld2450.h"
#include "kalman.h"
#include "motion_features.h"

extern uint32_t timer_instr(uint32_t val);   /* from startup.S */
extern uint32_t maskirq_instr(uint32_t val); /* from startup.S */

uint32_t timer_irq_count;
uint32_t illegal_irq_count;
uint32_t buserr_irq_count;
uint32_t irq3_count;

#define MEMSIZE 512
uint32_t mem[MEMSIZE];
uint32_t test_vals[] = {0, 0xffffffff, 0xaaaaaaaa, 0x55555555, 0xdeadbeef};

/* A simple memory test.  Delete this and also array mem
   above to free much of the SRAM for other things
*/

int mem_test (void)
{
  int i, test, errors;
  uint32_t val, val_read;

  errors = 0;
  for (test = 0; test < sizeof(test_vals)/sizeof(test_vals[0]); test++) {

    for (i = 0; i < MEMSIZE; i++) mem[i] = test_vals[test];

    for (i = 0; i < MEMSIZE; i++) {
      val_read = mem[i];
      if (val_read != test_vals[test]) errors += 1;
    }
  }

  for (i = 0; i < MEMSIZE; i++) mem[i] = i + (i << 17);

  for (i = 0; i < MEMSIZE; i++) {
    val_read = mem[i];
    if (val_read != i + (i << 17)) errors += 1;
  }

  return(errors);
}


void endian_test(void)
{
  volatile uint32_t test_loc = 0;
  volatile uint32_t *addr = &test_loc;
  volatile uint8_t *cp0, *cp3;
  char byte0, byte3;
  uint32_t i, ok;

  cp0 = (volatile uint8_t *) addr;
  cp3 = cp0 + 3;
  *addr = 0x44332211;
  byte0 = *cp0;
  byte3 = *cp3;
  *cp3 = 0xab;
  i = *addr;

  ok = (byte0 == 0x11) && (byte3 == 0x44) && (i == 0xab332211);
  uart_puts("\r\nEndian test: at ");
  uart_print_hex((uint32_t) addr);
  uart_puts(", byte0: ");
  uart_print_hex((uint32_t) byte0);
  uart_puts(", byte3: ");
  uart_print_hex((uint32_t) byte3);
  uart_puts(",\r\n     word: ");
  uart_print_hex(i);
  if (ok)
    uart_puts(" [PASSED]\r\n");
  else
    uart_puts(" [FAILED]\r\n");
}


/* la_functions are useful for looking at the bus using a logic
   analyzer.
*/

void la_wtest(void)
{
  uint32_t v;
  volatile uint32_t *ip = (volatile uint32_t *) &v;
  volatile uint16_t *sp = (volatile uint16_t *) &v;
  volatile uint8_t *cp = (volatile uint8_t *) &v;

  *ip = 0x03020100;  // addr 0x00

  *sp = 0x0302;      // addr 0x00
  *(sp+1) = 0x0100;  // addr 0x02

  *cp = 0x03;        // addr 0x00
  *(cp+1) = 0x02;    // addr 0x01
  *(cp+2) = 0x01;    // addr 0x02
  *(cp+3) = 0x00;    // addr 0x03
}


void la_rtest(void)
{
  uint32_t v;
  volatile uint32_t *ip = (volatile uint32_t *) &v;
  volatile uint16_t *sp = (volatile uint16_t *) &v;
  volatile uint8_t *cp = (volatile uint8_t *) &v;

  *ip = 0x03020100;  // addr 0x00

  *ip;     // addr 0x00

  *sp;     // addr 0x00
  *(sp+1); // addr 0x02

  *cp;     // addr 0x00
  *(cp+1); // addr 0x01
  *(cp+2); // addr 0x02
  *(cp+3); // addr 0x03
}


void countdown_timer_test(void)
{
  uint32_t val;
  uint32_t test_errors = 0;
  
  // If register is little-endian, write to 0x80000013 should set
  // the MSB,  Does it?

  cdt_wbyte3(0xff);
  val = cdt_read();
  if ((val == 0xff000000) || (val < 0xfe000000)) test_errors = 1;

  // Write zero to most significant half-word.
  cdt_whalf2(0);
  val = cdt_read();
  if (val > 0xffff) test_errors |= 2;

  uart_puts("Countdown timer test ");
  if (test_errors) {
    uart_puts("FAILED, mask = ");
    uart_print_hex(test_errors);
    uart_puts("\r\n");
  } else {
    uart_puts("PASSED\r\n");
  }
}

void cycle_delay(uint32_t cycles)
{
  uart_puts("delay ");
  uart_print_hex(cycles);
  uart_puts(" cycles\r\n");
  cdt_delay(cycles);
  uart_puts("done\r\n");
}

void read_led(void)
{
  uint8_t v;
  
  v = get_leds();
  uart_puts("LED = ");
  uart_print_hex(v);
  uart_puts("\r\n");
}

void incr_led(void)
{
  uint8_t v;
  
  v = get_leds();
  set_leds(v+1);
}

void set_led(uint32_t value)
{
  set_leds(value);
}

void memory_test(void)
{
  if (mem_test())
    uart_puts("memory test FAILED.\r\n");
  else
    uart_puts("memory test PASSED.\r\n");
}

void read_clock(void)
{
  uart_puts("time is ");
  uart_print_hex(readtime());
  uart_puts("\r\n");
}

void read_clock_ll(void)
{
  time_ll_t t;

  t = readtime_ll();
  uart_puts("time is ");
  uart_print_hex(t.s.time_high);
  uart_putchar(':');
  uart_print_hex(t.s.time_low);
  uart_puts("\r\n");
}

void toggle_spi(void)
{
  sd_spi_use_helper = 1 - sd_spi_use_helper;
  if (sd_spi_use_helper)
    uart_puts("\r\nusing helper\r\n");
  else
    uart_puts("\r\nusing bit-bang\r\n");
}

void sd_card_init(void)
{
  if (sd_init() >= 0)
    uart_puts("\r\nSD init OK\r\n");
  else
    uart_puts("\r\nSD init failed\r\n");
}

void sd_display_block(uint32_t blk)
{
  uint8_t *buf = (uint8_t *) mem;
  int i;

  if (sd_read_block(blk, buf) < 0) {
    uart_puts("\r\nread failed\r\n");
    return;
  }

  for (i = 0; i < 512/4; i++) {
    uart_print_hex(mem[i]);
    if (i % 8 == 7) uart_puts("\r\n");
    else uart_putchar(' ');
  }

}

void sd_set_block(uint32_t blk, uint32_t start_val)
{
  uint8_t *buf = (uint8_t *) mem;
  int i;

  for (i = 0; i < 512/4; i++) {
    mem[i] = start_val++;
  }

  if (sd_write_block(blk, buf) < 0) {
    uart_puts("\r\nwrite failed\r\n");
    return;
  }
}

void help(void)
{
  uart_puts("ct            : test countdown timer\r\n");
  uart_puts("dc cycles     : delay for cycles\r\n");
  uart_puts("et            : endian test\r\n");
  uart_puts("rl            : read LEDs\r\n");
  uart_puts("il            : increment LEDs\r\n");
  uart_puts("sl value      : set LEDs to value\r\n");
  uart_puts("mt            : memory test\r\n");
  uart_puts("rc            : read clock low\r\n");
  uart_puts("rd            : read clock hi:low\r\n");
  uart_puts("he            : print help\r\n");
  uart_puts("si            : soft reset SD card-- do first\r\n");
  uart_puts("sr            : read specified block from SD\r\n");
  uart_puts("sw            : write specified block of SD\r\n");
  uart_puts("sx            : Toggle using SD SPI helper\r\n");
  uart_puts("bi            : cause bus error\r\n");
  uart_puts("ii            : cause illegal instruction\r\n");
  uart_puts("mi            : set IRQ mask\r\n");
  uart_puts("pi            : print IRQ count\r\n");
  uart_puts("ti            : set timer to val\r\n");
  uart_puts("sn            : set ws2812b LED\r\n");
  uart_puts("rb addr       : read byte\r\n");
  uart_puts("rh addr       : read half word\r\n");
  uart_puts("rw addr       : read word\r\n");
  uart_puts("wb addr value : write byte\r\n");
  uart_puts("wh addr value : write half word\r\n");
  uart_puts("ww addr value : write word\r\n");
  uart_puts("   all numbers are hex\r\n");
}

void read_byte(uint32_t addr)
{
  volatile uint8_t *p = (volatile uint8_t *) addr;

  uart_print_hex(*p);
  uart_puts("\r\n");
}

void read_half(uint32_t addr)
{
  volatile uint16_t *p = (volatile uint16_t *) addr;

  uart_print_hex(*p);
  uart_puts("\r\n");
}

void read_word(uint32_t addr)
{
  volatile uint32_t *p = (volatile uint32_t *) addr;

  uart_print_hex(*p);
  uart_puts("\r\n");
}

void write_byte(uint32_t addr, uint32_t value)
{
  volatile uint8_t *p = (volatile uint8_t *) addr;

  *p = value;
}

void write_half(uint32_t addr, uint32_t value)
{
  volatile uint16_t *p = (volatile uint16_t *) addr;

  *p = value;
}

void write_word(uint32_t addr, uint32_t value)
{
  volatile uint32_t *p = (volatile uint32_t *) addr;

  *p = value;
}

void change_ws2812b(uint32_t value)
{
  uart_puts("setting neopixel to ");
  uart_print_hex(value);
  uart_puts("\r\n");
  set_ws2812b(value);
}

uint32_t *irq(uint32_t *regs, uint32_t irqs)
{
  if ((irqs & 1) != 0) {
    timer_irq_count++;
  }

  if ((irqs & 2) != 0) {
    illegal_irq_count++;
  }

  if ((irqs & 4) != 0) {
    buserr_irq_count++;
  }

  if ((irqs & 8) != 0) {
    irq3_count++;
  }

  return regs;
}

void print_irq_counts(void)
{
  uart_puts("timer: ");
  uart_print_hex(timer_irq_count);
  uart_puts("\r\nillegal: ");
  uart_print_hex(illegal_irq_count);
  uart_puts("\r\nbus error: ");
  uart_print_hex(buserr_irq_count);
  uart_puts("\r\nIRQ3: ");
  uart_print_hex(irq3_count);
  uart_puts("\r\n");
}

void do_timer_instr(uint32_t val)
{
  uint32_t old_val;

  old_val = timer_instr(val);
  uart_puts("old value was ");
  uart_print_hex(old_val);
  uart_puts("\r\n");
}

void do_maskirq_instr(uint32_t val)
{
  uint32_t old_val;

  old_val = maskirq_instr(val);
  uart_puts("old value was ");
  uart_print_hex(old_val);
  uart_puts("\r\n");
}

void illegal(void)
{
  asm volatile ("unimp"); /* Illegal instruction */
}

void buserr(void)
{
  int x, v;
  int *p = &x;

  /* If the compiler detects a misaligned access, it breaks it into a
     sequence of smaller aligned accesses so I use an asm to force a
     misaligned short read */
  asm volatile ("lh %[rd], 1(%[rs])" : [rd] "=r" (v) : [rs] "r" (p));
}


/* struct command lists available commands.  See function help.
 * Each function takes one or two arguments.  The table below
 * contains a pointer to the function to call for each command.
 */

struct command {
  char *cmd_string;
  int num_args;
  union {
    void (*func0)(void);
    void (*func1)(uint32_t val);
    void (*func2)(uint32_t val1, uint32_t val2);
  } u;
} commands[] = {
  {"ct", 0, .u.func0=countdown_timer_test},
  {"dc", 1, .u.func1=cycle_delay}, // cycles
  {"et", 0, .u.func0=endian_test},
  {"rl", 0, .u.func0=read_led},
  {"il", 0, .u.func0=incr_led},
  {"sl", 1, .u.func1=set_led},   // val
  {"mt", 0, .u.func0=memory_test},
  {"rc", 0, .u.func0=read_clock},
  {"rd", 0, .u.func0=read_clock_ll},
  {"he", 0, .u.func0=help},
  {"sx", 0, .u.func0=toggle_spi},
  {"si", 0, .u.func0=sd_card_init},
  {"sr", 1, .u.func1=sd_display_block},
  {"sw", 2, .u.func2=sd_set_block},
  {"bi", 0, .u.func0=buserr},
  {"ii", 0, .u.func0=illegal},
  {"mi", 1, .u.func1=do_maskirq_instr},
  {"pi", 0, .u.func0=print_irq_counts},
  {"ti", 1, .u.func1=do_timer_instr},
  {"sn", 1, .u.func1=change_ws2812b},
  {"rb", 1, .u.func1=read_byte}, // addr
  {"rh", 1, .u.func1=read_half}, // addr
  {"rw", 1, .u.func1=read_word}, // addr
  {"wb", 2, .u.func2=write_byte}, // addr, val
  {"wh", 2, .u.func2=write_half}, // addr, val
  {"ww", 2, .u.func2=write_word} // addr, val
};


void eat_spaces(char **buf, uint32_t *len)
{
  while (len > 0) {
    if (**buf == ' ') {
      *buf += 1;
      *len -= 1;
    } else
      break;
  }
}

/* Returns 1 if a number found, else 0.  Number is in *v */

uint32_t get_hex(char **buf, uint32_t *len, uint32_t *v)
{
  int valid = 0;
  int keep_going;
  char ch;

  keep_going = 1;

  *v = 0;
  while (keep_going && (*len > 0)) {

    ch = **buf;
    *buf += 1;
    *len -= 1;

    if ((ch >= '0') && (ch <= '9')) {
      *v = 16*(*v) + (ch - '0');
      valid = 1;
    } else if ((ch >= 'a') && (ch <= 'f')) {
      *v = 16*(*v) + (ch - 'a' + 10);
      valid = 1;
    } else if ((ch >= 'A') && (ch <= 'F')) {
      *v = 16*(*v) + (ch - 'A' + 10);
      valid = 1;
    } else {
      keep_going = 0;
    }
  }

  return valid;
}


void parse(char *buf, uint32_t len)
{
  int i, cmd_not_ok;
  uint32_t val1, val2;

  cmd_not_ok = 1;
  eat_spaces(&buf, &len);
  if (len < 2) goto err;

  for (i = 0; i < sizeof(commands)/sizeof(commands[0]); i++)
    if ((buf[0] == commands[i].cmd_string[0]) && (buf[1] == commands[i].cmd_string[1])) {
      buf += 2;
      len -= 2;
      switch (commands[i].num_args) {
      case 0:
        commands[i].u.func0();
        cmd_not_ok = 0;
        break;
      case 1:
        eat_spaces(&buf, &len);
        if (get_hex(&buf, &len, &val1)) {
          commands[i].u.func1(val1);
          cmd_not_ok = 0;
        }
        break;
      case 2:
        eat_spaces(&buf, &len);
        if (get_hex(&buf, &len, &val1)) {
          eat_spaces(&buf, &len);
          if (get_hex(&buf, &len, &val2)) {
            commands[i].u.func2(val1, val2);
            cmd_not_ok = 0;
          }
        }
        break;
      default:
        break;
      }
      break;
    }

 err:
  if (cmd_not_ok)
    uart_puts("illegal command, he for help\r\n");
}

#define BUFLEN 64

/* Initial Kalman tuning values. r_x and r_y are variances in mm^2. */
#define KALMAN_DEFAULT_DT           0.1f
#define KALMAN_SIGMA_A_MM_S2     1000.0f
#define KALMAN_R_X_MM2           2500.0f
#define KALMAN_R_Y_MM2           2500.0f
#define MAX_MISSED_FRAMES             5

#ifndef DEBUG_KALMAN
#define DEBUG_KALMAN                  0
#endif

#ifndef DEBUG_AI_FEATURE
#define DEBUG_AI_FEATURE              0
#endif

static void uart_print_unsigned_decimal(uint32_t value)
{
  char digits[10];
  int count = 0;

  do {
    digits[count++] = '0' + value % 10;
    value /= 10;
  } while (value != 0);

  while (count != 0)
    uart_putchar(digits[--count]);
}

static void uart_print_fixed_3(float value)
{
  uint32_t scaled;
  uint32_t fraction;

  if (value < 0.0f)
    value = 0.0f;

  scaled = (uint32_t)(value * 1000.0f + 0.5f);
  fraction = scaled % 1000u;

  uart_print_unsigned_decimal(scaled / 1000u);
  uart_putchar('.');
  uart_putchar('0' + (fraction / 100u));
  uart_putchar('0' + ((fraction / 10u) % 10u));
  uart_putchar('0' + (fraction % 10u));
}

static void uart_print_signed_fixed_3(float value)
{
  if (value < 0.0f) {
    uart_putchar('-');
    value = -value;
  }

  uart_print_fixed_3(value);
}

static void uart_print_signed_fixed_1(float value)
{
  uint32_t scaled;

  if (value < 0.0f) {
    uart_putchar('-');
    value = -value;
  }

  scaled = (uint32_t)(value * 10.0f + 0.5f);
  uart_print_unsigned_decimal(scaled / 10u);
  uart_putchar('.');
  uart_putchar('0' + (scaled % 10u));
}

static void uart_print_signed_decimal(int32_t value)
{
  if (value < 0) {
    uart_putchar('-');
    value = -value;
  }

  uart_print_unsigned_decimal((uint32_t)value);
}

static void update_kalman_tracks(const ld2450_frame_t *frame,
                                 KalmanTrack tracks[LD2450_TARGET_COUNT])
{
  uint32_t i;
  uint32_t detected_mask = 0;

  /*
   * Temporary slot mapping only: LD2450 T1/T2/T3 maps to tracks[0/1/2].
   * Add data association later because radar slots are not persistent IDs.
   */
  for (i = 0; i < LD2450_TARGET_COUNT; i++) {
    if (frame->target[i].valid) {
      detected_mask |= 1u << i;

      if (!tracks[i].initialized) {
        Kalman_Init(&tracks[i],
                    (float)frame->target[i].x_mm,
                    (float)frame->target[i].y_mm,
                    KALMAN_DEFAULT_DT,
                    KALMAN_SIGMA_A_MM_S2,
                    KALMAN_R_X_MM2,
                    KALMAN_R_Y_MM2);
      } else {
        Kalman_Predict(&tracks[i]);
        Kalman_Update(&tracks[i],
                      (float)frame->target[i].x_mm,
                      (float)frame->target[i].y_mm);
      }

      tracks[i].active = 1;
      tracks[i].missed_frames = 0;
    } else if (tracks[i].initialized) {
      Kalman_Predict(&tracks[i]);
      if (tracks[i].missed_frames != 0xffu)
        tracks[i].missed_frames++;

      if (tracks[i].missed_frames > MAX_MISSED_FRAMES)
        Kalman_Reset(&tracks[i]);
    }
  }

  set_leds(detected_mask);
}

static void update_motion_outputs(
    const KalmanTrack tracks[LD2450_TARGET_COUNT],
    TargetFeatureState feature_state[LD2450_TARGET_COUNT],
    AI_Features ai_features[LD2450_TARGET_COUNT],
    TargetUIData ui_targets[LD2450_TARGET_COUNT])
{
  uint32_t i;

  for (i = 0; i < LD2450_TARGET_COUNT; i++) {
    if (!tracks[i].active) {
      Feature_Reset(&feature_state[i]);
      AI_Features_Reset(&ai_features[i]);
      TargetUI_Reset(&ui_targets[i]);
      continue;
    }

    Feature_Update(&feature_state[i], &tracks[i], &ai_features[i],
                   tracks[i].dt);
    TargetUI_Update(&ui_targets[i], &tracks[i]);
  }
}

static void print_kalman_tracks(
    const ld2450_frame_t *frame,
    const KalmanTrack tracks[LD2450_TARGET_COUNT],
    const AI_Features ai_features[LD2450_TARGET_COUNT],
    const TargetUIData ui_targets[LD2450_TARGET_COUNT])
{
  uint32_t i;

  uart_puts("\r\nLD2450 Targets:\r\n\r\n");
  for (i = 0; i < LD2450_TARGET_COUNT; i++) {
#if DEBUG_KALMAN
    uart_puts("T");
    uart_putchar('1' + i);
    if (frame->target[i].valid) {
      uart_puts(" RAW[x=");
      uart_print_signed_decimal((int32_t)frame->target[i].x_mm);
      uart_puts(",y=");
      uart_print_signed_decimal((int32_t)frame->target[i].y_mm);
      uart_puts("]\r\n");
    } else {
      uart_puts(" RAW[missing]\r\n");
    }

    if (tracks[i].active) {
      uart_puts("T");
      uart_putchar('1' + i);
      uart_puts(" KF [x=");
      uart_print_signed_fixed_3(tracks[i].x);
      uart_puts(",y=");
      uart_print_signed_fixed_3(tracks[i].y);
      uart_puts(",vx=");
      uart_print_signed_fixed_3(tracks[i].vx);
      uart_puts(",vy=");
      uart_print_signed_fixed_3(tracks[i].vy);
      uart_puts("]\r\n");
    }
#endif

#if DEBUG_AI_FEATURE
    if (tracks[i].active) {
      uart_puts("T");
      uart_putchar('1' + i);
      uart_puts(" AI[dx=");
      uart_print_signed_fixed_3(ai_features[i].delta_x);
      uart_puts(",dy=");
      uart_print_signed_fixed_3(ai_features[i].delta_y);
      uart_puts(",vx=");
      uart_print_signed_fixed_3(ai_features[i].vx);
      uart_puts(",vy=");
      uart_print_signed_fixed_3(ai_features[i].vy);
      uart_puts(",ax=");
      uart_print_signed_fixed_3(ai_features[i].ax);
      uart_puts(",ay=");
      uart_print_signed_fixed_3(ai_features[i].ay);
      uart_puts("]\r\n");
    }
#endif

    uart_puts("T");
    uart_putchar('1' + i);
    uart_puts(": ");

    if (!ui_targets[i].valid) {
      uart_puts("not detected\r\n");
      continue;
    }

    uart_puts("Distance=");
    uart_print_fixed_3(ui_targets[i].distance_m);
    uart_puts(" m | Velocity=");
    uart_print_fixed_3(ui_targets[i].speed_m_s);
    uart_puts(" m/s | Angle=");
    uart_print_signed_fixed_1(ui_targets[i].angle_deg);
    uart_puts(" deg");
    if (ui_targets[i].predicted)
      uart_puts(" | predicted");
    uart_puts("\r\n");
  }
}

int main()
{
    ld2450_frame_t frame;
    KalmanTrack tracks[LD2450_TARGET_COUNT];
    TargetFeatureState feature_state[LD2450_TARGET_COUNT];
    AI_Features ai_features[LD2450_TARGET_COUNT];
    TargetUIData ui_targets[LD2450_TARGET_COUNT];
    uint32_t i;

    for (i = 0; i < LD2450_TARGET_COUNT; i++) {
        Kalman_Reset(&tracks[i]);
        Feature_Reset(&feature_state[i]);
        AI_Features_Reset(&ai_features[i]);
        TargetUI_Reset(&ui_targets[i]);
    }

    uart_set_div((CLK_FREQ + 57600) / 115200 - 2);
    set_leds(0);

    /* Show the startup logos without blocking LD2450 processing afterwards. */
    oled_init();
    oled_show_ptit_logo();
    cdt_delay(2 * CLK_FREQ);
    oled_show_fee_logo();

    uart_puts("\r\nPicoRV32 LD2450 three-target reader\r\n");
    uart_puts("Configuring LD2450 multi-target mode... ");
    if (ld2450_configure_multi_target())
        uart_puts("OK\r\n");
    else
        uart_puts("no ACK; continuing in current radar mode\r\n");

    uart_puts("Waiting for 30-byte target frames...\r\n");
    while (1) {
        if (ld2450_read_frame(&frame)) {
            update_kalman_tracks(&frame, tracks);
            update_motion_outputs(tracks, feature_state, ai_features,
                                  ui_targets);
            print_kalman_tracks(&frame, tracks, ai_features, ui_targets);
        }
    }
    return 0;
}
