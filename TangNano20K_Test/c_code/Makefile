# Copyright 2024 Grug Huhler.  License SPDX BSD-2-Clause.
#
# This makefile builds the PicoRV32 firmware for the Tang Nano 20K.
#
# Use this Makefile to build the software BEFORE using the Gowin
# IDE to build the Verilog bitfile.
#
# To build, run "make clean" followed by "make" or "make 20k".
#
# SRAM_ADDR_WIDTH sets the width of the SRAM's address in the Verilog.
# This implicitly sets the total bytes of SRAM to 4*(1 << SRAM_ADDR_WIDTH)
# which, in turn, sets the starting address of the stack pointer in
# startup.s to the top of the SRAM.
#
# The practical SRAM_ADDR_WIDTH values are
#   11 - giving 8192 bytes of SRAM (using 4 block SRAMS).
#   12 - giving 16384 bytes of SRAM (using 8 block SRAMS).
#   13 - giving 32768 bytes of SRAM  (using 16 block SRAMS).
#   14 - giving 65536 bytes of SRAM  (using 32 block SRAMS).
#
# Values less than 11 will work but are pointless on these FPGAs since
# 4 block SRAMs will still be used.
#
# Program conv_to_init creates the Verilog files in ../src.  See the
# comments in conv_to_init.c.


# Change memory size here
SRAM_ADDR_WIDTH = 13

# CLK_FREQ tells both software and Verilog the fixed system clock speed.
# This Tang Nano 20K project is configured and verified for 27 MHz.
# Generated file ../src/sys_parameters.v lets the Verilog build use the
# frequency value.  It is passed to software by a -DCLK_FREQ=XXXX during
# the build.

# Change system clock speed here (MHz).
CLK_FREQ = 27000000

BOARD = BOARD_20K
.DEFAULT_GOAL := 20k

# Bộ công cụ RISC-V
PREFIX = riscv64-unknown-elf
# Compile và link code C
CC = $(PREFIX)-gcc
# chuyển đổi định dạng file
OBJCOPY = $(PREFIX)-objcopy
# Xem nội dung file
OBJDUMP = $(PREFIX)-objdump
# biên dịch code Assembly .S/.s → .o
AS = $(PREFIX)-as

# kiểm tra phiên bản GCC đang sử dụng
GCCVERSION = $(shell $(PREFIX)-gcc -dumpversion)

# Chương trình được biên dịch cho kiến trúc RV32I
COMMONFLAGS = -march=rv32i2p0 -mabi=ilp32
CFLAGS = -mno-save-restore $(COMMONFLAGS) -nostartfiles -nostdlib -static \
	-O1 -D$(BOARD) 
# Cấu hình biên dịch C cho RISC-V: dùng RV32I, không dùng startup/thư viện chuẩn,
# link tĩnh, tối ưu mức O1 và định nghĩa macro theo board
# Tạo 1 macro để biết đang sử dụng board nào để build
CPPFLAGS = -D$(BOARD)
# cấu hình compile Assembly
ASFLAGS = --defsym SRAM_ADDR_WIDTH=$(SRAM_ADDR_WIDTH) $(COMMONFLAGS)

ifeq ($(GCCVERSION), 10.2.0)
# This is a hack for Ubuntu 22.04 whose riscv gcc appears to have
# problems that are resolved in laer gcc version.
# Workaround cho GCC 10.2.0 trên Ubuntu, các phiên bản khác dùng libgcc mặc định
LIBS = /usr/lib/gcc/riscv64-unknown-elf/10.2.0/rv32i/ilp32/libgcc.a
else
LIBS = -lgcc
endif

# khai báo danh sách các file source .c cần biên dịch và gộp lại
COMMON_CSRCS = main.c countdown_timer.c uart.c leds.c sd_card_cgpt.c i2c.c oled.c

NANO20K_CSRCS = ws2812b.c ld2450.c kalman.c motion_features.c
ALL_CSRCS = $(COMMON_CSRCS) $(NANO20K_CSRCS)

%.o: %.c
	$(CC) -c $(CFLAGS) -DCLK_FREQ=$(CLK_FREQ) $< -o $@ 
# biên dịch file .c -> .o

%.d: %.c
	$(CC) -MM $(CPPFLAGS) $< > $@.$$$$; \
	sed 's,\($*\)\.o[ :]*,\1.o $@ : ,g' < $@.$$$$ > $@; \
	rm -f $@.$$$$
# biên dịch file.c -> .d để theo dõi thay đổi của các header

.PHONY: 20k clean # Khai báo các target lệnh
20k: prog_20k.hex # Build firmware

ifneq ($(MAKECMDGOALS),clean)
include $(COMMON_CSRCS:.c=.d) $(NANO20K_CSRCS:.c=.d)
endif
# Nạp dependency khi không chạy clean

conv_to_init: conv_to_init.c
	gcc -o conv_to_init conv_to_init.c

startup.o: startup.S
	$(CC) $(CFLAGS) -DSRAM_ADDR_WIDTH=$(SRAM_ADDR_WIDTH) -c startup.S

prog_20k.elf: startup.o $(COMMON_CSRCS:.c=.o) $(NANO20K_CSRCS:.c=.o)
	$(CC) $(CFLAGS) -Tlink_cmd.ld -o prog_20k.elf startup.o \
              $(COMMON_CSRCS:.c=.o) $(NANO20K_CSRCS:.c=.o) $(LIBS)
# Tạo file.elf

prog_20k.bin: prog_20k.elf conv_to_init 
	$(OBJCOPY) prog_20k.elf -O binary prog_20k.bin
	rm -f ../src/mem_init[0-3].ini ../src/sys_parameters.v
	./conv_to_init $(BOARD) $(CLK_FREQ) $(SRAM_ADDR_WIDTH) prog_20k.bin
# Chuyển ELF sang BIN và tạo dữ liệu khởi tạo bộ nhớ cho FPGA

prog_20k.hex: prog_20k.bin conv_to_init
	od -v -Ax -t x4 prog_20k.bin > prog_20k.hex
# Chuyển firmware BIN sang định dạng HEX

clean:
	rm -f prog_20k.elf prog_20k.hex prog_20k.bin startup.o \
	      $(ALL_CSRCS:.c=.d) $(ALL_CSRCS:.c=.o) \
              conv_to_init
