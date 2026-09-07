`timescale 1ns/1ps

/*
 * End-to-end simulation for the Tang Nano 20K PicoRV32 LD2450 build.
 *
 * This test runs the real firmware from src/mem_init[0-3].ini, models the
 * LD2450 command/ACK exchange, sends a three-target frame, decodes the SoC
 * USB UART output, and checks the valid-target LED mask.
 */

/* Simulation replacement for the Gowin PLL.  The project input and system
 * clocks are both 27 MHz in this build, so a direct connection is accurate
 * for functional simulation. */
module Gowin_rPLL #(
    parameter integer IDIV_SEL = 0,
    parameter integer FBDIV_SEL = 0,
    parameter integer ODIV_SEL = 32
) (
    output wire clkout,
    input  wire clkin
);
    assign clkout = clkin;
endmodule

module picorv32_ld2450_soc_tb;
    localparam integer RADAR_CLKS_PER_BIT = 105;
    localparam integer USB_CLKS_PER_BIT = 234;

    reg clk_in = 1'b0;
    reg reset_button = 1'b1;
    reg button2 = 1'b1;
    reg radar_rx = 1'b1;
    reg sd_miso = 1'b1;
    reg uart_rx = 1'b1;

    wire ws2812b_din;
    wire radar_tx;
    wire sd_cs;
    wire sd_mosi;
    wire sd_clk;
    wire uart_tx;
    wire oled_scl;
    tri  oled_sda;
    wire [5:0] leds;

    integer sensor_errors = 0;
    integer i;
    reg [7:0] radar_byte;
    reg [7:0] usb_byte;
    reg [7:0] enable_command [0:13];
    reg [7:0] multi_command [0:11];
    reg [7:0] end_command [0:11];
    reg [7:0] target_frame [0:29];

    string usb_line;
    reg configuration_ok_seen = 1'b0;
    reg target1_seen = 1'b0;
    reg target2_seen = 1'b0;
    reg target3_seen = 1'b0;

    /* 27 MHz input clock. */
    always #18.5185 clk_in = ~clk_in;

    top dut (
        .clk_in(clk_in),
        .reset_button(reset_button),
        .button2(button2),
        .ws2812b_din(ws2812b_din),
        .radar_rx(radar_rx),
        .radar_tx(radar_tx),
        .sd_miso(sd_miso),
        .sd_cs(sd_cs),
        .sd_mosi(sd_mosi),
        .sd_clk(sd_clk),
        .uart_rx(uart_rx),
        .uart_tx(uart_tx),
        .oled_scl(oled_scl),
        .oled_sda(oled_sda),
        .leds(leds)
    );

    task receive_radar_byte;
        output [7:0] data;
        integer bit_index;
        begin
            @(negedge radar_tx);
            repeat (RADAR_CLKS_PER_BIT + RADAR_CLKS_PER_BIT/2)
                @(posedge clk_in);
            for (bit_index = 0; bit_index < 8; bit_index = bit_index + 1) begin
                data[bit_index] = radar_tx;
                repeat (RADAR_CLKS_PER_BIT) @(posedge clk_in);
            end
            if (radar_tx !== 1'b1) begin
                $display("FAIL: LD2450 command UART stop bit");
                sensor_errors = sensor_errors + 1;
            end
        end
    endtask

    task send_radar_byte;
        input [7:0] data;
        integer bit_index;
        begin
            radar_rx = 1'b0;
            repeat (RADAR_CLKS_PER_BIT) @(posedge clk_in);
            for (bit_index = 0; bit_index < 8; bit_index = bit_index + 1) begin
                radar_rx = data[bit_index];
                repeat (RADAR_CLKS_PER_BIT) @(posedge clk_in);
            end
            radar_rx = 1'b1;
            repeat (RADAR_CLKS_PER_BIT) @(posedge clk_in);
        end
    endtask

    task send_ack;
        input [7:0] command;
        begin
            send_radar_byte(8'hfd);
            send_radar_byte(8'hfc);
            send_radar_byte(8'hfb);
            send_radar_byte(8'hfa);
            send_radar_byte(8'h04);
            send_radar_byte(8'h00);
            send_radar_byte(command);
            send_radar_byte(8'h01);
            send_radar_byte(8'h00);
            send_radar_byte(8'h00);
            send_radar_byte(8'h04);
            send_radar_byte(8'h03);
            send_radar_byte(8'h02);
            send_radar_byte(8'h01);
        end
    endtask

    task receive_usb_byte;
        output [7:0] data;
        integer bit_index;
        begin
            @(negedge uart_tx);
            repeat (USB_CLKS_PER_BIT + USB_CLKS_PER_BIT/2)
                @(posedge clk_in);
            for (bit_index = 0; bit_index < 8; bit_index = bit_index + 1) begin
                data[bit_index] = uart_tx;
                repeat (USB_CLKS_PER_BIT) @(posedge clk_in);
            end
            if (uart_tx !== 1'b1)
                $display("FAIL: USB UART stop bit");
        end
    endtask

    task process_usb_byte;
        input [7:0] data;
        begin
            if (data == 8'h0d) begin
                /* Ignore CR and process the line on LF. */
            end else if (data == 8'h0a) begin
                if (usb_line != "")
                    $display("USB: %0s", usb_line);

                if (usb_line == "Configuring LD2450 multi-target mode... OK")
                    configuration_ok_seen = 1'b1;
                if (usb_line == "  T1: distance=2.228 m, velocity=0.000 m/s")
                    target1_seen = 1'b1;
                if (usb_line == "  T2: not detected")
                    target2_seen = 1'b1;
                if (usb_line == "  T3: distance=4.177 m, velocity=0.000 m/s")
                    target3_seen = 1'b1;

                usb_line = "";
            end else begin
                usb_line = {usb_line, data};
            end
        end
    endtask

    /* Decode every character written by the firmware to the board USB UART. */
    initial begin
        usb_line = "";
        forever begin
            receive_usb_byte(usb_byte);
            process_usb_byte(usb_byte);
        end
    end

    /* Reset the complete SoC. */
    initial begin
        repeat (8) @(posedge clk_in);
        reset_button = 1'b0;
    end

    /* LD2450 model: validate three commands, ACK each one, then send data. */
    initial begin
        enable_command[0]  = 8'hfd; enable_command[1]  = 8'hfc;
        enable_command[2]  = 8'hfb; enable_command[3]  = 8'hfa;
        enable_command[4]  = 8'h04; enable_command[5]  = 8'h00;
        enable_command[6]  = 8'hff; enable_command[7]  = 8'h00;
        enable_command[8]  = 8'h01; enable_command[9]  = 8'h00;
        enable_command[10] = 8'h04; enable_command[11] = 8'h03;
        enable_command[12] = 8'h02; enable_command[13] = 8'h01;

        multi_command[0]  = 8'hfd; multi_command[1]  = 8'hfc;
        multi_command[2]  = 8'hfb; multi_command[3]  = 8'hfa;
        multi_command[4]  = 8'h02; multi_command[5]  = 8'h00;
        multi_command[6]  = 8'h90; multi_command[7]  = 8'h00;
        multi_command[8]  = 8'h04; multi_command[9]  = 8'h03;
        multi_command[10] = 8'h02; multi_command[11] = 8'h01;

        end_command[0]  = 8'hfd; end_command[1]  = 8'hfc;
        end_command[2]  = 8'hfb; end_command[3]  = 8'hfa;
        end_command[4]  = 8'h02; end_command[5]  = 8'h00;
        end_command[6]  = 8'hfe; end_command[7]  = 8'h00;
        end_command[8]  = 8'h04; end_command[9]  = 8'h03;
        end_command[10] = 8'h02; end_command[11] = 8'h01;

        /* Header. */
        target_frame[0] = 8'haa; target_frame[1] = 8'hff;
        target_frame[2] = 8'h03; target_frame[3] = 8'h00;
        /* T1: X=+350, Y=+2200, speed=-12, resolution=320. */
        target_frame[4] = 8'h5e; target_frame[5] = 8'h81;
        target_frame[6] = 8'h98; target_frame[7] = 8'h88;
        target_frame[8] = 8'h0c; target_frame[9] = 8'h00;
        target_frame[10] = 8'h40; target_frame[11] = 8'h01;
        /* T2 absent. */
        for (i = 12; i < 20; i = i + 1)
            target_frame[i] = 8'h00;
        /* T3: X=-800, Y=+4100, speed=+5, resolution=450. */
        target_frame[20] = 8'h20; target_frame[21] = 8'h03;
        target_frame[22] = 8'h04; target_frame[23] = 8'h90;
        target_frame[24] = 8'h05; target_frame[25] = 8'h80;
        target_frame[26] = 8'hc2; target_frame[27] = 8'h01;
        target_frame[28] = 8'h55; target_frame[29] = 8'hcc;

        wait (dut.reset_n === 1'b1);

        for (i = 0; i < 14; i = i + 1) begin
            receive_radar_byte(radar_byte);
            if (radar_byte !== enable_command[i]) begin
                $display("FAIL: enable command byte %0d = %02x, expected %02x",
                         i, radar_byte, enable_command[i]);
                sensor_errors = sensor_errors + 1;
            end
        end
        send_ack(8'hff);

        for (i = 0; i < 12; i = i + 1) begin
            receive_radar_byte(radar_byte);
            if (radar_byte !== multi_command[i]) begin
                $display("FAIL: multi-target command byte %0d = %02x, expected %02x",
                         i, radar_byte, multi_command[i]);
                sensor_errors = sensor_errors + 1;
            end
        end
        send_ack(8'h90);

        for (i = 0; i < 12; i = i + 1) begin
            receive_radar_byte(radar_byte);
            if (radar_byte !== end_command[i]) begin
                $display("FAIL: end command byte %0d = %02x, expected %02x",
                         i, radar_byte, end_command[i]);
                sensor_errors = sensor_errors + 1;
            end
        end
        send_ack(8'hfe);

        /* Allow firmware to consume the ACK and clear the RX FIFO. */
        repeat (2000) @(posedge clk_in);
        for (i = 0; i < 30; i = i + 1)
            send_radar_byte(target_frame[i]);
    end

    /* End-to-end pass condition. */
    initial begin
        wait (configuration_ok_seen && target1_seen && target2_seen && target3_seen);
        repeat (500) @(posedge clk_in);

        if (sensor_errors != 0) begin
            $display("FAIL: %0d LD2450 model error(s)", sensor_errors);
            $fatal(1);
        end
        if (dut.soc_leds.leds_data_o[5:0] !== 6'b000101) begin
            $display("FAIL: logical LED mask = %06b, expected 000101",
                     dut.soc_leds.leds_data_o[5:0]);
            $fatal(1);
        end
        if (leds !== 6'b111010) begin
            $display("FAIL: active-low board LEDs = %06b, expected 111010", leds);
            $fatal(1);
        end

        $display("PASS: full PicoRV32 firmware + LD2450 three-target integration");
        $finish;
    end

    /* About 148 ms of simulated 27 MHz time. */
    initial begin
        repeat (4000000) @(posedge clk_in);
        $display("FAIL: SoC simulation timeout");
        $display("  config=%0d T1=%0d T2=%0d T3=%0d sensor_errors=%0d",
                 configuration_ok_seen, target1_seen, target2_seen,
                 target3_seen, sensor_errors);
        $fatal(1);
    end
endmodule
