`timescale 1ns/1ps

module ld2450_uart_tb;
    localparam integer CLKS_PER_BIT = 105;

    reg clk = 0;
    reg reset_n = 0;
    reg sel = 0;
    reg [1:0] addr = 0;
    reg [3:0] wstrb = 0;
    reg [31:0] wdata = 0;
    wire [31:0] rdata;
    wire ready;
    reg radar_rx = 1;
    wire radar_tx;
    integer i;
    integer errors = 0;
    reg [31:0] value;
    reg [7:0] expected [0:29];
    reg [7:0] tx_expected = 8'ha5;

    always #5 clk = ~clk;

    ld2450_uart #(
        .CLK_FREQ(27000000),
        .BAUD(256000),
        .FIFO_ADDR_WIDTH(6)
    ) dut (
        .clk(clk),
        .reset_n(reset_n),
        .sel(sel),
        .addr(addr),
        .wstrb(wstrb),
        .wdata(wdata),
        .rdata(rdata),
        .ready(ready),
        .radar_rx(radar_rx),
        .radar_tx(radar_tx)
    );

    task send_uart_byte;
        input [7:0] data;
        integer bit_index;
        begin
            radar_rx = 0;
            repeat (CLKS_PER_BIT) @(posedge clk);
            for (bit_index = 0; bit_index < 8; bit_index = bit_index + 1) begin
                radar_rx = data[bit_index];
                repeat (CLKS_PER_BIT) @(posedge clk);
            end
            radar_rx = 1;
            repeat (CLKS_PER_BIT) @(posedge clk);
        end
    endtask

    task mmio_read;
        input [1:0] register_address;
        output [31:0] data;
        begin
            @(negedge clk);
            sel = 1;
            addr = register_address;
            wstrb = 0;
            #1 data = rdata;
            @(posedge clk);
            @(negedge clk);
            sel = 0;
        end
    endtask

    task mmio_write_byte;
        input [1:0] register_address;
        input [7:0] data;
        begin
            @(negedge clk);
            sel = 1;
            addr = register_address;
            wstrb = 4'b0001;
            wdata = data;
            while (!ready) @(negedge clk);
            @(posedge clk);
            @(negedge clk);
            sel = 0;
            wstrb = 0;
        end
    endtask

    initial begin
        expected[0] = 8'haa;
        expected[1] = 8'hff;
        expected[2] = 8'h03;
        expected[3] = 8'h00;
        expected[4] = 8'h0e;
        expected[5] = 8'h03;
        expected[6] = 8'hb1;
        expected[7] = 8'h86;
        expected[8] = 8'h10;
        expected[9] = 8'h00;
        expected[10] = 8'h40;
        expected[11] = 8'h01;
        for (i = 12; i < 28; i = i + 1)
            expected[i] = 0;
        expected[28] = 8'h55;
        expected[29] = 8'hcc;

        repeat (5) @(posedge clk);
        reset_n = 1;
        repeat (5) @(posedge clk);

        for (i = 0; i < 30; i = i + 1)
            send_uart_byte(expected[i]);

        repeat (4) @(posedge clk);
        mmio_read(2'd0, value);
        if (value[14:8] != 30) begin
            $display("FAIL: FIFO count is %0d, expected 30", value[14:8]);
            errors = errors + 1;
        end

        for (i = 0; i < 30; i = i + 1) begin
            mmio_read(2'd1, value);
            if (value[7:0] != expected[i]) begin
                $display("FAIL: byte %0d is %02x, expected %02x",
                         i, value[7:0], expected[i]);
                errors = errors + 1;
            end
        end

        mmio_read(2'd1, value);
        if (value != 32'hffff_ffff) begin
            $display("FAIL: empty FIFO returned %08x", value);
            errors = errors + 1;
        end

        mmio_write_byte(2'd2, tx_expected);
        repeat (CLKS_PER_BIT/2) @(posedge clk);
        if (radar_tx !== 1'b0) begin
            $display("FAIL: TX start bit");
            errors = errors + 1;
        end
        for (i = 0; i < 8; i = i + 1) begin
            repeat (CLKS_PER_BIT) @(posedge clk);
            if (radar_tx !== tx_expected[i]) begin
                $display("FAIL: TX data bit %0d", i);
                errors = errors + 1;
            end
        end
        repeat (CLKS_PER_BIT) @(posedge clk);
        if (radar_tx !== 1'b1) begin
            $display("FAIL: TX stop bit");
            errors = errors + 1;
        end

        if (errors == 0)
            $display("PASS: LD2450 UART RX FIFO and TX");
        else
            $display("FAIL: %0d error(s)", errors);

        $finish;
    end
endmodule
