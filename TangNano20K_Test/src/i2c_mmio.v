`default_nettype none

module i2c_mmio (
    input  wire        clk,
    input  wire        resetn,

    input  wire        valid,
    input  wire        wen,
    input  wire [31:0] addr,
    input  wire [31:0] wdata,
    output reg  [31:0] rdata,
    output wire        ready,

    output wire        scl_oe,
    output wire        sda_oe,
    input  wire        sda_in
);

    assign ready = valid;

    // Register map:
    // 0x80000040 CTRL    bit0=start, bit1=rw
    // 0x80000044 STATUS  bit0=busy, bit1=ack_err
    // 0x80000048 ADDR    [6:0] i2c address
    // 0x8000004C LEN     [3:0] tx_len, [7:4] rx_len
    // 0x80000050 TX0     byte0..byte3
    // 0x80000054 TX1     byte4..byte7
    // 0x80000058 TX2     byte8..byte11
    // 0x8000005C TX3     byte12..byte15
    // 0x80000060 RX0
    // 0x80000064 RX1
    // 0x80000068 RX2
    // 0x8000006C RX3

    reg        start_pulse;
    reg        rw_reg;
    reg [6:0]  i2c_addr;
    reg [3:0]  tx_len;
    reg [3:0]  rx_len;
    reg [127:0] tx_buf;

    wire [127:0] rx_buf;
    wire busy;
    wire ack_err;

    always @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            start_pulse <= 1'b0;
            rw_reg      <= 1'b0;
            i2c_addr    <= 7'h3C;
            tx_len      <= 4'd0;
            rx_len      <= 4'd0;
            tx_buf      <= 128'd0;
        end else begin
            start_pulse <= 1'b0;

            if (valid && wen) begin
                case (addr)
                    32'h80000040: begin
                        start_pulse <= wdata[0];
                        rw_reg      <= wdata[1];
                    end

                    32'h80000048: begin
                        i2c_addr <= wdata[6:0];
                    end

                    32'h8000004C: begin
                        tx_len <= wdata[3:0];
                        rx_len <= wdata[7:4];
                    end

                    32'h80000050: begin
                        tx_buf[127:96] <= wdata;
                    end

                    32'h80000054: begin
                        tx_buf[95:64] <= wdata;
                    end

                    32'h80000058: begin
                        tx_buf[63:32] <= wdata;
                    end

                    32'h8000005C: begin
                        tx_buf[31:0] <= wdata;
                    end
                endcase
            end
        end
    end

    always @(*) begin
        case (addr)
            32'h80000044: rdata = {30'd0, ack_err, busy};
            32'h80000048: rdata = {25'd0, i2c_addr};
            32'h8000004C: rdata = {24'd0, rx_len, tx_len};
            32'h80000050: rdata = tx_buf[127:96];
            32'h80000054: rdata = tx_buf[95:64];
            32'h80000058: rdata = tx_buf[63:32];
            32'h8000005C: rdata = tx_buf[31:0];
            32'h80000060: rdata = rx_buf[127:96];
            32'h80000064: rdata = rx_buf[95:64];
            32'h80000068: rdata = rx_buf[63:32];
            32'h8000006C: rdata = rx_buf[31:0];
            default:      rdata = 32'd0;
        endcase
    end

    i2c_master #(
        .CLK_DIV(67)
    ) i2c_master_inst (
        .clk(clk),
        .resetn(resetn),

        .start(start_pulse),
        .rw(rw_reg),
        .addr(i2c_addr),
        .tx_len(tx_len),
        .rx_len(rx_len),
        .tx_buf(tx_buf),
        .rx_buf(rx_buf),

        .busy(busy),
        .ack_err(ack_err),

        .scl_oe(scl_oe),
        .sda_oe(sda_oe),
        .sda_in(sda_in)
    );

endmodule

`default_nettype wire