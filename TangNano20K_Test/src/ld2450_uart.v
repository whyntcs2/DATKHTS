/* Dedicated UART for the Hi-Link LD2450 radar.
 *
 * Register map (offset from the peripheral base address):
 *   0x0 STATUS  [0] RX data available, [1] RX overflow,
 *               [2] TX busy, [14:8] RX FIFO count
 *   0x4 RXDATA  next received byte in [7:0], 0xffffffff when empty
 *   0x8 TXDATA  write a byte in [7:0] (the bus waits while TX is busy)
 *   0xc CONTROL write [0] to clear RX FIFO, [1] to clear overflow
 */

module ld2450_uart #(
    parameter integer CLK_FREQ = 27000000,
    parameter integer BAUD = 256000,
    parameter integer FIFO_ADDR_WIDTH = 6
) (
    input  wire        clk,
    input  wire        reset_n,
    input  wire        sel,
    input  wire [1:0]  addr,
    input  wire [3:0]  wstrb,
    input  wire [31:0] wdata,
    output reg  [31:0] rdata,
    output wire        ready,
    input  wire        radar_rx,
    output wire        radar_tx
);

    localparam integer CLKS_PER_BIT = (CLK_FREQ + BAUD/2) / BAUD;
    localparam integer FIFO_DEPTH = 1 << FIFO_ADDR_WIDTH;
    localparam integer FIFO_COUNT_WIDTH = FIFO_ADDR_WIDTH + 1;

    wire read_access = sel && !(|wstrb);
    wire write_access = sel && (|wstrb);
    wire tx_request = write_access && wstrb[0] && (addr == 2'd2);
    wire clear_fifo = write_access && wstrb[0] &&
                      (addr == 2'd3) && wdata[0];
    wire clear_overflow = write_access && wstrb[0] &&
                          (addr == 2'd3) && wdata[1];

    reg tx_busy;
    assign ready = sel && (!tx_request || !tx_busy);

    reg [7:0] rx_fifo [0:FIFO_DEPTH-1];
    reg [FIFO_ADDR_WIDTH-1:0] rx_write_ptr;
    reg [FIFO_ADDR_WIDTH-1:0] rx_read_ptr;
    reg [FIFO_COUNT_WIDTH-1:0] rx_count;
    reg rx_overflow;

    wire rx_not_empty = (rx_count != 0);
    wire rx_full = (rx_count == FIFO_DEPTH);
    wire rx_pop = read_access && (addr == 2'd1) && rx_not_empty;

    always @(*) begin
        case (addr)
            2'd0: rdata = {17'd0, rx_count, 5'd0,
                           tx_busy, rx_overflow, rx_not_empty};
            2'd1: rdata = rx_not_empty ? {24'd0, rx_fifo[rx_read_ptr]} :
                                        32'hffff_ffff;
            2'd3: rdata = CLKS_PER_BIT;
            default: rdata = 32'd0;
        endcase
    end

    reg radar_rx_meta;
    reg radar_rx_sync;
    reg [1:0] rx_state;
    reg [31:0] rx_clock_count;
    reg [2:0] rx_bit_index;
    reg [7:0] rx_shift;
    reg [7:0] rx_byte;
    reg rx_byte_valid;

    localparam [1:0] RX_IDLE  = 2'd0;
    localparam [1:0] RX_START = 2'd1;
    localparam [1:0] RX_DATA  = 2'd2;
    localparam [1:0] RX_STOP  = 2'd3;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            radar_rx_meta <= 1'b1;
            radar_rx_sync <= 1'b1;
            rx_state <= RX_IDLE;
            rx_clock_count <= 0;
            rx_bit_index <= 0;
            rx_shift <= 0;
            rx_byte <= 0;
            rx_byte_valid <= 1'b0;
        end else begin
            radar_rx_meta <= radar_rx;
            radar_rx_sync <= radar_rx_meta;
            rx_byte_valid <= 1'b0;

            case (rx_state)
                RX_IDLE: begin
                    if (!radar_rx_sync) begin
                        rx_state <= RX_START;
                        rx_clock_count <= CLKS_PER_BIT/2;
                    end
                end

                RX_START: begin
                    if (rx_clock_count != 0) begin
                        rx_clock_count <= rx_clock_count - 1;
                    end else if (!radar_rx_sync) begin
                        rx_state <= RX_DATA;
                        rx_bit_index <= 0;
                        rx_clock_count <= CLKS_PER_BIT - 1;
                    end else begin
                        rx_state <= RX_IDLE;
                    end
                end

                RX_DATA: begin
                    if (rx_clock_count != 0) begin
                        rx_clock_count <= rx_clock_count - 1;
                    end else begin
                        rx_shift[rx_bit_index] <= radar_rx_sync;
                        rx_clock_count <= CLKS_PER_BIT - 1;
                        if (rx_bit_index == 3'd7) begin
                            rx_state <= RX_STOP;
                        end else begin
                            rx_bit_index <= rx_bit_index + 1'b1;
                        end
                    end
                end

                RX_STOP: begin
                    if (rx_clock_count != 0) begin
                        rx_clock_count <= rx_clock_count - 1;
                    end else begin
                        if (radar_rx_sync) begin
                            rx_byte <= rx_shift;
                            rx_byte_valid <= 1'b1;
                        end
                        rx_state <= RX_IDLE;
                    end
                end
            endcase
        end
    end

    wire rx_push = rx_byte_valid && (!rx_full || rx_pop);

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            rx_write_ptr <= 0;
            rx_read_ptr <= 0;
            rx_count <= 0;
            rx_overflow <= 1'b0;
        end else if (clear_fifo) begin
            rx_write_ptr <= 0;
            rx_read_ptr <= 0;
            rx_count <= 0;
            rx_overflow <= 1'b0;
        end else begin
            if (clear_overflow)
                rx_overflow <= 1'b0;

            if (rx_push) begin
                rx_fifo[rx_write_ptr] <= rx_byte;
                rx_write_ptr <= rx_write_ptr + 1'b1;
            end else if (rx_byte_valid && rx_full) begin
                rx_overflow <= 1'b1;
            end

            if (rx_pop)
                rx_read_ptr <= rx_read_ptr + 1'b1;

            case ({rx_push, rx_pop})
                2'b10: rx_count <= rx_count + 1'b1;
                2'b01: rx_count <= rx_count - 1'b1;
                default: rx_count <= rx_count;
            endcase
        end
    end

    reg [9:0] tx_shift;
    reg [3:0] tx_bits_left;
    reg [31:0] tx_clock_count;
    wire tx_accept = tx_request && !tx_busy;

    assign radar_tx = tx_busy ? tx_shift[0] : 1'b1;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            tx_shift <= 10'h3ff;
            tx_bits_left <= 0;
            tx_clock_count <= 0;
            tx_busy <= 1'b0;
        end else if (tx_accept) begin
            tx_shift <= {1'b1, wdata[7:0], 1'b0};
            tx_bits_left <= 4'd10;
            tx_clock_count <= CLKS_PER_BIT - 1;
            tx_busy <= 1'b1;
        end else if (tx_busy) begin
            if (tx_clock_count != 0) begin
                tx_clock_count <= tx_clock_count - 1;
            end else if (tx_bits_left == 1) begin
                tx_shift <= 10'h3ff;
                tx_bits_left <= 0;
                tx_busy <= 1'b0;
            end else begin
                tx_shift <= {1'b1, tx_shift[9:1]};
                tx_bits_left <= tx_bits_left - 1'b1;
                tx_clock_count <= CLKS_PER_BIT - 1;
            end
        end
    end

endmodule
