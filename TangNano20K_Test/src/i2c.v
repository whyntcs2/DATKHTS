`default_nettype none
// ============================================================
//  i2c_master.v — I2C Master, multi-byte TX/RX, hỗ trợ
//  combined write/read với repeated-start
//
//  Hỗ trợ:
//    1) Write only:
//       [START][ADDR+W][TX...][STOP]
//
//    2) Read only (tx_len = 0):
//       [START][ADDR+R][RX...][STOP]
//
//    3) Register read (rw=1, tx_len>0):
//       [START][ADDR+W][TX...][REPEATED START][ADDR+R][RX...][STOP]
//
//  Clock: 27 MHz → I2C ~100 kHz (CLK_DIV=67)
// ============================================================
module i2c_master #(
    parameter CLK_DIV = 67
)(
    input  wire         clk,
    input  wire         resetn,

    input  wire         start,
    input  wire         rw,           // 0=write, 1=read / combined read
    input  wire [6:0]   addr,
    input  wire [3:0]   tx_len,       // số byte TX (0..15)
    input  wire [3:0]   rx_len,       // số byte RX (0..15)

    input  wire [127:0] tx_buf,       // byte0=bits[127:120]
    output reg  [127:0] rx_buf,       // byte0=bits[127:120]

    output reg          busy,
    output reg          ack_err,

    output wire         scl_oe,       // 1 -> kéo SCL xuống 0
    output wire         sda_oe,       // 1 -> kéo SDA xuống 0
    input  wire         sda_in
);

// ── Helper: lấy byte idx từ flat bus ─────────────────────────
function [7:0] get_byte;
    input [127:0] bus;
    input [3:0]   idx;
    begin
        get_byte = bus[127 - (idx << 3) -: 8];
    end
endfunction

// ── Clock divider: 4 phase / bit ─────────────────────────────
reg [6:0] clk_cnt;
reg [1:0] phase;
reg       tick;

always @(posedge clk or negedge resetn) begin
    if (!resetn) begin
        clk_cnt <= 7'd0;
        phase   <= 2'd0;
        tick    <= 1'b0;
    end else if (busy) begin
        tick <= 1'b0;
        if (clk_cnt == CLK_DIV - 1) begin
            clk_cnt <= 7'd0;
            phase   <= phase + 2'd1;
            tick    <= 1'b1;
        end else begin
            clk_cnt <= clk_cnt + 7'd1;
        end
    end else begin
        clk_cnt <= 7'd0;
        phase   <= 2'd0;
        tick    <= 1'b0;
    end
end

// ── FSM states ───────────────────────────────────────────────
localparam S_IDLE       = 4'd0,
           S_START      = 4'd1,
           S_ADDR       = 4'd2,
           S_ADDR_ACK   = 4'd3,
           S_WRITE      = 4'd4,
           S_WRITE_ACK  = 4'd5,
           S_RESTART    = 4'd6,
           S_READ       = 4'd7,
           S_READ_ACK1  = 4'd8,
           S_READ_ACK2  = 4'd9,
           S_STOP       = 4'd10,
           S_READ_NACK  = 4'd11;
// ── Registers ────────────────────────────────────────────────
reg [3:0]   state;
reg [2:0]   bit_idx;
reg [3:0]   byte_idx;

reg         rw_r;
reg [6:0]   addr_r;
reg [3:0]   tx_len_r;
reg [3:0]   rx_len_r;
reg [127:0] tx_buf_r;

// phase hiện tại đang gửi địa chỉ theo hướng nào
reg         addr_phase_rw;

// cờ cho combined transaction:
// rw=1 và tx_len>0 => write subaddr trước rồi repeated-start read
reg         combined_read;

reg scl_oe_r, sda_oe_r;
assign scl_oe = scl_oe_r;
assign sda_oe = sda_oe_r;

// Byte đang gửi
wire [7:0] cur_tx_byte = get_byte(tx_buf_r, byte_idx);

// vị trí bit trong rx_buf
wire [7:0] rx_bit_pos = 8'd127 - {byte_idx, 3'b000} - (3'd7 - bit_idx);

always @(posedge clk or negedge resetn) begin
    if (!resetn) begin
        state         <= S_IDLE;
        scl_oe_r      <= 1'b0;
        sda_oe_r      <= 1'b0;
        busy          <= 1'b0;
        ack_err       <= 1'b0;
        bit_idx       <= 3'd7;
        byte_idx      <= 4'd0;
        rw_r          <= 1'b0;
        addr_r        <= 7'd0;
        tx_len_r      <= 4'd0;
        rx_len_r      <= 4'd0;
        tx_buf_r      <= 128'd0;
        rx_buf        <= 128'd0;
        addr_phase_rw <= 1'b0;
        combined_read <= 1'b0;
    end else begin
        case (state)

        // =====================================================
        S_IDLE: begin
            scl_oe_r <= 1'b0;
            sda_oe_r <= 1'b0;
            busy     <= 1'b0;

            if (start) begin
                busy          <= 1'b1;
                ack_err       <= 1'b0;
                rw_r          <= rw;
                addr_r        <= addr;
                tx_len_r      <= tx_len;
                rx_len_r      <= rx_len;
                tx_buf_r      <= tx_buf;
                rx_buf        <= 128'd0;
                bit_idx       <= 3'd7;
                byte_idx      <= 4'd0;
                combined_read <= rw && (tx_len != 0);

                // Nếu combined read thì pha địa chỉ đầu là write
                // nếu không thì dùng rw trực tiếp
                addr_phase_rw <= (rw && (tx_len != 0)) ? 1'b0 : rw;

                state <= S_START;
            end
        end

        // =====================================================
        // START: khi SCL đang thả cao, kéo SDA xuống
        S_START: begin
            if (tick && phase == 2'd2)
                sda_oe_r <= 1'b1;

            if (tick && phase == 2'd3) begin
                scl_oe_r <= 1'b1;   // kéo SCL xuống để bắt đầu shift bit
                bit_idx  <= 3'd7;
                state    <= S_ADDR;
            end
        end

        // =====================================================
        // Gửi 8 bit địa chỉ + rw
        // bit7..1 = addr[6:0], bit0 = addr_phase_rw
        S_ADDR: begin
            if (tick && phase == 2'd0)
                sda_oe_r <= (bit_idx > 0) ? ~addr_r[bit_idx-1] : ~addr_phase_rw;

            if (tick && phase == 2'd1)
                scl_oe_r <= 1'b0;   // nhả SCL lên cao

            if (tick && phase == 2'd3) begin
                scl_oe_r <= 1'b1;   // kéo SCL xuống
                if (bit_idx == 0) begin
                    sda_oe_r <= 1'b0; // release SDA cho ACK bit
                    state    <= S_ADDR_ACK;
                end else begin
                    bit_idx <= bit_idx - 3'd1;
                end
            end
        end

        // =====================================================
        // ACK sau address
        S_ADDR_ACK: begin
            if (tick && phase == 2'd1)
                scl_oe_r <= 1'b0;

            if (tick && phase == 2'd2)
                ack_err <= sda_in;

            if (tick && phase == 2'd3) begin
                scl_oe_r <= 1'b1;
                bit_idx  <= 3'd7;
                byte_idx <= 4'd0;

                if (sda_in) begin
                    state <= S_STOP;
                end else begin
                    if (addr_phase_rw) begin
                        // address+R xong thì đi vào đọc
                        state <= S_READ;
                    end else begin
                        // address+W xong:
                        // - nếu còn tx data thì ghi tiếp
                        // - nếu combined read và tx_len=0 thì restart luôn
                        // - nếu write only mà tx_len=0 thì stop
                        if (tx_len_r != 0)
                            state <= S_WRITE;
                        else if (combined_read) begin
                            addr_phase_rw <= 1'b1;
                            state         <= S_RESTART;
                        end else
                            state <= S_STOP;
                    end
                end
            end
        end

        // =====================================================
        // Ghi từng byte TX
        S_WRITE: begin
            if (tick && phase == 2'd0)
                sda_oe_r <= ~cur_tx_byte[bit_idx];

            if (tick && phase == 2'd1)
                scl_oe_r <= 1'b0;

            if (tick && phase == 2'd3) begin
                scl_oe_r <= 1'b1;
                if (bit_idx == 0) begin
                    sda_oe_r <= 1'b0; // release ACK bit
                    state    <= S_WRITE_ACK;
                end else begin
                    bit_idx <= bit_idx - 3'd1;
                end
            end
        end

        // =====================================================
        // ACK sau mỗi byte write
        S_WRITE_ACK: begin
            if (tick && phase == 2'd1)
                scl_oe_r <= 1'b0;

            if (tick && phase == 2'd2)
                ack_err <= sda_in;

            if (tick && phase == 2'd3) begin
                scl_oe_r <= 1'b1;
                bit_idx  <= 3'd7;

                if (sda_in) begin
                    state <= S_STOP;
                end else if (byte_idx == tx_len_r - 1) begin
                    // đã ghi xong byte cuối
                    if (combined_read) begin
                        addr_phase_rw <= 1'b1;
                        state         <= S_RESTART;
                    end else begin
                        state <= S_STOP;
                    end
                end else begin
                    byte_idx <= byte_idx + 4'd1;
                    state    <= S_WRITE;
                end
            end
        end

        // =====================================================
        // REPEATED START
        // Bắt đầu từ trạng thái bus đang SCL=0.
        // Thả SDA, thả SCL lên cao, rồi kéo SDA xuống lại.
        S_RESTART: begin
            // Đảm bảo bus đang ở trạng thái SDA=1, SCL=1 trước khi tạo repeated START
            if (tick && phase == 2'd0) begin
                sda_oe_r <= 1'b0;   // nhả SDA
                scl_oe_r <= 1'b0;   // nhả SCL
            end

            if (tick && phase == 2'd2)
                sda_oe_r <= 1'b1;   // SDA high -> low khi SCL high

            if (tick && phase == 2'd3) begin
                scl_oe_r <= 1'b1;   // kéo SCL xuống để bắt đầu shift địa chỉ mới
                bit_idx  <= 3'd7;
                state    <= S_ADDR;
            end
        end

        S_READ: begin
            if (tick && phase == 2'd0)
                sda_oe_r <= 1'b0;

            if (tick && phase == 2'd1)
                scl_oe_r <= 1'b0;

            if (tick && phase == 2'd2)
                rx_buf[rx_bit_pos] <= sda_in;

            if (tick && phase == 2'd3) begin
                scl_oe_r <= 1'b1;
                if (bit_idx == 0) begin
                    bit_idx <= 3'd7;
                    state   <= S_READ_NACK;
                end else
                    bit_idx <= bit_idx - 3'd1;
            end
        end

        // setup ACK/NACK khi SCL đang thấp
        S_READ_ACK1: begin
            if (tick && phase == 2'd0) begin
                if (byte_idx < rx_len_r - 1)
                    sda_oe_r <= 1'b1;   // ACK
                else
                    sda_oe_r <= 1'b0;   // NACK
            end

            if (tick && phase == 2'd1)
                scl_oe_r <= 1'b0;       // cho SCL lên cao để slave thấy ACK/NACK

            if (tick && phase == 2'd3) begin
                scl_oe_r <= 1'b1;       // kéo SCL xuống lại
                state    <= S_READ_ACK2;
            end
        end

        // giữ ACK/NACK đủ lâu, rồi mới chuyển state
        S_READ_ACK2: begin
            if (tick && phase == 2'd0) begin
                if (byte_idx == rx_len_r - 1) begin
                    state <= S_STOP;
                end else begin
                    sda_oe_r <= 1'b0;   // nhả SDA sau khi ACK xong
                    byte_idx <= byte_idx + 4'd1;
                    state    <= S_READ;
                end
            end
        end
        S_READ_NACK: begin
            if (tick && phase == 2'd0)
                sda_oe_r <= (byte_idx < rx_len_r - 1) ? 1'b1 : 1'b0;

            if (tick && phase == 2'd1)
                scl_oe_r <= 1'b0;

            if (tick && phase == 2'd3) begin
                scl_oe_r <= 1'b1;
                sda_oe_r <= 1'b0;
                if (byte_idx == rx_len_r - 1)
                    state <= S_STOP;
                else begin
                    byte_idx <= byte_idx + 4'd1;
                    state <= S_READ;
                end
            end
        end


        // =====================================================
        // STOP: SDA low -> high khi SCL high
        S_STOP: begin
            if (tick && phase == 2'd0)
                sda_oe_r <= 1'b1;

            if (tick && phase == 2'd1)
                scl_oe_r <= 1'b0;

            if (tick && phase == 2'd2)
                sda_oe_r <= 1'b0;

            if (tick && phase == 2'd3) begin
                state <= S_IDLE;
                busy  <= 1'b0;
            end
        end

        default: begin
            state <= S_IDLE;
        end

        endcase
    end
end

endmodule
`default_nettype wire