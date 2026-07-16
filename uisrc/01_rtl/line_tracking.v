// ============================================================
// line_tracking.v — 6 路循迹模块
// 功能：读取 6 路 TCRT5000，加权质心法计算黑线位置
// 传感器极性：sensor=0 表示黑线，sensor=1 表示白底
// ============================================================

module line_tracking (
    input  wire        clk,
    input  wire        rst_n,
    input  wire [5:0]  sensor_in,        // {S6,S5,S4,S3,S2,S1}
    output reg  [3:0]  position,         // 0=偏左, 7=居中, 15=偏右
    output reg         line_lost         // 1=全部白底（脱线）
);

// ── 权重分配（S1~S6 从左到右）───────────────────────────────
// S1(最左)权重小, S6(最右)权重大
localparam [3:0] W1 = 4'd1;
localparam [3:0] W2 = 4'd3;
localparam [3:0] W3 = 4'd6;
localparam [3:0] W4 = 4'd10;
localparam [3:0] W5 = 4'd13;
localparam [3:0] W6 = 4'd15;

// ── 组合逻辑：加权质心 ─────────────────────────────────────
wire [5:0] active;                     // 取反：sensor=0(黑线) → active=1
assign active = ~sensor_in;            // {S6_n, S5_n, S4_n, S3_n, S2_n, S1_n}

// ── 输出 ────────────────────────────────────────────────────
always @(*) begin
    if (active == 6'b000000) begin
        position  = 4'd7;
        line_lost = 1'b1;
    end else begin
        line_lost = 1'b0;
        case (active)
            6'b000001: position = 4'd1;
            6'b000010: position = 4'd3;
            6'b000011: position = 4'd2;
            6'b000100: position = 4'd6;
            6'b000101: position = 4'd3;
            6'b000110: position = 4'd4;
            6'b000111: position = 4'd3;
            6'b001000: position = 4'd10;
            6'b001001: position = 4'd5;
            6'b001010: position = 4'd6;
            6'b001011: position = 4'd4;
            6'b001100: position = 4'd8;
            6'b001101: position = 4'd5;
            6'b001110: position = 4'd6;
            6'b001111: position = 4'd5;
            6'b010000: position = 4'd13;
            6'b010001: position = 4'd7;
            6'b010010: position = 4'd8;
            6'b010011: position = 4'd5;
            6'b010100: position = 4'd9;
            6'b010101: position = 4'd6;
            6'b010110: position = 4'd7;
            6'b010111: position = 4'd5;
            6'b011000: position = 4'd11;
            6'b011001: position = 4'd8;
            6'b011010: position = 4'd8;
            6'b011011: position = 4'd6;
            6'b011100: position = 4'd9;
            6'b011101: position = 4'd7;
            6'b011110: position = 4'd8;
            6'b011111: position = 4'd6;
            6'b100000: position = 4'd15;
            6'b100001: position = 4'd8;
            6'b100010: position = 4'd9;
            6'b100011: position = 4'd6;
            6'b100100: position = 4'd10;
            6'b100101: position = 4'd7;
            6'b100110: position = 4'd8;
            6'b100111: position = 4'd6;
            6'b101000: position = 4'd12;
            6'b101001: position = 4'd8;
            6'b101010: position = 4'd9;
            6'b101011: position = 4'd7;
            6'b101100: position = 4'd10;
            6'b101101: position = 4'd8;
            6'b101110: position = 4'd8;
            6'b101111: position = 4'd7;
            6'b110000: position = 4'd14;
            6'b110001: position = 4'd9;
            6'b110010: position = 4'd10;
            6'b110011: position = 4'd8;
            6'b110100: position = 4'd11;
            6'b110101: position = 4'd8;
            6'b110110: position = 4'd9;
            6'b110111: position = 4'd7;
            6'b111000: position = 4'd12;
            6'b111001: position = 4'd9;
            6'b111010: position = 4'd10;
            6'b111011: position = 4'd8;
            6'b111100: position = 4'd11;
            6'b111101: position = 4'd9;
            6'b111110: position = 4'd9;
            6'b111111: position = 4'd8;
            default:   position = 4'd7;
        endcase
    end
end

endmodule
