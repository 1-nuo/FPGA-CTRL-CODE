// ============================================================
// buzzer_ctrl.v — 蜂鸣器控制模块
// 功能：根据障碍距离控制蜂鸣器通断
//   < 10cm: 连续响
//   < 20cm: 间歇响 (4Hz)
//   ≥ 20cm: 不响
// ============================================================

module buzzer_ctrl (
    input  wire         clk,
    input  wire         rst_n,
    input  wire [15:0]  obstacle_dist,      // 来自超声波模块 (cm)
    input  wire         enable,              // 蜂鸣器使能
    output reg          buzzer_out           // 到 GPIO_16
);

// ── 4Hz 闪烁计数器 ─────────────────────────────────────────
// 50MHz / 4Hz / 2 = 6_250_000
reg [22:0] blinky_cnt;
reg        blinky_out;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        blinky_cnt <= 23'd0;
        blinky_out <= 1'b0;
    end else begin
        if (blinky_cnt >= 23'd6_250_000 - 1) begin
            blinky_cnt <= 23'd0;
            blinky_out <= ~blinky_out;
        end else begin
            blinky_cnt <= blinky_cnt + 1'b1;
        end
    end
end

// ── 蜂鸣器输出逻辑 ─────────────────────────────────────────
always @(*) begin
    if (!enable) begin
        buzzer_out = 1'b0;
    end else if (obstacle_dist < 16'd10) begin
        buzzer_out = 1'b1;                   // < 10cm 连续响
    end else if (obstacle_dist < 16'd20) begin
        buzzer_out = blinky_out;             // < 20cm 间歇响
    end else begin
        buzzer_out = 1'b0;                   // ≥ 20cm 不响
    end
end

endmodule
