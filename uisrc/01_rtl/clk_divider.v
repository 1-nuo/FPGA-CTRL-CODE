// ============================================================
// clk_divider.v — 时钟分频模块（级联计数器版，无取模运算）
// 功能：从系统时钟产生各种时钟使能脉冲
// 优点：纯计数器+比较器，无硬件除法器，时序友好
// ============================================================

module clk_divider (
    input  wire        clk,
    input  wire        rst_n,
    output reg         tick_1MHz,     // 1MHz 使能（超声波用）
    output reg         tick_10kHz,    // 10kHz 使能（PWM 用）
    output reg         tick_1kHz,     // 1kHz 使能（LCD 用）
    output reg         tick_1Hz       // 1Hz 使能（状态刷新用）
);

// ============================================================
// 级联计数器方式：
//   50MHz → ÷50 → 1MHz → ÷100 → 10kHz → ÷10 → 1kHz → ÷1000 → 1Hz
// 全部使用计数+比较，无 % 运算器，时序友好
// ============================================================

// ── 1MHz: 50MHz ÷ 50 ───────────────────────────────────────
reg [5:0] cnt_1m;      // 0~49

wire tick_1m = (cnt_1m == 6'd49);

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        cnt_1m <= 6'd0;
    else if (tick_1m)
        cnt_1m <= 6'd0;
    else
        cnt_1m <= cnt_1m + 1'b1;
end

// ── 10kHz: 1MHz ÷ 100 ──────────────────────────────────────
reg [6:0] cnt_10k;     // 0~99

wire tick_10k = (cnt_10k == 7'd99 && tick_1m);

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        cnt_10k <= 7'd0;
    else if (tick_1m) begin
        if (tick_10k)
            cnt_10k <= 7'd0;
        else
            cnt_10k <= cnt_10k + 1'b1;
    end
end

// ── 1kHz: 10kHz ÷ 10 ───────────────────────────────────────
reg [3:0] cnt_1k;      // 0~9

wire tick_1k = (cnt_1k == 4'd9 && tick_10k);

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        cnt_1k <= 4'd0;
    else if (tick_10k) begin
        if (tick_1k)
            cnt_1k <= 4'd0;
        else
            cnt_1k <= cnt_1k + 1'b1;
    end
end

// ── 1Hz: 1kHz ÷ 1000 ──────────────────────────────────────
reg [9:0] cnt_1hz;     // 0~999

wire tick_1h = (cnt_1hz == 10'd999 && tick_1k);

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        cnt_1hz <= 10'd0;
    else if (tick_1k) begin
        if (tick_1h)
            cnt_1hz <= 10'd0;
        else
            cnt_1hz <= cnt_1hz + 1'b1;
    end
end

// ── 输出使能脉冲（打一拍，对齐寄存器输出）───────────────────
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        tick_1MHz  <= 1'b0;
        tick_10kHz <= 1'b0;
        tick_1kHz  <= 1'b0;
        tick_1Hz   <= 1'b0;
    end else begin
        tick_1MHz  <= tick_1m;
        tick_10kHz <= tick_10k;
        tick_1kHz  <= tick_1k;
        tick_1Hz   <= tick_1h;
    end
end

endmodule
