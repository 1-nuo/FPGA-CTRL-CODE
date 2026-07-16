// ============================================================
// ultrasonic_driver.v — HC-SR04 超声波测距模块
// 功能：发 10μs 触发脉冲，测量 ECHO 脉宽，换算成距离(cm)
// 距离 = pulse_us / 58  (1cm ≈ 58μs @ 声速 343m/s)
// ============================================================

module ultrasonic_driver (
    input  wire         clk,
    input  wire         rst_n,
    input  wire         tick_1MHz,          // 1MHz 使能 (1μs 精度)
    input  wire         trigger_start,      // 主控触发测量
    output reg          trig,               // 到 HC-SR04 TRIG
    input  wire         echo,               // 来自 HC-SR04 ECHO(已分压)
    output reg [15:0]   distance_cm,        // 测距结果(cm)
    output reg          measure_done        // 测量完成脉冲
);

// ── 状态机 ──────────────────────────────────────────────────
localparam IDLE      = 3'd0;
localparam TRIG_PULSE = 3'd1;
localparam WAIT_ECHO  = 3'd2;
localparam MEASURE    = 3'd3;
localparam DONE       = 3'd4;
localparam [7:0]  TRIG_US            = 8'd10;
localparam [15:0] WAIT_TIMEOUT_US    = 16'd30000;
localparam [15:0] MEASURE_TIMEOUT_US = 16'd50000;

reg [2:0] state, next_state;
reg [7:0] trig_cnt;           // 10μs 计数
reg [15:0] echo_cnt;          // ECHO 脉宽计数
reg [15:0] timeout_cnt;       // 超时保护 (>30ms 无回波视为超时)
wire echo_rising;              // ECHO 上升沿检测 (assign 驱动，必须 wire)
wire echo_falling;             // ECHO 下降沿检测
reg echo_r1;
reg echo_r2; // 同步 + 边沿检测

// ── 边沿检测 ────────────────────────────────────────────────
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        echo_r1 <= 1'b0;
        echo_r2 <= 1'b0;
    end else begin
        echo_r1 <= echo;
        echo_r2 <= echo_r1;
    end
end

assign echo_rising  = (echo_r1 & ~echo_r2);
assign echo_falling = (~echo_r1 & echo_r2);

// ── 状态跳转 ────────────────────────────────────────────────
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        state <= IDLE;
    else
        state <= next_state;
end

// ── 次态逻辑 ────────────────────────────────────────────────
always @(*) begin
    next_state = state;
    case (state)
        IDLE:       if (trigger_start)               next_state = TRIG_PULSE;
        TRIG_PULSE: if (tick_1MHz && trig_cnt >= (TRIG_US - 1'b1)) next_state = WAIT_ECHO;
        WAIT_ECHO:  if (echo_rising)                 next_state = MEASURE;
                    else if (timeout_cnt >= WAIT_TIMEOUT_US) next_state = DONE;   // 30ms 超时
        MEASURE:    if (echo_falling)                 next_state = DONE;
                    else if (timeout_cnt >= MEASURE_TIMEOUT_US) next_state = DONE;   // 50ms 超时
        DONE:       next_state = IDLE;
        default:    next_state = IDLE;
    endcase
end

// ── 时序逻辑 ────────────────────────────────────────────────
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        trig        <= 1'b0;
        trig_cnt    <= 8'd0;
        echo_cnt    <= 16'd0;
        timeout_cnt <= 16'd0;
        distance_cm <= 16'd0;
        measure_done <= 1'b0;
    end else begin
        measure_done <= 1'b0;  // 脉冲信号
        timeout_cnt  <= 16'd0; // 默认清零，只在 WAIT_ECHO/MEASURE 中计数

        case (state)
            IDLE: begin
                trig     <= 1'b0;
                trig_cnt <= 8'd0;
                echo_cnt <= 16'd0;
            end

            TRIG_PULSE: begin
                trig <= 1'b1;
                if (tick_1MHz) begin
                    if (trig_cnt >= (TRIG_US - 1'b1))
                        trig_cnt <= 8'd0;
                    else
                        trig_cnt <= trig_cnt + 1'b1;
                end
            end

            WAIT_ECHO: begin
                trig <= 1'b0;
                // timeout_cnt 只在 1MHz tick 上计数，阈值单位才是真正的微秒。
                if (echo_rising)
                    timeout_cnt <= 16'd0;
                else if (tick_1MHz && timeout_cnt < WAIT_TIMEOUT_US)
                    timeout_cnt <= timeout_cnt + 1'b1;
            end

            MEASURE: begin
                if (tick_1MHz && echo_cnt < MEASURE_TIMEOUT_US)
                    echo_cnt <= echo_cnt + 1'b1;
                if (tick_1MHz && timeout_cnt < MEASURE_TIMEOUT_US)
                    timeout_cnt <= timeout_cnt + 1'b1;
            end

            DONE: begin
                distance_cm <= echo_cnt / 16'd58;   // 换算为 cm
                measure_done <= 1'b1;
            end
        endcase
    end
end

endmodule
