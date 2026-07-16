// ============================================================
// motor_control.v — 电机控制模块
// 功能：4 路独立 PWM + 循迹 PD 差速控制
// 电机映射：
//   m1: 左前 (U1 IN1=正转, IN2=反转)
//   m2: 右前 (U1 IN3=反转, IN4=正转)
//   m3: 左后 (U2 IN1=正转, IN2=反转)
//   m4: 右后 (U2 IN3=反转, IN4=正转)
// ============================================================

module motor_control (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        tick_10kHz,       // 控制量更新时基，PWM 本身由 50MHz 直接产生
    input  wire [3:0]  position,         // 来自 line_tracking，0=偏左，7=居中，15=偏右
    input  wire        line_lost,
    input  wire [1:0]  mode,             // 00=停止, 01=前进, 10=后退, 11=转向
    input  wire [3:0]  turn_dir,         // bit0=左前, bit1=右前, bit2=左后, bit3=右后
    input  wire [7:0]  base_speed,       // 基础速度 0~255
    // 输出到 L298N
    output reg         m1_in1,  m1_in2,  // 左前
    output reg         m2_in3,  m2_in4,  // 右前
    output reg         m3_in1,  m3_in2,  // 左后
    output reg         m4_in3,  m4_in4   // 右后
);

// ── 参数：后续调车主要改这里 ───────────────────────────────
localparam [12:0] PWM_PERIOD = 13'd5000; // 50MHz / 5000 = 10kHz PWM
localparam signed [7:0] KP = 8'sd16;     // 比例系数
localparam signed [7:0] KD = 8'sd4;      // 微分系数，先小后调
localparam [7:0] MIN_SPEED = 8'd40;      // 前进时最低占空比，避免单侧电机停转
localparam [7:0] SLEW_STEP = 8'd4;       // 每个 10kHz 控制周期最大变化量

// ── 10kHz PWM 计数器 ───────────────────────────────────────
reg [12:0] pwm_cnt;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        pwm_cnt <= 13'd0;
    else if (pwm_cnt >= (PWM_PERIOD - 1'b1))
        pwm_cnt <= 13'd0;
    else
        pwm_cnt <= pwm_cnt + 1'b1;
end

// 8bit 速度映射到 0~5000 的 PWM 比较阈值。
function [12:0] duty_to_count;
    input [7:0] duty;
    reg [13:0] scaled;
    begin
        scaled = ({6'd0, duty} << 4) + ({6'd0, duty} << 2); // duty * 20
        if (scaled >= {1'b0, PWM_PERIOD})
            duty_to_count = PWM_PERIOD;
        else
            duty_to_count = scaled[12:0];
    end
endfunction

function [7:0] abs_signed16;
    input signed [15:0] value;
    begin
        abs_signed16 = value[15] ? -value[7:0] : value[7:0];
    end
endfunction

function [7:0] reduce_speed;
    input [7:0] base;
    input [7:0] corr;
    reg [7:0] floor_speed;
    begin
        floor_speed = (base > MIN_SPEED) ? MIN_SPEED : base;
        if (base == 8'd0)
            reduce_speed = 8'd0;
        else if (corr >= (base - floor_speed))
            reduce_speed = floor_speed;
        else
            reduce_speed = base - corr;
    end
endfunction

function [7:0] slew_next;
    input [7:0] current;
    input [7:0] target;
    reg [7:0] delta;
    begin
        delta = 8'd0;
        if (target > current) begin
            delta = target - current;
            slew_next = (delta > SLEW_STEP) ? (current + SLEW_STEP) : target;
        end else if (target < current) begin
            delta = current - target;
            slew_next = (delta > SLEW_STEP) ? (current - SLEW_STEP) : target;
        end else begin
            slew_next = current;
        end
    end
endfunction

// ── PD 差速计算 ─────────────────────────────────────────────
reg signed [5:0] prev_error;
reg [7:0] left_target;
reg [7:0] right_target;
reg [7:0] left_out;
reg [7:0] right_out;

wire in_deadband = (position >= 4'd6) && (position <= 4'd8);
wire signed [5:0] error_now = (line_lost || in_deadband) ? 6'sd0 :
                              ($signed({1'b0, position}) - 6'sd7);
wire signed [6:0] error_delta = error_now - prev_error;
wire signed [15:0] control_term = (error_now * KP) + (error_delta * KD);
wire [7:0] correction = abs_signed16(control_term);

always @(*) begin
    left_target  = base_speed;
    right_target = base_speed;

    if (!line_lost && !in_deadband) begin
        if (control_term < 0) begin
            // 黑线偏左时降低左轮，帮助车头向左修正。
            left_target = reduce_speed(base_speed, correction);
        end else if (control_term > 0) begin
            // 黑线偏右时降低右轮，帮助车头向右修正。
            right_target = reduce_speed(base_speed, correction);
        end
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        prev_error <= 6'sd0;
        left_out   <= 8'd0;
        right_out  <= 8'd0;
    end else if (tick_10kHz) begin
        prev_error <= error_now;
        // 输出斜率限制：让占空比渐变，减少机械急加速造成的振荡。
        left_out   <= slew_next(left_out, left_target);
        right_out  <= slew_next(right_out, right_target);
    end
end

wire [12:0] left_pwm_count  = duty_to_count(left_out);
wire [12:0] right_pwm_count = duty_to_count(right_out);
wire [12:0] base_pwm_count  = duty_to_count(base_speed);

// ── 电机输出 ───────────────────────────────────────────────
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        {m1_in1, m1_in2} <= 2'b00;
        {m2_in3, m2_in4} <= 2'b00;
        {m3_in1, m3_in2} <= 2'b00;
        {m4_in3, m4_in4} <= 2'b00;
    end else case (mode)
        2'b00: begin                    // 停止
            {m1_in1, m1_in2} <= 2'b00;
            {m2_in3, m2_in4} <= 2'b00;
            {m3_in1, m3_in2} <= 2'b00;
            {m4_in3, m4_in4} <= 2'b00;
        end
        2'b01: begin                    // 前进（PD 差速循迹）
            m1_in1 <= (pwm_cnt < left_pwm_count);
            m1_in2 <= 1'b0;
            m2_in4 <= (pwm_cnt < right_pwm_count);
            m2_in3 <= 1'b0;
            m3_in1 <= (pwm_cnt < left_pwm_count);
            m3_in2 <= 1'b0;
            m4_in4 <= (pwm_cnt < right_pwm_count);
            m4_in3 <= 1'b0;
        end
        2'b10: begin                    // 后退
            m1_in1 <= 1'b0;
            m1_in2 <= (pwm_cnt < base_pwm_count);
            m2_in4 <= 1'b0;
            m2_in3 <= (pwm_cnt < base_pwm_count);
            m3_in1 <= 1'b0;
            m3_in2 <= (pwm_cnt < base_pwm_count);
            m4_in4 <= 1'b0;
            m4_in3 <= (pwm_cnt < base_pwm_count);
        end
        2'b11: begin                    // 转向
            m1_in1 <= turn_dir[0] ? (pwm_cnt < base_pwm_count) : 1'b0;
            m1_in2 <= 1'b0;
            m2_in4 <= turn_dir[1] ? (pwm_cnt < base_pwm_count) : 1'b0;
            m2_in3 <= 1'b0;
            m3_in1 <= turn_dir[2] ? (pwm_cnt < base_pwm_count) : 1'b0;
            m3_in2 <= 1'b0;
            m4_in4 <= turn_dir[3] ? (pwm_cnt < base_pwm_count) : 1'b0;
            m4_in3 <= 1'b0;
        end
        default: begin
            {m1_in1, m1_in2} <= 2'b00;
            {m2_in3, m2_in4} <= 2'b00;
            {m3_in1, m3_in2} <= 2'b00;
            {m4_in3, m4_in4} <= 2'b00;
        end
    endcase
end

endmodule
