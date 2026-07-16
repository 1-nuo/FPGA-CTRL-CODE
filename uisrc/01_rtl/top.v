// ============================================================
// top.v — 顶层模块
// 功能：例化所有子模块 + main_controller 主状态机协调
// 芯片：安路 EF2L45BG256B (正式) / Cyclone V 5CSEMA5F31C6 (DE1-SoC 测试)
// ============================================================

module top (
    // 系统
    input  wire        clk,          // 板载晶振输入
    input  wire        rst_n,        // 复位（低有效）

    // 循迹传感器（6 路输入）
    input  wire [5:0]  track_sensor, // {S6,S5,S4,S3,S2,S1}

    // 电机驱动（U1 L298N — 前桥）
    output wire        m1_in1,       // 左前 正转/PWM
    output wire        m1_in2,       // 左前 反转/PWM
    output wire        m2_in3,       // 右前 反转/PWM
    output wire        m2_in4,       // 右前 正转/PWM

    // 电机驱动（U2 L298N — 后桥）
    output wire        m3_in1,       // 左后 正转/PWM
    output wire        m3_in2,       // 左后 反转/PWM
    output wire        m4_in3,       // 右后 反转/PWM
    output wire        m4_in4,       // 右后 正转/PWM

    // LCD1602
    output wire        lcd_rs,
    output wire        lcd_en,
    output wire [3:0]  lcd_data,     // {D7,D6,D5,D4}

    // HC-SR04 超声波
    output wire        ultra_trig,
    input  wire        ultra_echo,

    // DHT11
    inout  wire        dht_data,

    // 蜂鸣器
    output wire        buzzer_out,

    // L298N 通道使能 (经 74HCT245D 升压到 5V)
    output wire        en_u1_ena,     // U1 前桥 A 通道使能
    output wire        en_u1_enb,     // U1 前桥 B 通道使能
    output wire        en_u2_ena,     // U2 后桥 A 通道使能
    output wire        en_u2_enb,     // U2 后桥 B 通道使能

    // 调试: DE1-SoC 板载 LED
    // LEDR0 (PIN_V16): 1Hz 闪烁 = 代码在运行
    // LEDR1 (PIN_W16): 亮 = LCD 初始化完成
    output wire        led_test,
    output wire        led_dbg2
);

// ═══════════════════════════════════════════════════════════
//  时钟与复位
// ═══════════════════════════════════════════════════════════
wire        clk_sys;       // PLL 输出系统时钟
wire        pll_locked;    // PLL 锁定信号

// PLL (DE1-SoC 测试用 bypass, 正式部署时替换为 TD PLL IP)
pll u_pll (
    .refclk   (clk),
    .reset    (1'b0),
    .clk0_out (clk_sys),
    .extlock  (pll_locked)
);

wire sys_rst_n_async = rst_n & pll_locked;

// 复位采用“异步拉低、同步释放”，避免 PLL lock 或按键复位释放时卡到时钟边沿。
reg [1:0] rst_sync;
always @(posedge clk_sys or negedge sys_rst_n_async) begin
    if (!sys_rst_n_async)
        rst_sync <= 2'b00;
    else
        rst_sync <= {rst_sync[0], 1'b1};
end

wire sys_rst_n = rst_sync[1];   // 系统内部统一使用同步释放后的复位

// ═══════════════════════════════════════════════════════════
//  时钟分频使能
// ═══════════════════════════════════════════════════════════
wire tick_1MHz;
wire tick_10kHz;
wire tick_1kHz;
wire tick_1Hz;

clk_divider u_clk_divider (
    .clk        (clk_sys),
    .rst_n      (sys_rst_n),
    .tick_1MHz  (tick_1MHz),
    .tick_10kHz (tick_10kHz),
    .tick_1kHz  (tick_1kHz),
    .tick_1Hz   (tick_1Hz)
);

// ═══════════════════════════════════════════════════════════
//  循迹模块
// ═══════════════════════════════════════════════════════════
wire [3:0] track_position;
wire       line_lost;
wire [5:0] track_sensor_sync;

// 循迹传感器来自板外，先同步到 clk_sys，降低亚稳态导致位置跳变的概率。
reg [5:0] track_sensor_r1;
reg [5:0] track_sensor_r2;

always @(posedge clk_sys or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        track_sensor_r1 <= 6'b111111;  // 复位时按“全白底”处理
        track_sensor_r2 <= 6'b111111;
    end else begin
        track_sensor_r1 <= track_sensor;
        track_sensor_r2 <= track_sensor_r1;
    end
end

assign track_sensor_sync = track_sensor_r2;

line_tracking u_line_tracking (
    .clk       (clk_sys),
    .rst_n     (sys_rst_n),
    .sensor_in (track_sensor_sync),
    .position  (track_position),
    .line_lost (line_lost)
);

// ═══════════════════════════════════════════════════════════
//  超声波测距
// ═══════════════════════════════════════════════════════════
wire [15:0] ultra_distance;
wire        ultra_done;
wire        ultra_start;

ultrasonic_driver u_ultrasonic (
    .clk           (clk_sys),
    .rst_n         (sys_rst_n),
    .tick_1MHz     (tick_1MHz),
    .trigger_start (ultra_start),         // ← main_controller 驱动
    .trig          (ultra_trig),
    .echo          (ultra_echo),
    .distance_cm   (ultra_distance),
    .measure_done  (ultra_done)
);

// ═══════════════════════════════════════════════════════════
//  温湿度采集
// ═══════════════════════════════════════════════════════════
wire [15:0] dht_temp;
wire [15:0] dht_hum;
wire        dht_done;
wire        dht_start;

dht11_driver u_dht11 (
    .clk         (clk_sys),
    .rst_n       (sys_rst_n),
    .tick_1MHz   (tick_1MHz),
    .start_read  (dht_start),            // ← main_controller 驱动
    .dht_data    (dht_data),
    .temperature (dht_temp),
    .humidity    (dht_hum),
    .read_done   (dht_done)
);

// ═══════════════════════════════════════════════════════════
//  LCD 驱动
// ═══════════════════════════════════════════════════════════
wire        lcd_busy;
wire        lcd_init_done;
wire        lcd_wr;
wire [7:0]  lcd_wr_data;
wire        lcd_wr_is_data;

lcd_driver u_lcd (
    .clk          (clk_sys),
    .rst_n        (sys_rst_n),
    .tick_1kHz    (tick_1kHz),
    .cmd_valid    (lcd_wr),              // ← main_controller 驱动
    .cmd_data     (lcd_wr_data),
    .cmd_is_data  (lcd_wr_is_data),
    .cmd_busy     (lcd_busy),
    .init_done    (lcd_init_done),
    .lcd_rs       (lcd_rs),
    .lcd_en       (lcd_en),
    .lcd_data     (lcd_data)
);

// ═══════════════════════════════════════════════════════════
//  主状态机 — 协调所有外设
// ═══════════════════════════════════════════════════════════
wire [3:0]  ctrl_motor_position;
wire        ctrl_motor_line_lost;
wire [1:0]  ctrl_motor_mode;
wire [7:0]  ctrl_motor_speed;
wire [15:0] ctrl_buzz_distance;

main_controller u_main (
    .clk               (clk_sys),
    .rst_n             (sys_rst_n),
    .tick_1kHz         (tick_1kHz),
    .tick_1Hz          (tick_1Hz),

    // 传感器状态输入
    .track_position    (track_position),
    .line_lost         (line_lost),
    .ultra_distance    (ultra_distance),
    .ultra_done        (ultra_done),
    .dht_temperature   (dht_temp),
    .dht_humidity      (dht_hum),
    .dht_done          (dht_done),
    .lcd_busy          (lcd_busy),

    // 控制输出
    .ultra_start       (ultra_start),
    .dht_start         (dht_start),
    .lcd_wr            (lcd_wr),
    .lcd_wr_data       (lcd_wr_data),
    .lcd_wr_is_data    (lcd_wr_is_data),

    // 电机控制
    .motor_position    (ctrl_motor_position),
    .motor_line_lost   (ctrl_motor_line_lost),
    .motor_mode        (ctrl_motor_mode),
    .motor_speed       (ctrl_motor_speed),

    // 蜂鸣器
    .buzz_distance     (ctrl_buzz_distance)
);

// ═══════════════════════════════════════════════════════════
//  电机控制
// ═══════════════════════════════════════════════════════════
motor_control u_motor (
    .clk        (clk_sys),
    .rst_n      (sys_rst_n),
    .tick_10kHz (tick_10kHz),
    .position   (ctrl_motor_position),   // ← main_controller 转发
    .line_lost  (ctrl_motor_line_lost),
    .mode       (ctrl_motor_mode),
    .turn_dir   (4'b0000),
    .base_speed (ctrl_motor_speed),
    .m1_in1     (m1_in1),
    .m1_in2     (m1_in2),
    .m2_in3     (m2_in3),
    .m2_in4     (m2_in4),
    .m3_in1     (m3_in1),
    .m3_in2     (m3_in2),
    .m4_in3     (m4_in3),
    .m4_in4     (m4_in4)
);

// ═══════════════════════════════════════════════════════════
//  蜂鸣器
// ═══════════════════════════════════════════════════════════
buzzer_ctrl u_buzzer (
    .clk           (clk_sys),
    .rst_n         (sys_rst_n),
    .obstacle_dist (ctrl_buzz_distance),  // ← main_controller 转发
    .enable        (1'b1),
    .buzzer_out    (buzzer_out)
);

// ═══════════════════════════════════════════════════════════
//  L298N 通道使能 (常高, 经 74HCT245D → 5V)
// ═══════════════════════════════════════════════════════════
assign en_u1_ena = 1'b1;
assign en_u1_enb = 1'b1;
assign en_u2_ena = 1'b1;
assign en_u2_enb = 1'b1;

// ═══════════════════════════════════════════════════════════
//  调试 LED — 1Hz 闪烁 (验证时钟和代码在运行)
//  接到 DE1-SoC 板载 LEDR0 (PIN_V16)
// ═══════════════════════════════════════════════════════════
reg led_blink;
always @(posedge clk_sys or negedge sys_rst_n) begin
    if (!sys_rst_n)
        led_blink <= 1'b0;
    else if (tick_1Hz)
        led_blink <= ~led_blink;
end
assign led_test  = led_blink;
assign led_dbg2  = lcd_init_done;

endmodule
