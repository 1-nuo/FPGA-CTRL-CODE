// ============================================================
// main_controller.v — 主状态机
// 功能：协调循迹/超声波/电机/温湿度/蜂鸣器/LCD 的工作节奏
// LCD 显示格式:
//   第1行: T:+XXC H:XX%    (温度 湿度)
//   第2行: 2024111622 FYN  (学号+姓名首字母)
// ============================================================

module main_controller (
    input  wire         clk,
    input  wire         rst_n,
    input  wire         tick_1kHz,           // 1kHz 使能
    input  wire         tick_1Hz,            // 1Hz 使能

    // ── 传感器状态 ──────────────────────────────────────
    input  wire [3:0]   track_position,       // 循迹位置 (0-15)
    input  wire         line_lost,            // 脱线标志
    input  wire [15:0]  ultra_distance,       // 超声波距离(cm)
    input  wire         ultra_done,           // 超声波测量完成
    input  wire [15:0]  dht_temperature,      // 温度 (16位)
    input  wire [15:0]  dht_humidity,         // 湿度 (16位)
    input  wire         dht_done,             // DHT11 读取完成
    input  wire         lcd_busy,             // LCD 忙

    // ── 控制输出 ──────────────────────────────────────
    output reg          ultra_start,          // 触发超声波测距
    output reg          dht_start,            // 触发 DHT11 读取
    output reg          lcd_wr,               // LCD 写使能
    output reg [7:0]    lcd_wr_data,          // LCD 数据
    output reg          lcd_wr_is_data,       // 0=指令, 1=数据

    // ── 电机控制 ──────────────────────────────────────
    output reg [3:0]    motor_position,       // 转发循迹位置
    output reg          motor_line_lost,       // 转发脱线标志
    output reg [1:0]    motor_mode,           // 电机模式
    output reg [7:0]    motor_speed,          // 速度设置

    // ── 蜂鸣器 ─────────────────────────────────────────
    output reg [15:0]   buzz_distance         // 转发障碍距离
);

// ── 状态定义 ────────────────────────────────────────────
localparam S_INIT       = 4'd0;
localparam S_IDLE       = 4'd1;
localparam S_ULTRA_TRIG = 4'd2;
localparam S_ULTRA_WAIT = 4'd3;
localparam S_DHT_TRIG   = 4'd4;
localparam S_DHT_WAIT   = 4'd5;
localparam S_LCD_WRITE  = 4'd6;    // 写 LCD (数据/指令)

reg [3:0]  state;

// ── 定时器 ────────────────────────────────────────────
reg [15:0] ultra_timer;
reg [15:0] dht_timer;
reg        ultra_pending;
reg        dht_pending;
localparam ULTRA_PERIOD_MS = 16'd200;
localparam DHT_PERIOD_S    = 16'd1;

// ── LCD 调度 ─────────────────────────────────────────
reg        lcd_req;                  // 1Hz 刷新请求
reg        lcd_active;               // LCD 分段刷新正在进行
reg [5:0]  lcd_step;                 // 写进度 0~33
reg        lcd_wait_ack;             // wait until lcd_driver samples lcd_wr and raises busy
localparam LCD_TOTAL = 8'd34;        // 共 34 次写操作

// ═══════════════════════════════════════════════════════
//  LCD 显示缓冲区 (32 字节, 16字/行 × 2行)
//  在 tick_1Hz 时从传感器值计算填充
// ═══════════════════════════════════════════════════════
reg [7:0]  lcd_buf [0:31];           // 显示缓冲区

// ── 十进制数码提取 (组合逻辑, 仅温湿度需要) ────────────
wire [7:0]  temp_int  = dht_temperature[15:8];
wire [7:0]  hum_int   = dht_humidity[15:8];

function [7:0] bin_to_2digit_bcd;
    input [7:0] value;
    reg [7:0] v;
    reg [7:0] base;
    reg [3:0] tens;
    reg [3:0] ones;
    begin
        v = (value > 8'd99) ? 8'd99 : value;
        if (v >= 8'd90) begin
            tens = 4'd9;
            base = 8'd90;
        end else if (v >= 8'd80) begin
            tens = 4'd8;
            base = 8'd80;
        end else if (v >= 8'd70) begin
            tens = 4'd7;
            base = 8'd70;
        end else if (v >= 8'd60) begin
            tens = 4'd6;
            base = 8'd60;
        end else if (v >= 8'd50) begin
            tens = 4'd5;
            base = 8'd50;
        end else if (v >= 8'd40) begin
            tens = 4'd4;
            base = 8'd40;
        end else if (v >= 8'd30) begin
            tens = 4'd3;
            base = 8'd30;
        end else if (v >= 8'd20) begin
            tens = 4'd2;
            base = 8'd20;
        end else if (v >= 8'd10) begin
            tens = 4'd1;
            base = 8'd10;
        end else begin
            tens = 4'd0;
            base = 8'd0;
        end
        ones = v[3:0] - base[3:0];
        bin_to_2digit_bcd = {tens, ones};
    end
endfunction

wire [7:0] temp_bcd = bin_to_2digit_bcd(temp_int);
wire [7:0] hum_bcd  = bin_to_2digit_bcd(hum_int);
wire [3:0] t_t = temp_bcd[7:4];
wire [3:0] t_o = temp_bcd[3:0];
wire [3:0] h_t = hum_bcd[7:4];
wire [3:0] h_o = hum_bcd[3:0];

// ═══════════════════════════════════════════════════════
//  主状态机
// ═══════════════════════════════════════════════════════
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state           <= S_INIT;
        ultra_start     <= 1'b0;
        dht_start       <= 1'b0;
        lcd_wr          <= 1'b0;
        lcd_wr_data     <= 8'd0;
        lcd_wr_is_data  <= 1'b0;
        motor_position  <= 4'd7;
        motor_line_lost <= 1'b0;
        motor_mode      <= 2'b00;
        motor_speed     <= 8'd128;
        buzz_distance   <= 16'd0;
        ultra_timer     <= 16'd0;
        dht_timer       <= 16'd0;
        ultra_pending   <= 1'b0;
        dht_pending     <= 1'b0;
        lcd_req         <= 1'b0;
        lcd_active      <= 1'b0;
        lcd_step        <= 6'd0;
        lcd_wait_ack    <= 1'b0;
    end else begin
        // ── 连续转发 ─────────────────────────────────
        motor_position  <= track_position;
        motor_line_lost <= line_lost;
        motor_mode      <= (line_lost) ? 2'b00 : 2'b01;
        motor_speed     <= (line_lost) ? 8'd64 : 8'd128;
        buzz_distance   <= ultra_distance;

        // ── 脉冲信号默认清零 ─────────────────────────
        ultra_start <= 1'b0;
        dht_start   <= 1'b0;
        lcd_wr      <= 1'b0;

        // ── 定时器 ──────────────────────────────────
        if (tick_1kHz) begin
            if (ultra_timer > 0)
                ultra_timer <= ultra_timer - 1'b1;
            else
                ultra_pending <= 1'b1;
        end

        if (tick_1Hz) begin
            if (dht_timer > 0)
                dht_timer <= dht_timer - 1'b1;
            else
                dht_pending <= 1'b1;
            // ── 计算 LCD 显示数据 ──────────────────
            // 第1行: T:+XXC H:XX%    (温度 湿度)
            lcd_buf[ 0] <= 8'h54;                    // 'T'
            lcd_buf[ 1] <= 8'h3A;                    // ':'
            lcd_buf[ 2] <= 8'h2B;                    // '+'
            lcd_buf[ 3] <= 8'h30 + t_t[3:0];         // 十位
            lcd_buf[ 4] <= 8'h30 + t_o[3:0];         // 个位
            lcd_buf[ 5] <= 8'h43;                    // 'C'
            lcd_buf[ 6] <= 8'h20;                    // ' '
            lcd_buf[ 7] <= 8'h48;                    // 'H'
            lcd_buf[ 8] <= 8'h3A;                    // ':'
            lcd_buf[ 9] <= 8'h30 + h_t[3:0];         // 十位
            lcd_buf[10] <= 8'h30 + h_o[3:0];         // 个位
            lcd_buf[11] <= 8'h25;                    // '%'
            lcd_buf[12] <= 8'h20;                    // ' '
            lcd_buf[13] <= 8'h20;                    // ' '
            lcd_buf[14] <= 8'h20;                    // ' '
            lcd_buf[15] <= 8'h20;                    // ' '

            // 第2行: 2024111635WHZ      (学号+姓名)
            lcd_buf[16] <= 8'h32;                    // '2'
            lcd_buf[17] <= 8'h30;                    // '0'
            lcd_buf[18] <= 8'h32;                    // '2'
            lcd_buf[19] <= 8'h34;                    // '4'
            lcd_buf[20] <= 8'h31;                    // '1'
            lcd_buf[21] <= 8'h31;                    // '1'
            lcd_buf[22] <= 8'h31;                    // '1'
            lcd_buf[23] <= 8'h36;                    // '6'
            lcd_buf[24] <= 8'h33;                    // '3'
            lcd_buf[25] <= 8'h35;                    // '5'
            lcd_buf[26] <= 8'h57;                    // 'W'
            lcd_buf[27] <= 8'h48;                    // 'H'
            lcd_buf[28] <= 8'h5A;                    // 'Z'
            lcd_buf[29] <= 8'h20;                    // ' '
            lcd_buf[30] <= 8'h20;                    // ' '
            lcd_buf[31] <= 8'h20;                    // ' '

            lcd_req <= 1'b1;   // 请求 LCD 刷新
        end

        // ── 状态机 ─────────────────────────────────
        case (state)

            S_INIT: begin
                if (!lcd_busy) begin  // LCD 初始化完成 → 立即开始写第一屏
                    lcd_req  <= 1'b1;       // 触发 LCD 刷新 (不等 tick_1Hz)
                    state    <= S_IDLE;
                    ultra_timer <= ULTRA_PERIOD_MS;
                    dht_timer   <= 16'd0;
                    dht_pending <= 1'b1;
                end
            end

            S_IDLE: begin
                if (ultra_pending) begin
                    state <= S_ULTRA_TRIG;
                end else if (dht_pending) begin
                    state <= S_DHT_TRIG;
                end else if ((lcd_req || lcd_active) && !lcd_busy) begin
                    // LCD 刷新低优先级执行；每写完一个字节都会回到这里检查传感器请求。
                    if (!lcd_active) begin
                        lcd_req    <= 1'b0;
                        lcd_active <= 1'b1;
                        lcd_step   <= 6'd0;
                    end
                    lcd_wait_ack <= 1'b0;
                    state <= S_LCD_WRITE;
                end
            end

            S_ULTRA_TRIG: begin
                ultra_start <= 1'b1;
                ultra_pending <= 1'b0;
                ultra_timer <= ULTRA_PERIOD_MS;
                state       <= S_ULTRA_WAIT;
            end

            S_ULTRA_WAIT: begin
                if (ultra_done) state <= S_IDLE;
            end

            S_DHT_TRIG: begin
                dht_start <= 1'b1;
                dht_pending <= 1'b0;
                dht_timer <= (DHT_PERIOD_S > 0) ? (DHT_PERIOD_S - 1'b1) : 16'd0;
                state     <= S_DHT_WAIT;
            end

            S_DHT_WAIT: begin
                if (dht_done) state <= S_IDLE;
            end

            S_LCD_WRITE: begin
                if (lcd_wait_ack) begin
                    if (lcd_busy)
                        lcd_wait_ack <= 1'b0;
                end else if (lcd_busy) begin
                    // LCD is executing the command accepted above.
                end else if (lcd_step >= LCD_TOTAL[5:0]) begin
                    lcd_active <= 1'b0;
                    state <= S_IDLE;
                end else if (ultra_pending || dht_pending) begin
                    // 慢速 LCD 刷新让位给传感器触发，避免一次 34 字节刷新长期占住主状态机。
                    state <= S_IDLE;
                end else begin
                    lcd_step     <= lcd_step + 1'b1;
                    lcd_wait_ack <= 1'b1;
                    case (lcd_step)
                        6'd0:  begin lcd_wr_data <= 8'h80; lcd_wr_is_data <= 1'b0; lcd_wr <= 1'b1; end // ADDR row1
                        6'd1:  begin lcd_wr_data <= lcd_buf[ 0]; lcd_wr_is_data <= 1'b1; lcd_wr <= 1'b1; end
                        6'd2:  begin lcd_wr_data <= lcd_buf[ 1]; lcd_wr_is_data <= 1'b1; lcd_wr <= 1'b1; end
                        6'd3:  begin lcd_wr_data <= lcd_buf[ 2]; lcd_wr_is_data <= 1'b1; lcd_wr <= 1'b1; end
                        6'd4:  begin lcd_wr_data <= lcd_buf[ 3]; lcd_wr_is_data <= 1'b1; lcd_wr <= 1'b1; end
                        6'd5:  begin lcd_wr_data <= lcd_buf[ 4]; lcd_wr_is_data <= 1'b1; lcd_wr <= 1'b1; end
                        6'd6:  begin lcd_wr_data <= lcd_buf[ 5]; lcd_wr_is_data <= 1'b1; lcd_wr <= 1'b1; end
                        6'd7:  begin lcd_wr_data <= lcd_buf[ 6]; lcd_wr_is_data <= 1'b1; lcd_wr <= 1'b1; end
                        6'd8:  begin lcd_wr_data <= lcd_buf[ 7]; lcd_wr_is_data <= 1'b1; lcd_wr <= 1'b1; end
                        6'd9:  begin lcd_wr_data <= lcd_buf[ 8]; lcd_wr_is_data <= 1'b1; lcd_wr <= 1'b1; end
                        6'd10: begin lcd_wr_data <= lcd_buf[ 9]; lcd_wr_is_data <= 1'b1; lcd_wr <= 1'b1; end
                        6'd11: begin lcd_wr_data <= lcd_buf[10]; lcd_wr_is_data <= 1'b1; lcd_wr <= 1'b1; end
                        6'd12: begin lcd_wr_data <= lcd_buf[11]; lcd_wr_is_data <= 1'b1; lcd_wr <= 1'b1; end
                        6'd13: begin lcd_wr_data <= lcd_buf[12]; lcd_wr_is_data <= 1'b1; lcd_wr <= 1'b1; end
                        6'd14: begin lcd_wr_data <= lcd_buf[13]; lcd_wr_is_data <= 1'b1; lcd_wr <= 1'b1; end
                        6'd15: begin lcd_wr_data <= lcd_buf[14]; lcd_wr_is_data <= 1'b1; lcd_wr <= 1'b1; end
                        6'd16: begin lcd_wr_data <= lcd_buf[15]; lcd_wr_is_data <= 1'b1; lcd_wr <= 1'b1; end
                        6'd17: begin lcd_wr_data <= 8'hC0;      lcd_wr_is_data <= 1'b0; lcd_wr <= 1'b1; end // ADDR row2
                        6'd18: begin lcd_wr_data <= lcd_buf[16]; lcd_wr_is_data <= 1'b1; lcd_wr <= 1'b1; end
                        6'd19: begin lcd_wr_data <= lcd_buf[17]; lcd_wr_is_data <= 1'b1; lcd_wr <= 1'b1; end
                        6'd20: begin lcd_wr_data <= lcd_buf[18]; lcd_wr_is_data <= 1'b1; lcd_wr <= 1'b1; end
                        6'd21: begin lcd_wr_data <= lcd_buf[19]; lcd_wr_is_data <= 1'b1; lcd_wr <= 1'b1; end
                        6'd22: begin lcd_wr_data <= lcd_buf[20]; lcd_wr_is_data <= 1'b1; lcd_wr <= 1'b1; end
                        6'd23: begin lcd_wr_data <= lcd_buf[21]; lcd_wr_is_data <= 1'b1; lcd_wr <= 1'b1; end
                        6'd24: begin lcd_wr_data <= lcd_buf[22]; lcd_wr_is_data <= 1'b1; lcd_wr <= 1'b1; end
                        6'd25: begin lcd_wr_data <= lcd_buf[23]; lcd_wr_is_data <= 1'b1; lcd_wr <= 1'b1; end
                        6'd26: begin lcd_wr_data <= lcd_buf[24]; lcd_wr_is_data <= 1'b1; lcd_wr <= 1'b1; end
                        6'd27: begin lcd_wr_data <= lcd_buf[25]; lcd_wr_is_data <= 1'b1; lcd_wr <= 1'b1; end
                        6'd28: begin lcd_wr_data <= lcd_buf[26]; lcd_wr_is_data <= 1'b1; lcd_wr <= 1'b1; end
                        6'd29: begin lcd_wr_data <= lcd_buf[27]; lcd_wr_is_data <= 1'b1; lcd_wr <= 1'b1; end
                        6'd30: begin lcd_wr_data <= lcd_buf[28]; lcd_wr_is_data <= 1'b1; lcd_wr <= 1'b1; end
                        6'd31: begin lcd_wr_data <= lcd_buf[29]; lcd_wr_is_data <= 1'b1; lcd_wr <= 1'b1; end
                        6'd32: begin lcd_wr_data <= lcd_buf[30]; lcd_wr_is_data <= 1'b1; lcd_wr <= 1'b1; end
                        6'd33: begin lcd_wr_data <= lcd_buf[31]; lcd_wr_is_data <= 1'b1; lcd_wr <= 1'b1; end
                    endcase
                end
            end

            default: state <= S_INIT;

        endcase
    end
end

endmodule
