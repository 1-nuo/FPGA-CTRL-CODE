// ============================================================
// lcd_driver.v — LCD1602 HD44780 4-bit 模式驱动模块
// 参考: HD44780 datasheet (ADE-207-272Z) + gateGPT HD44780 driver
// 4位模式下每个字节分高4位和低4位两次发送
// 基于 tick_1kHz (1ms/tick) 做时序控制
// ============================================================

module lcd_driver #(
    parameter INIT_WAIT_MS = 40     // 上电等待 (ms)，仿真可改小
) (
    input  wire         clk,
    input  wire         rst_n,
    input  wire         tick_1kHz,           // ~1kHz 使能 (1ms)
    // 命令接口
    input  wire         cmd_valid,           // 命令有效 (脉冲)
    input  wire [7:0]   cmd_data,            // 命令或数据
    input  wire         cmd_is_data,          // 1=显示数据, 0=指令
    output reg          cmd_busy = 1'b1,     // 忙标志 (默认1: 上电等LCD初始化)
    output reg          init_done,           // 初始化完成标志
    // LCD 引脚
    output reg          lcd_rs,
    output reg          lcd_en,
    output reg [3:0]    lcd_data            // {DB7,DB6,DB5,DB4}
);

// ====================================================================
//  状态定义
// ====================================================================
localparam S_INIT_WAIT   = 4'd0;   // 上电等待 40ms
localparam S_INIT_HI      = 4'd1;   // 发初始化命令高4位
localparam S_INIT_E1      = 4'd2;   // E=1 (高4位锁存)
localparam S_INIT_LO      = 4'd3;   // 发初始化命令低4位
localparam S_INIT_E2      = 4'd4;   // E=1 (低4位锁存)
localparam S_INIT_DLY     = 4'd5;   // 命令间延时
localparam S_READY        = 4'd6;   // 正常待命
localparam S_TEST_ADDR    = 4'd7;   // 自检: 设DDRAM地址
localparam S_TEST_DATA    = 4'd8;   // 自检: 发字符
localparam S_CMD_HI       = 4'd9;   // 发正常命令高4位
localparam S_CMD_E1       = 4'd10;  // E=1 (高4位)
localparam S_CMD_LO       = 4'd11;  // 发正常命令低4位
localparam S_CMD_E2       = 4'd12;  // E=1 (低4位)
localparam S_CMD_DLY      = 4'd13;  // 命令执行延时
// 4'd14/15 reserved for future

reg [3:0] st;

// ====================================================================
//  HD44780 4-bit 模式初始化序列 ROM
//  init_seq[7:0] = 完整8位命令
//  init_dly[3:0] = 命令执行后延时 (1kHz tick 数)
//  前 4 条只需高 4 位 (8-bit 同步阶段)
// ====================================================================
localparam N_INIT = 8;
reg [7:0]  init_seq [0:N_INIT-1];
reg [3:0]  init_dly [0:N_INIT-1];
reg [3:0]  ip;                     // init pointer
reg [4:0]  test_step;              // 自检写入进度

// 延时计数器 + 命令缓存
reg [7:0]  dc;                     // delay counter
reg [7:0]  cmd_buf;
reg        cmd_isdata_buf;
reg        is_nibble_only;         // 1=只需高4位 (前4条命令)

initial begin
    // {命令, 延时(tick数@1kHz)}
    init_seq[0] = 8'h30; init_dly[0] = 4'd5;    // 0x3, 等 5ms (>4.1ms)
    init_seq[1] = 8'h30; init_dly[1] = 4'd1;    // 0x3, 等 1ms (>100µs)
    init_seq[2] = 8'h30; init_dly[2] = 4'd1;    // 0x3, 等 1ms
    init_seq[3] = 8'h20; init_dly[3] = 4'd5;    // 0x2 → 4-bit, 等 5ms
    init_seq[4] = 8'h28; init_dly[4] = 4'd1;    // Function Set: 4-bit, 2行, 5×8
    init_seq[5] = 8'h0C; init_dly[5] = 4'd1;    // Display ON, cursor OFF
    init_seq[6] = 8'h01; init_dly[6] = 4'd5;    // Clear Display (>1.52ms)
    init_seq[7] = 8'h06; init_dly[7] = 4'd1;    // Entry Mode: inc, no shift
end

// ====================================================================
//  主状态机
//  所有状态切换由 tick_1kHz 驱动 (50MHz clock 太快)
// ====================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        st              <= S_INIT_WAIT;
        lcd_rs          <= 1'b0;
        lcd_en          <= 1'b0;
        lcd_data        <= 4'd0;
        cmd_busy        <= 1'b1;
        init_done       <= 1'b0;
        cmd_buf         <= 8'd0;
        cmd_isdata_buf  <= 1'b0;
        ip              <= 4'd0;
        dc              <= 8'd0;
        is_nibble_only  <= 1'b0;
    end else if (tick_1kHz || (st == S_READY)) begin
        // S_READY: 每个时钟周期检查 cmd_valid (免被 tick_1kHz 错过)
        // 其他状态: 由 tick_1kHz 驱动 (E 脉冲/延时需 1ms 时序)

        case (st)

            // ════════════════════════════════════════════════
            //  初始化阶段
            // ════════════════════════════════════════════════

            S_INIT_WAIT: begin                        // 上电等 40ms (参数可调)
                cmd_busy <= 1'b1;
                lcd_rs   <= 1'b0;
                lcd_data <= 4'd0;
                if (dc >= INIT_WAIT_MS) begin          // 40 tick = 40ms
                    dc <= 8'd0;
                    ip <= 4'd0;
                    // 装载第一条命令
                    lcd_data      <= init_seq[0][7:4];  // 高 4 位
                    is_nibble_only <= (4'd0 <= 3);       // 前 4 条只需高 4 位
                    lcd_en        <= 1'b1;
                    st            <= S_INIT_E1;
                end else begin
                    dc     <= dc + 1'b1;
                    lcd_en <= 1'b0;
                end
            end

            S_INIT_HI: begin                          // 发当前命令高 4 位
                lcd_rs   <= 1'b0;
                lcd_data <= init_seq[ip][7:4];
                is_nibble_only <= (ip < 4);
                lcd_en   <= 1'b1;
                st       <= S_INIT_E1;
            end

            S_INIT_E1: begin                          // 高 4 位 E 脉冲结束 (下降沿锁存)
                lcd_rs   <= 1'b0;
                lcd_en   <= 1'b0;                     // E 下降 → 锁存高4位
                if (is_nibble_only) begin             // 只需高4位 → 延时
                    dc <= 8'd0;
                    st <= S_INIT_DLY;
                end else begin                        // 需低4位 → 转到 S_INIT_LO
                    st <= S_INIT_LO;                  // 下一拍再发低4位
                end
            end

            S_INIT_LO: begin                          // 发当前命令低 4 位
                lcd_rs   <= 1'b0;
                lcd_data <= init_seq[ip][3:0];
                lcd_en   <= 1'b1;
                st       <= S_INIT_E2;
            end

            S_INIT_E2: begin                          // 低 4 位 E 脉冲结束
                lcd_rs   <= 1'b0;
                lcd_en   <= 1'b0;
                dc       <= 8'd0;
                st       <= S_INIT_DLY;
            end

            S_INIT_DLY: begin                         // 命令执行延时
                lcd_en   <= 1'b0;
                lcd_rs   <= 1'b0;
                if (dc >= init_dly[ip]) begin
                    dc <= 8'd0;
                    ip <= ip + 1'b1;
                    if (ip + 1 >= N_INIT) begin
                        // 初始化完成 → 进入正常模式
                        cmd_busy  <= 1'b0;
                        init_done <= 1'b1;
                        st        <= S_READY;
                    end else begin
                        // 下一条初始化命令
                        st <= S_INIT_HI;
                    end
                end else begin
                    dc <= dc + 1'b1;
                end
            end

            // ════════════════════════════════════════════════
            //  正常工作阶段
            // ════════════════════════════════════════════════

            S_READY: begin
                cmd_busy <= 1'b0;
                lcd_en   <= 1'b0;              // 空闲时 E=0
                if (cmd_valid) begin
                    cmd_buf        <= cmd_data;
                    cmd_isdata_buf <= cmd_is_data;
                    cmd_busy       <= 1'b1;
                    // 发高 4 位
                    lcd_data       <= cmd_data[7:4];
                    lcd_rs         <= cmd_is_data;
                    lcd_en         <= 1'b1;
                    st             <= S_CMD_E1;
                end
            end

            S_CMD_HI: begin
                lcd_data <= cmd_buf[7:4];
                lcd_rs   <= cmd_isdata_buf;
                lcd_en   <= 1'b1;
                st       <= S_CMD_E1;
            end

            S_CMD_E1: begin
                // 保持 E 高电平；下降沿后继续保持高4位，下一 tick 再切低4位
                lcd_rs   <= cmd_isdata_buf;
                if (tick_1kHz) begin
                    lcd_en   <= 1'b0;          // E 下降 → 锁存高4位
                    st       <= S_CMD_LO;
                end else begin
                    lcd_en   <= 1'b1;          // 继续保持 E 高
                end
            end

            S_CMD_LO: begin
                lcd_data <= cmd_buf[3:0];
                lcd_rs   <= cmd_isdata_buf;
                lcd_en   <= 1'b1;
                st       <= S_CMD_E2;
            end

            S_CMD_E2: begin
                lcd_en   <= 1'b0;
                dc       <= 8'd0;
                st       <= S_CMD_DLY;
            end

            S_CMD_DLY: begin                         // 等 40µs+ → 1 tick 足够
                lcd_en   <= 1'b0;
                if (dc >= 1) begin                   // 1ms > 40µs
                    dc       <= 8'd0;
                    cmd_busy <= 1'b0;
                    st       <= S_READY;
                end else begin
                    dc <= dc + 1'b1;
                end
            end

            default: st <= S_INIT_WAIT;

        endcase
    end
end

endmodule
