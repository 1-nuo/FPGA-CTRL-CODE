// ============================================================
// dht11_driver.v — DHT11 温湿度采集模块
// 功能：单总线协议，40 位数据读取
// 数据格式: 湿度高8位+湿度低8位+温度高8位+温度低8位+校验和
// 采样方式: 边沿检测 + 定时采样 (不使用固定定时)
// ============================================================

module dht11_driver (
    input  wire         clk,
    input  wire         rst_n,
    input  wire         tick_1MHz,          // 1MHz 使能
    input  wire         start_read,         // 触发读取
    inout  wire         dht_data,           // 双向单总线
    output reg [15:0]   temperature,        // 温度 (16位)
    output reg [15:0]   humidity,           // 湿度 (16位)
    output reg          read_done           // 读取完成
);

// ── 双向 IO 控制 ────────────────────────────────────────────
reg dht_out;
reg dht_oe;              // 输出使能: 1=FPGA驱动, 0=高阻(让DHT11驱动)
assign dht_data = dht_oe ? dht_out : 1'bz;

// ── 边沿检测 ────────────────────────────────────────────────
reg dht_r1, dht_r2;
wire dht_rising, dht_falling;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        dht_r1 <= 1'b1;
        dht_r2 <= 1'b1;
    end else begin
        dht_r1 <= dht_data;
        dht_r2 <= dht_r1;
    end
end

assign dht_rising  = ( dht_r1 & ~dht_r2);
assign dht_falling = (~dht_r1 &  dht_r2);
wire dht_in = dht_r2;

// ── 状态机 ──────────────────────────────────────────────────
localparam IDLE        = 5'd0;
localparam START_LOW   = 5'd1;    // 拉低 ≥ 18ms
localparam START_HIGH  = 5'd2;    // 释放并上拉 20~40μs
localparam WAIT_RESP   = 5'd3;    // 等待 DHT11 拉低(响应开始)
localparam RESP_LOW    = 5'd4;    // DHT11 拉低 ~80μs (边沿检测)
localparam RESP_HIGH   = 5'd5;    // DHT11 拉高 ~80μs (边沿检测)
localparam READ_BIT    = 5'd6;    // 读取 40 位数据
localparam BIT_LOW     = 5'd7;    // 每 bit 低电平, 等待上升沿
localparam SAMPLE_BIT  = 5'd8;    // 定时采样: 40μs 后读 dht_data
localparam CHECKSUM    = 5'd9;    // 校验
localparam DONE        = 5'd10;

reg [4:0]  state;
reg [23:0] delay_cnt;            // 长延时计数
reg [5:0]  bit_cnt;              // 当前位 (0~39)
reg [39:0] data_buf;             // 40 位移位寄存器
reg        bit_low_seen;         // 当前 bit 的 50us 低电平前导是否已看到

// ── 主状态机 ────────────────────────────────────────────────
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state       <= IDLE;
        dht_out     <= 1'b1;
        dht_oe      <= 1'b0;
        delay_cnt   <= 24'd0;
        bit_cnt     <= 6'd0;
        data_buf    <= 40'd0;
        bit_low_seen <= 1'b0;
        temperature <= 16'd0;
        humidity    <= 16'd0;
        read_done   <= 1'b0;
    end else begin
        read_done <= 1'b0;

        case (state)

            IDLE: begin
                if (start_read) begin
                    dht_oe    <= 1'b1;        // FPGA 驱动总线
                    dht_out   <= 1'b0;         // 拉低
                    delay_cnt <= 24'd0;
                    state     <= START_LOW;
                end
            end

            // ═══ 主机启动序列 (不变) ═════════════════════
            START_LOW: begin                   // 拉低 ≥ 18ms
                if (tick_1MHz) begin
                    if (delay_cnt >= 18000) begin  // 18ms
                        dht_oe    <= 1'b0;        // 释放总线
                        dht_out   <= 1'b1;
                        delay_cnt <= 24'd0;
                        state     <= START_HIGH;
                    end else begin
                        delay_cnt <= delay_cnt + 1'b1;
                    end
                end
            end

            START_HIGH: begin                  // 释放 20~40μs (等 DHT11 反应)
                if (tick_1MHz && delay_cnt >= 30) begin
                    delay_cnt <= 24'd0;
                    state     <= WAIT_RESP;
                end else if (tick_1MHz) begin
                    delay_cnt <= delay_cnt + 1'b1;
                end
            end

            // ═══ DHT11 响应 (边沿检测) ═══════════════════
            WAIT_RESP: begin                   // 等 DHT11 拉低 (响应开始)
                if (dht_falling || !dht_in) begin
                    delay_cnt <= 24'd0;
                    state     <= RESP_LOW;
                end
                // 超时 500μs
                if (tick_1MHz && delay_cnt >= 500) begin
                    state <= DONE;
                end else if (tick_1MHz) begin
                    delay_cnt <= delay_cnt + 1'b1;
                end
            end

            RESP_LOW: begin                    // DHT11 拉低中, 等上升沿
                if (dht_rising || dht_in) begin
                    delay_cnt <= 24'd0;
                    state     <= RESP_HIGH;
                end
                // 超时 200μs
                if (tick_1MHz && delay_cnt >= 200) begin
                    state <= DONE;
                end else if (tick_1MHz) begin
                    delay_cnt <= delay_cnt + 1'b1;
                end
            end

            RESP_HIGH: begin                   // DHT11 拉高中, 等下降沿
                if (dht_falling || !dht_in) begin
                    delay_cnt <= 24'd0;
                    bit_cnt   <= 6'd0;
                    state     <= READ_BIT;
                end
                // 超时 200μs
                if (tick_1MHz && delay_cnt >= 200) begin
                    state <= DONE;
                end else if (tick_1MHz) begin
                    delay_cnt <= delay_cnt + 1'b1;
                end
            end

            // ═══ 数据读取 (边沿检测 + 定时采样) ═════════
            READ_BIT: begin                    // 读取 40 位
                if (bit_cnt < 40) begin
                    delay_cnt <= 24'd0;
                    bit_low_seen <= 1'b0;
                    state     <= BIT_LOW;
                end else begin
                    state <= CHECKSUM;
                end
            end

            BIT_LOW: begin                     // 每 bit: 50μs 低电平, 等上升沿
                if (!bit_low_seen) begin
                    if (dht_falling || !dht_in) begin
                        bit_low_seen <= 1'b1;
                        delay_cnt    <= 24'd0;
                    end else if (tick_1MHz && delay_cnt >= 200) begin
                        state <= DONE;
                    end else if (tick_1MHz) begin
                        delay_cnt <= delay_cnt + 1'b1;
                    end
                end else begin
                    if (dht_rising) begin
                        delay_cnt <= 24'd0;
                        state     <= SAMPLE_BIT;
                    end else if (tick_1MHz && delay_cnt >= 200) begin
                        state <= DONE;
                    end else if (tick_1MHz) begin
                        delay_cnt <= delay_cnt + 1'b1;
                    end
                end
            end

            SAMPLE_BIT: begin                  // 上升沿后 40μs 采样 dht_data
                if (tick_1MHz) begin
                    if (delay_cnt >= 40) begin
                        // 40μs 是 28μs(bit0) 和 70μs(bit1) 的中点
                        if (dht_in)
                            data_buf <= {data_buf[38:0], 1'b1};  // 仍高→bit1
                        else
                            data_buf <= {data_buf[38:0], 1'b0};  // 已低→bit0
                        bit_cnt <= bit_cnt + 1'b1;
                        state   <= READ_BIT;
                    end else begin
                        delay_cnt <= delay_cnt + 1'b1;
                    end
                end
                // 超时 200μs (保护)
                if (tick_1MHz && delay_cnt >= 200) begin
                    state <= DONE;
                end
            end

            // ═══ 校验与完成 ═══════════════════════════════
            CHECKSUM: begin                    // 校验
                if ((data_buf[39:32] + data_buf[31:24] +
                     data_buf[23:16] + data_buf[15:8] == data_buf[7:0]) &&
                    (data_buf[31:24] == 8'd0) &&
                    (data_buf[15:8]  == 8'd0) &&
                    (data_buf[39:32] <= 8'd90) &&
                    (data_buf[23:16] <= 8'd60)) begin
                    humidity    <= {data_buf[39:32], data_buf[31:24]};
                    temperature <= {data_buf[23:16], data_buf[15:8]};
                end else begin
                    // 校验失败: 保持旧值
                end
                state <= DONE;
            end

            DONE: begin
                read_done <= 1'b1;
                state     <= IDLE;
            end

            default: state <= IDLE;

        endcase
    end
end

endmodule
