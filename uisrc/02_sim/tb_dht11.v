// ============================================================
// tb_dht11.v — DHT11 模块仿真测试 (完整 40 位协议模拟)
// 模拟 DHT11 温湿度传感器回应:
//   湿度=65% (0x41), 温度=28°C (0x1C)
//   校验和 = 0x41+0x00+0x1C+0x00 = 0x5D
// ============================================================
`timescale 1ns/1ps

module tb_dht11;

reg         clk;
reg         rst_n;
reg         tick_1MHz;
reg         start_read;
tri1        dht_data;          // 模拟 DHT11 总线外部上拉
wire [15:0] temperature;
wire [15:0] humidity;
wire        read_done;

dht11_driver uut (
    .clk         (clk),
    .rst_n       (rst_n),
    .tick_1MHz   (tick_1MHz),
    .start_read  (start_read),
    .dht_data    (dht_data),
    .temperature (temperature),
    .humidity    (humidity),
    .read_done   (read_done)
);

// ── 时钟 + tick_1MHz ─────────────────────────────────────
initial clk = 0;
always #10 clk = ~clk;

reg [5:0] div_cnt = 0;
always @(posedge clk) begin
    if (div_cnt >= 49) begin
        div_cnt <= 0;
        tick_1MHz <= 1;
    end else begin
        div_cnt <= div_cnt + 1;
        tick_1MHz <= 0;
    end
end

// ── DHT11 总线驱动器 ─────────────────────────────────────
// 模拟 DHT11 芯片: 在 dt_drv=1 时驱动总线
reg dt_drv;
reg dt_val;
assign dht_data = dt_drv ? dt_val : 1'bz;

// ── DHT11 响应状态机 ─────────────────────────────────────
reg [31:0] cnt;               // 微秒计数器
reg [5:0]  bit_ptr;           // 当前位 (0~39)
reg [39:0] tx_data;           // 要发送的 40 位数据

// 测试数据: 湿度=65%, 温度=28°C
// 数据格式: 湿度高8位 + 湿度低8位 + 温度高8位 + 温度低8位 + 校验和
localparam [7:0] HUM_H = 8'h41;  // 65
localparam [7:0] HUM_L = 8'h00;  // 0
localparam [7:0] TMP_H = 8'h1C;  // 28
localparam [7:0] TMP_L = 8'h00;  // 0
// 校验和 = 0x41+0x00+0x1C+0x00 = 0x5D
localparam [7:0] CHK   = 8'h5D;

// ── 主测试流程 ──────────────────────────────────────────
initial begin
    // 初始化
    rst_n = 0;  start_read = 0;
    dt_drv = 0; dt_val = 0;
    cnt = 0; bit_ptr = 0;
    #200 rst_n = 1;

    // 构建 40 位发送数据
    tx_data = {HUM_H, HUM_L, TMP_H, TMP_L, CHK};

    $display("=== DHT11 Test: expect Temp=28, Hum=65 ===");
    $display("TX data: %h %h %h %h %h (chk=%h)",
             HUM_H, HUM_L, TMP_H, TMP_L, CHK, CHK);

    // 等待模块空闲，然后触发读取
    #1000;
    @(posedge clk);
    start_read = 1;
    @(posedge clk);
    start_read = 0;

    // ── 等待模块拉低总线 (START_LOW, 18ms) ──────────
    // 此时模块 dht_oe=1, dht_out=0, dht_data 被驱动为 0
    // 我们在 dt_drv=0 状态下等待
    $display("=== Module pulling LOW (18ms) ===");
    wait (uut.dht_oe == 1'b1);
    wait (uut.dht_oe == 1'b0);

    // ── 模块释放总线 (START_HIGH, ~30μs) ─────────────
    // 模块设 dht_oe=0, 总线浮空
    // 等待 40μs 后开始 DHT11 响应
    $display("=== Module released, wait 25us then respond ===");
    cnt = 0;
    while (cnt < 25) begin
        @(posedge tick_1MHz);
        cnt = cnt + 1;
    end

    // ── DHT11 响应: 拉低 80μs ────────────────────────
    $display("=== DHT11 RESPONSE: pull LOW 80us ===");
    dt_drv = 1; dt_val = 0;
    cnt = 0;
    while (cnt < 82) begin
        @(posedge tick_1MHz);
        cnt = cnt + 1;
    end

    // ── DHT11 响应: 拉高 80μs ────────────────────────
    $display("=== DHT11 RESPONSE: pull HIGH 80us ===");
    dt_val = 1;
    cnt = 0;
    while (cnt < 82) begin
        @(posedge tick_1MHz);
        cnt = cnt + 1;
    end

    // ── 发送 40 位数据 ───────────────────────────────
    $display("=== DHT11 DATA: 40 bits ===");
    for (bit_ptr = 0; bit_ptr < 40; bit_ptr = bit_ptr + 1) begin
        // 每 bit 开始: 拉低 50μs
        dt_val = 0;
        cnt = 0;
        while (cnt < 52) begin
            @(posedge tick_1MHz);
            cnt = cnt + 1;
        end

        // 拉高: 28μs=bit0, 70μs=bit1
        dt_val = 1;
        if (tx_data[39 - bit_ptr]) begin  // MSB first
            cnt = 0;
            while (cnt < 72) begin
                @(posedge tick_1MHz);
                cnt = cnt + 1;
            end
        end else begin
            cnt = 0;
            while (cnt < 30) begin
                @(posedge tick_1MHz);
                cnt = cnt + 1;
            end
        end
    end

    // ── DHT11 结束: 释放总线 ─────────────────────────
    @(posedge tick_1MHz);
    dt_drv = 0;
    $display("=== DHT11 DATA transmission complete ===");

    // 等待模块处理完成；read_done 只是一个脉冲，不能只靠沿触发
    wait (uut.state == 5'd0 && uut.bit_cnt == 6'd40);
    #2000;
    $display("Temp = %d (expect 28), Hum = %d (expect 65)",
             temperature[15:8], humidity[15:8]);
    if (temperature !== 16'h1C00 || humidity !== 16'h4100) begin
        $display("FAIL: raw temperature=%h humidity=%h", temperature, humidity);
        $stop;
    end
    $display("=== Test Complete ===");

    #50000 $finish;
end

// ── 波形 ────────────────────────────────────────────────
initial begin
    $dumpfile("tb_dht11.vcd");
    $dumpvars(0, tb_dht11);
end

endmodule
