// ============================================================
// tb_lcd_driver.v — LCD 驱动模块仿真测试
// ============================================================
`timescale 1ns/1ps

module tb_lcd_driver;

reg         clk;
reg         rst_n;
reg         tick_1kHz;
reg         cmd_valid;
reg  [7:0]  cmd_data;
reg         cmd_is_data;
wire        cmd_busy;
wire        lcd_rs, lcd_en;
wire [3:0]  lcd_data;

lcd_driver #(.INIT_WAIT_MS(1)) uut (   // 仿真加速: 上电等 1ms
    .clk          (clk),
    .rst_n        (rst_n),
    .tick_1kHz    (tick_1kHz),
    .cmd_valid    (cmd_valid),
    .cmd_data     (cmd_data),
    .cmd_is_data  (cmd_is_data),
    .cmd_busy     (cmd_busy),
    .lcd_rs       (lcd_rs),
    .lcd_en       (lcd_en),
    .lcd_data     (lcd_data)
);

initial clk = 0;
always #10 clk = ~clk;

reg [15:0] div_cnt = 0;
always @(posedge clk) begin
    if (div_cnt >= 999) begin           // 仿真加速: 50kHz tick
        div_cnt <= 0;
        tick_1kHz <= 1;
    end else begin
        div_cnt <= div_cnt + 1;
        tick_1kHz <= 0;
    end
end

initial begin
    rst_n = 0;  cmd_valid = 0;  cmd_data = 0;  cmd_is_data = 0;
    #200 rst_n = 1;

    // 等待初始化完成 (INIT_WAIT_MS=1 + init sequence ~= 22ms)
    // Wait until cmd_busy goes low
    @(negedge cmd_busy);
    $display("=== LCD Init Complete at %t ===", $time);
    #1000;

    // 发指令: Set DDRAM addr 0x80
    @(posedge clk);
    cmd_valid = 1;  cmd_data = 8'h80;  cmd_is_data = 0;
    @(posedge clk);
    cmd_valid = 0;
    $display("=== Sent CMD: 0x80 (Set DDRAM) at %t ===", $time);

    // Wait for busy to clear
    @(negedge cmd_busy);
    #1000;

    // 发数据: 'A' (0x41)
    @(posedge clk);
    cmd_valid = 1;  cmd_data = 8'h41;  cmd_is_data = 1;
    @(posedge clk);
    cmd_valid = 0;
    $display("=== Sent DATA: 'A' (0x41) at %t ===", $time);

    @(negedge cmd_busy);
    #1000;
    $display("=== Test Complete! ===");
    #50000 $finish;
end

initial begin
    $dumpfile("tb_lcd_driver.vcd");
    $dumpvars(0, tb_lcd_driver);
end

endmodule
