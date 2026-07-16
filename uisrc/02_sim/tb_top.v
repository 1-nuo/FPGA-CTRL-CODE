// ============================================================
// tb_top.v — 顶层仿真测试
// ============================================================
`timescale 1ns/1ps

module tb_top;

reg         clk;
reg         rst_n;
reg  [5:0]  track_sensor;
wire        m1_in1, m1_in2, m2_in3, m2_in4;
wire        m3_in1, m3_in2, m4_in3, m4_in4;
wire        lcd_rs, lcd_en;
wire [3:0]  lcd_data;
wire        ultra_trig;
reg         ultra_echo;
wire        dht_data = 1'b1;  // 默认上拉
wire        buzzer_out;

top u_top (
    .clk          (clk),
    .rst_n        (rst_n),
    .track_sensor (track_sensor),
    .m1_in1       (m1_in1),
    .m1_in2       (m1_in2),
    .m2_in3       (m2_in3),
    .m2_in4       (m2_in4),
    .m3_in1       (m3_in1),
    .m3_in2       (m3_in2),
    .m4_in3       (m4_in3),
    .m4_in4       (m4_in4),
    .lcd_rs       (lcd_rs),
    .lcd_en       (lcd_en),
    .lcd_data     (lcd_data),
    .ultra_trig   (ultra_trig),
    .ultra_echo   (ultra_echo),
    .dht_data     (dht_data),
    .buzzer_out   (buzzer_out)
);

// ── 时钟 50MHz ──────────────────────────────────────────────
initial clk = 0;
always #10 clk = ~clk;

// ── 测试激励 ────────────────────────────────────────────────
initial begin
    rst_n = 0;
    track_sensor = 6'b111111;
    ultra_echo = 0;
    #100;
    rst_n = 1;

    // 模拟 S3、S4 检测到黑线（居中偏右）
    #1000 track_sensor = 6'b001111;

    // 模拟全部白底（脱线）
    #100000 track_sensor = 6'b111111;

    // 模拟 ECHO 返回（模拟 50cm 障碍）
    #200000;
    ultra_echo = 1;
    #2900;      // 50cm × 58μs
    ultra_echo = 0;

    #500000 $finish;
end

initial begin
    $dumpfile("tb_top.vcd");
    $dumpvars(0, tb_top);
end

endmodule
