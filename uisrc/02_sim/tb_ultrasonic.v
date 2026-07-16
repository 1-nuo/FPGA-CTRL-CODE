// ============================================================
// tb_ultrasonic.v — 超声波模块仿真测试
// ============================================================
`timescale 1ns/1ps

module tb_ultrasonic;

reg         clk;
reg         rst_n;
reg         tick_1MHz;
reg         trigger_start;
wire        trig;
reg         echo = 0;
wire [15:0] distance_cm;
wire        measure_done;

ultrasonic_driver uut (
    .clk           (clk),
    .rst_n         (rst_n),
    .tick_1MHz     (tick_1MHz),
    .trigger_start (trigger_start),
    .trig          (trig),
    .echo          (echo),
    .distance_cm   (distance_cm),
    .measure_done  (measure_done)
);

initial clk = 0;
always #10 clk = ~clk;

// 产生 1MHz tick (1μs 脉冲)
reg [5:0] div_cnt;
initial div_cnt = 0;
always @(posedge clk) begin
    if (div_cnt >= 49) begin
        div_cnt <= 0;
        tick_1MHz <= 1;
    end else begin
        div_cnt <= div_cnt + 1;
        tick_1MHz <= 0;
    end
end

initial begin
    rst_n = 0;  echo = 0;  trigger_start = 0;
    #200 rst_n = 1;

    // 触发测距
    #1000 trigger_start = 1;
    #20   trigger_start = 0;

    // 等待 trig 脉冲完成后模拟 ECHO
    #30_000;     // 等 30μs (trig 脉冲约 11μs)
    echo = 1;
    // 模拟 100cm 障碍: 100cm × 58μs/cm = 5800μs = 5_800_000ns
    #5_800_000;
    echo = 0;

    #500_000;
    $display("Distance = %d cm (expect 100)", distance_cm);

    #1_000_000 $finish;
end

initial begin
    $dumpfile("tb_ultrasonic.vcd");
    $dumpvars(0, tb_ultrasonic);
end

endmodule
