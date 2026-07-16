// ============================================================
// tb_line_tracking.v — 循迹模块仿真测试
// ============================================================
`timescale 1ns/1ps

module tb_line_tracking;

reg        clk;
reg        rst_n;
reg [5:0]  sensor_in;
wire [3:0] position;
wire       line_lost;

line_tracking uut (
    .clk       (clk),
    .rst_n     (rst_n),
    .sensor_in (sensor_in),
    .position  (position),
    .line_lost (line_lost)
);

initial clk = 0;
always #10 clk = ~clk;

initial begin
    rst_n = 0;  sensor_in = 6'b111111;
    #100 rst_n = 1;

    // 全部白底 → line_lost=1
    #200 $display("S=111111 pos=%d lost=%b", position, line_lost);

    // S3 黑线 → 居中
    sensor_in = 6'b110111;  // S3=0
    #200 $display("S=110111 pos=%d lost=%b", position, line_lost);

    // S1+S2 黑线 → 偏左
    sensor_in = 6'b111100;  // S1=S2=0
    #200 $display("S=111100 pos=%d lost=%b", position, line_lost);

    // S5+S6 黑线 → 偏右
    sensor_in = 6'b001111;  // S5=S6=0
    #200 $display("S=001111 pos=%d lost=%b", position, line_lost);

    #500 $finish;
end

initial begin
    $dumpfile("tb_line_tracking.vcd");
    $dumpvars(0, tb_line_tracking);
end

endmodule
