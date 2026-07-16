// ============================================================
// tb_motor_control.v — 电机 PWM 与循迹差速的基础仿真
// ============================================================
`timescale 1ns/1ps

module tb_motor_control;

reg        clk;
reg        rst_n;
reg        tick_10kHz;
reg [3:0]  position;
reg        line_lost;
reg [1:0]  mode;
reg [3:0]  turn_dir;
reg [7:0]  base_speed;

wire m1_in1, m1_in2;
wire m2_in3, m2_in4;
wire m3_in1, m3_in2;
wire m4_in3, m4_in4;

motor_control uut (
    .clk        (clk),
    .rst_n      (rst_n),
    .tick_10kHz (tick_10kHz),
    .position   (position),
    .line_lost  (line_lost),
    .mode       (mode),
    .turn_dir   (turn_dir),
    .base_speed (base_speed),
    .m1_in1     (m1_in1),
    .m1_in2     (m1_in2),
    .m2_in3     (m2_in3),
    .m2_in4     (m2_in4),
    .m3_in1     (m3_in1),
    .m3_in2     (m3_in2),
    .m4_in4     (m4_in4),
    .m4_in3     (m4_in3)
);

initial clk = 1'b0;
always #10 clk = ~clk; // 50MHz

reg [12:0] tick_div;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        tick_div   <= 13'd0;
        tick_10kHz <= 1'b0;
    end else if (tick_div == 13'd4999) begin
        tick_div   <= 13'd0;
        tick_10kHz <= 1'b1;
    end else begin
        tick_div   <= tick_div + 1'b1;
        tick_10kHz <= 1'b0;
    end
end

task wait_control_ticks;
    input integer n;
    integer i;
    begin
        for (i = 0; i < n; i = i + 1) begin
            @(posedge tick_10kHz);
            @(posedge clk);
        end
    end
endtask

initial begin
    rst_n      = 1'b0;
    position   = 4'd7;
    line_lost  = 1'b0;
    mode       = 2'b01;
    turn_dir   = 4'b0000;
    base_speed = 8'd128;

    #200;
    rst_n = 1'b1;

    // PWM 周期应为 5000 个 50MHz 时钟，即 10kHz。
    wait (uut.pwm_cnt == 13'd4999);
    @(posedge clk);
    #1;
    if (uut.pwm_cnt !== 13'd0) begin
        $display("FAIL: PWM counter did not wrap at 5000 clocks");
        $finish;
    end

    // 居中死区：左右输出应最终一致。
    position = 4'd7;
    wait_control_ticks(40);
    if (uut.left_out !== uut.right_out) begin
        $display("FAIL: deadband output mismatch L=%0d R=%0d", uut.left_out, uut.right_out);
        $finish;
    end

    // 偏左：降低左轮，且不低于最低速度。
    position = 4'd4;
    wait_control_ticks(30);
    if (!(uut.left_out < uut.right_out && uut.left_out >= 8'd40)) begin
        $display("FAIL: left correction invalid L=%0d R=%0d", uut.left_out, uut.right_out);
        $finish;
    end

    // 偏右：降低右轮，且不低于最低速度。
    position = 4'd10;
    wait_control_ticks(40);
    if (!(uut.right_out < uut.left_out && uut.right_out >= 8'd40)) begin
        $display("FAIL: right correction invalid L=%0d R=%0d", uut.left_out, uut.right_out);
        $finish;
    end

    $display("PASS: motor_control basic PWM/PD checks passed");
    $finish;
end

endmodule
