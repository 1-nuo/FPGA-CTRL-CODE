`timescale 1ns/1ps

module tb_main_lcd_handshake;

reg         clk;
reg         rst_n;
reg         tick_1kHz;
reg         tick_1Hz;
reg  [3:0]  track_position;
reg         line_lost;
reg  [15:0] ultra_distance;
reg         ultra_done;
reg  [15:0] dht_temperature;
reg  [15:0] dht_humidity;
reg         dht_done;
reg         lcd_busy;

wire        ultra_start;
wire        dht_start;
wire        lcd_wr;
wire [7:0]  lcd_wr_data;
wire        lcd_wr_is_data;
wire [3:0]  motor_position;
wire        motor_line_lost;
wire [1:0]  motor_mode;
wire [7:0]  motor_speed;
wire [15:0] buzz_distance;

main_controller uut (
    .clk               (clk),
    .rst_n             (rst_n),
    .tick_1kHz         (tick_1kHz),
    .tick_1Hz          (tick_1Hz),
    .track_position    (track_position),
    .line_lost         (line_lost),
    .ultra_distance    (ultra_distance),
    .ultra_done        (ultra_done),
    .dht_temperature   (dht_temperature),
    .dht_humidity      (dht_humidity),
    .dht_done          (dht_done),
    .lcd_busy          (lcd_busy),
    .ultra_start       (ultra_start),
    .dht_start         (dht_start),
    .lcd_wr            (lcd_wr),
    .lcd_wr_data       (lcd_wr_data),
    .lcd_wr_is_data    (lcd_wr_is_data),
    .motor_position    (motor_position),
    .motor_line_lost   (motor_line_lost),
    .motor_mode        (motor_mode),
    .motor_speed       (motor_speed),
    .buzz_distance     (buzz_distance)
);

initial clk = 1'b0;
always #10 clk = ~clk;

reg [7:0] captured [0:33];
integer capture_count;
integer busy_countdown;
integer i;
reg     init_hold;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        capture_count  <= 0;
        busy_countdown <= 0;
    end else begin
        if (init_hold) begin
            lcd_busy <= 1'b1;
        end else if (lcd_wr && !lcd_busy) begin
            if (capture_count < 34)
                captured[capture_count] <= lcd_wr_data;
            capture_count  <= capture_count + 1;
            busy_countdown <= 4;
            lcd_busy       <= 1'b1;
        end else if (busy_countdown > 0) begin
            busy_countdown <= busy_countdown - 1;
            lcd_busy       <= 1'b1;
        end else begin
            lcd_busy       <= 1'b0;
        end
    end
end

task pulse_1hz;
begin
    @(posedge clk);
    tick_1Hz = 1'b1;
    @(posedge clk);
    tick_1Hz = 1'b0;
end
endtask

initial begin
    rst_n           = 1'b0;
    tick_1kHz       = 1'b0;
    tick_1Hz        = 1'b0;
    track_position  = 4'd7;
    line_lost       = 1'b0;
    ultra_distance  = 16'd0;
    ultra_done      = 1'b0;
    dht_temperature = 16'h1C00; // 28 C, DHT11 integer byte in [15:8]
    dht_humidity    = 16'h4100; // 65 %, DHT11 integer byte in [15:8]
    dht_done        = 1'b0;
    lcd_busy        = 1'b1;
    init_hold       = 1'b1;

    #200;
    rst_n = 1'b1;

    pulse_1hz();
    repeat (5) @(posedge clk);
    init_hold = 1'b0;

    wait (dht_start);
    repeat (5) @(posedge clk);
    dht_done = 1'b1;
    @(posedge clk);
    dht_done = 1'b0;

    wait (capture_count == 34);
    repeat (20) @(posedge clk);

    if (captured[0] !== 8'h80) begin
        $display("FAIL: captured[0]=%02h, expected 80", captured[0]);
        $fatal;
    end
    if (captured[1] !== "T" || captured[2] !== ":" || captured[3] !== "+") begin
        $display("FAIL: row1 prefix %02h %02h %02h", captured[1], captured[2], captured[3]);
        $fatal;
    end
    if (captured[4] !== "2" || captured[5] !== "8" ||
        captured[10] !== "6" || captured[11] !== "5") begin
        $display("FAIL: DHT display digits T=%c%c H=%c%c",
                 captured[4], captured[5], captured[10], captured[11]);
        $fatal;
    end
    if (captured[17] !== 8'hC0) begin
        $display("FAIL: captured[17]=%02h, expected C0", captured[17]);
        $fatal;
    end
    if (captured[18] !== "2" || captured[19] !== "0" || captured[20] !== "2" || captured[21] !== "4") begin
        $display("FAIL: row2 prefix %02h %02h %02h %02h",
                 captured[18], captured[19], captured[20], captured[21]);
        $fatal;
    end

    $display("PASS: captured 34 ordered LCD writes");
    for (i = 0; i < 34; i = i + 1)
        $write("%02h ", captured[i]);
    $display("");
    $finish;
end

endmodule
