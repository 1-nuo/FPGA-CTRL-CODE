# ============================================================
# de1_soc_test.sdc — 时序约束
# 器件: Cyclone V 5CSEMA5F31C6, 时钟: 50MHz
# ============================================================

# ── 主时钟: 50MHz 板载晶振
create_clock -name clk_50 -period 20.000 -waveform {0 10} [get_ports {clk}]

# ── 衍生时钟与不确定性
derive_pll_clocks -use_tan_name
derive_clock_uncertainty

# ── 输入延迟 (外部传感器到 FPGA)
set_input_delay -clock [get_clocks clk_50] -max 5 [get_ports {track_sensor*}]
set_input_delay -clock [get_clocks clk_50] -min 2 [get_ports {track_sensor*}]
set_input_delay -clock [get_clocks clk_50] -max 5 [get_ports {ultra_echo}]
set_input_delay -clock [get_clocks clk_50] -min 2 [get_ports {ultra_echo}]

# ── 输出延迟 (FPGA 到外部器件)
set_output_delay -clock [get_clocks clk_50] -max 8 [get_ports {m1_*}]
set_output_delay -clock [get_clocks clk_50] -min 2 [get_ports {m1_*}]
set_output_delay -clock [get_clocks clk_50] -max 8 [get_ports {m2_*}]
set_output_delay -clock [get_clocks clk_50] -min 2 [get_ports {m2_*}]
set_output_delay -clock [get_clocks clk_50] -max 8 [get_ports {m3_*}]
set_output_delay -clock [get_clocks clk_50] -min 2 [get_ports {m3_*}]
set_output_delay -clock [get_clocks clk_50] -max 8 [get_ports {m4_*}]
set_output_delay -clock [get_clocks clk_50] -min 2 [get_ports {m4_*}]
set_output_delay -clock [get_clocks clk_50] -max 10 [get_ports {lcd_*}]
set_output_delay -clock [get_clocks clk_50] -min 2 [get_ports {lcd_*}]
set_output_delay -clock [get_clocks clk_50] -max 8 [get_ports {ultra_trig}]
set_output_delay -clock [get_clocks clk_50] -min 2 [get_ports {ultra_trig}]
set_output_delay -clock [get_clocks clk_50] -max 8 [get_ports {buzzer_out}]
set_output_delay -clock [get_clocks clk_50] -min 2 [get_ports {buzzer_out}]
