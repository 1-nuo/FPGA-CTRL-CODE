# ============================================================
# timing.sdc — 时序约束文件
# 芯片：安路 EF2L45BG256B
# ============================================================

# ── 主时钟约束 ──────────────────────────────────────────────
# 假设板载晶振 50MHz，周期 20ns
create_clock -name sys_clk -period 20.000 [get_ports clk]

# ── 生成时钟约束（PLL 输出） ───────────────────────────────
# 若 PLL 输出仍为 50MHz：
create_generated_clock -name clk_sys \
    -source [get_ports clk] \
    -master_clock sys_clk \
    -divide_by 1 \
    -multiply_by 1 \
    [get_nets u_pll/clk0_out]

# ── 输入延迟 ────────────────────────────────────────────────
set_input_delay -clock sys_clk 5.000 [get_ports track_sensor*]
set_input_delay -clock sys_clk 5.000 [get_ports ultra_echo]

# ── 输出延迟 ────────────────────────────────────────────────
set_output_delay -clock sys_clk 5.000 [get_ports m1_in1]
set_output_delay -clock sys_clk 5.000 [get_ports m1_in2]
set_output_delay -clock sys_clk 5.000 [get_ports m2_in3]
set_output_delay -clock sys_clk 5.000 [get_ports m2_in4]
set_output_delay -clock sys_clk 5.000 [get_ports m3_in1]
set_output_delay -clock sys_clk 5.000 [get_ports m3_in2]
set_output_delay -clock sys_clk 5.000 [get_ports m4_in3]
set_output_delay -clock sys_clk 5.000 [get_ports m4_in4]
set_output_delay -clock sys_clk 5.000 [get_ports lcd_rs]
set_output_delay -clock sys_clk 5.000 [get_ports lcd_en]
set_output_delay -clock sys_clk 5.000 [get_ports lcd_data*]
set_output_delay -clock sys_clk 5.000 [get_ports ultra_trig]
set_output_delay -clock sys_clk 5.000 [get_ports buzzer_out]
