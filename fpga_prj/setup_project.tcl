# ============================================================
# Quartus Prime Lite - DE1-SoC 工程自动化创建脚本
# 使用方式：VSCode → Ctrl+Shift+B → "Quartus: 创建工程"
# 或在命令行：quartus_sh -t setup_project.tcl
# ============================================================

package require ::quartus::project
load_package project

# ---------- 工程基本信息 ----------
set project_name "de1_soc_test"
set script_dir [file dirname [info script]]
# 如果脚本从绝对路径调用，取绝对路径；否则取当前目录
if {[string equal ${script_dir} "."]} {
    set script_dir [pwd]
}
# 工作区根目录 = fpga_prj 的父目录
set workspace_dir [file dirname ${script_dir}]

project_new ${script_dir}/${project_name} -overwrite

# ---------- 顶层实体 ----------
set_global_assignment -name TOP_LEVEL_ENTITY top

# ---------- 器件选择 ----------
# DE1-SoC: Cyclone V 5CSEMA5F31C6
set_global_assignment -name FAMILY "Cyclone V"
set_global_assignment -name DEVICE 5CSEMA5F31C6

# ---------- 源文件路径 ----------
set src_dir [file join ${workspace_dir} uisrc 01_rtl]
set sim_dir [file join ${workspace_dir} uisrc 02_sim]

# ---------- 添加 RTL 源文件 ----------
set rtl_files [list \
    "top.v" \
    "pll_bypass.v" \
    "main_controller.v" \
    "clk_divider.v" \
    "line_tracking.v" \
    "motor_control.v" \
    "ultrasonic_driver.v" \
    "buzzer_ctrl.v" \
    "lcd_driver.v" \
    "dht11_driver.v" \
]

foreach f ${rtl_files} {
    set filepath [file join ${src_dir} ${f}]
    if {[file exists ${filepath}]} {
        set_global_assignment -name VERILOG_FILE ${filepath}
        puts "  + 添加源文件: ${f}"
    } else {
        puts "  - 跳过（未找到）: ${f}"
    }
}

# ---------- PLL IP 文件 ----------
set pll_ip_dir [file join ${workspace_dir} uisrc 03_ip pll]
if {[file exists [file join ${pll_ip_dir} pll.v]]} {
    set_global_assignment -name VERILOG_FILE [file join ${pll_ip_dir} pll.v]
    puts "  + 添加 PLL IP: pll.v"
}
if {[file exists [file join ${pll_ip_dir} pll.qip]]} {
    set_global_assignment -name QIP_FILE [file join ${pll_ip_dir} pll.qip]
    puts "  + 添加 PLL QIP: pll.qip"
}

# ---------- 综合选项 ----------
set_global_assignment -name NUM_PARALLEL_PROCESSORS ALL
set_global_assignment -name STRATIX_DEVICE_IO_STANDARD "3.3-V LVTTL"

# ---------- 时序约束 ----------
set sdc_file [file join ${script_dir} de1_soc_test.sdc]
if {[file exists ${sdc_file}]} {
    set_global_assignment -name SDC_FILE ${sdc_file}
    puts "  + 添加时序约束: de1_soc_test.sdc"
}

# ---------- 引脚约束 ----------
set pin_file [file join ${workspace_dir} uisrc 04_pin de1_soc_pins.qsf]
if {[file exists ${pin_file}]} {
    source ${pin_file}
    puts "  + 加载引脚约束: de1_soc_pins.qsf"
}

# ---------- 保存并关闭工程 ----------
project_close

puts ""
puts "============================================"
puts "工程创建完成！"
puts "  位置: ${script_dir}/${project_name}.qpf"
puts "  器件: 5CSEMA5F31C6 (Cyclone V, DE1-SoC)"
puts "  顶层: top"
puts "  源文件: ${src_dir}"
puts "============================================"
puts ""
puts "下一步：Ctrl+Shift+B → 选择「全流程编译」"
puts "============================================"
