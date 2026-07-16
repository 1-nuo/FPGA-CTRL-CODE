# FPGA 智能小车控制代码

## 项目说明

安路 EF2L45BG256B FPGA 控制代码，用于暑期实训智能小车项目。

## 文件夹结构

```
uisrc/
├── 01_rtl/       # Verilog 源文件
├── 02_sim/       # 仿真测试文件
├── 03_ip/        # IP 核（PLL 等）
├── 04_pin/       # 引脚约束(.adc) + 时序约束(.sdc)
├── 05_boot/      # 编译产物(.bit/.bin)
└── 06_doc/       # 文档
fpga_prj/          # TD 工程文件夹
.vscode/           # VSCode 配置
```

## 开发工具

- 语言：Verilog-2001
- IDE：TangDynasty (TD) 5.6.2
- 仿真：ModelSim
- 芯片：EF2L45BG256B
