# FPGA 智能小车控制系统

基于 **DE1-SoC (Cyclone V)** / **安路 EF2L45BG256** 的智能小车 FPGA 控制实现，支持循迹行驶、超声波避障、温湿度检测与 LCD 实时显示。

---

## 目录

- [项目概述](#项目概述)
- [硬件平台](#硬件平台)
- [系统架构](#系统架构)
- [模块详解](#模块详解)
- [引脚映射](#引脚映射)
- [开发环境配置](#开发环境配置)
- [快速开始](#快速开始)
- [仿真验证](#仿真验证)
- [文件结构](#文件结构)
- [License](#license)

---

## 项目概述

本项目是电子工艺实习 FPGA 方向的智能小车控制系统。系统以 FPGA 为核心控制器，配合 TCRT5000 循迹传感器、HC-SR04 超声波模块、DHT11 温湿度传感器、L298N 电机驱动和 LCD1602 显示屏，实现一辆具备自主循迹、障碍物检测与规避、环境温湿度检测与显示功能的四驱智能小车。

### 核心功能

| 功能 | 实现方案 | 状态 |
|:-----|:---------|:----:|
| 循迹行驶 | 6 路 TCRT5000 传感器 + 加权质心算法 + PD 差速控制 | ✅ |
| 超声波避障 | HC-SR04 测距 + 蜂鸣器多级报警 | ✅ |
| 温湿度检测 | DHT11 单总线协议采集 | ✅ |
| LCD 实时显示 | LCD1602 4 位模式显示温湿度 + 学号 | ✅ |
| 四轮差速驱动 | 2×L298N 驱动 4 路直流电机 | ✅ |

---

## 硬件平台

### 目标芯片

| 参数 | 值 |
|:-----|:----|
| 芯片型号 | Cyclone V 5CSEMA5F31C6 (DE1-SoC) / EF2L45BG256 (安路) |
| 逻辑单元 | 85K LE (Cyclone V) / 45K LE (安路) |
| 片上内存 | 4,450 Kbits (Cyclone V) / 1,728 Kbits (安路) |
| 工作频率 | 50 MHz（板载晶振） |

### 外设清单

| 外设 | 型号 | 数量 | 接口 |
|:-----|:-----|:----:|:-----|
| 循迹传感器 | TCRT5000 (红外反射式) | 6 | GPIO 数字输入 |
| 超声波模块 | HC-SR04 | 1 | GPIO (TRIG + ECHO) |
| 温湿度传感器 | DHT11 | 1 | 单总线 (GPIO) |
| 液晶显示屏 | LCD1602 (HD44780) | 1 | GPIO 4 位并行 |
| 电机驱动模块 | L298N | 2 | GPIO (PWM + 方向) |
| 直流减速电机 | — | 4 | L298N 输出 |
| 电平转换 | 74HCT245D | 1 | 3.3V → 5V (使能信号) |
| 蜂鸣器 | 有源 5V | 1 | 三极管 S8050 驱动 |

---

## 系统架构

### 模块层级图

```
┌─────────────────────────────────────────────────────────────┐
│                          top.v                              │
│  (顶层模块 — 例化所有子模块、信号连线)                       │
└──────────────────────────┬──────────────────────────────────┘
                           │
           ┌───────────────┼───────────────┐
           │               │               │
           ▼               ▼               ▼
   ┌───────────────┐ ┌───────────┐ ┌───────────────┐
   │  clk_divider   │ │ pll_bypass│ │ main_controller│
   │  50MHz→tick   │ │ 时钟透传  │ │ 主状态机调度   │
   └───────┬───────┘ └───────────┘ └───────┬───────┘
           │  tick_1MHz                     │
           │  tick_10kHz                    │
           │  tick_1kHz    ┌────────────────┼────────────────┐
           │  tick_1Hz     │                │                │
           ▼               ▼                ▼                ▼
   ┌───────────────┐ ┌───────────┐ ┌──────────────┐ ┌──────────────┐
   │ line_tracking │ │motor_ctrl │ │ultrasonic_drv│ │ dht11_driver │
   │ 6路循迹→位置  │ │PD差速→PWM │ │HC-SR04→距离  │ │单总线→温湿度 │
   └───────┬───────┘ └─────┬─────┘ └──────┬───────┘ └──────┬───────┘
           │               │              │                │
           └───────┬───────┘              │                │
                   │             ┌────────┴────────┐       │
                   ▼             ▼                 ▼       ▼
           ┌───────────────┐ ┌───────────┐ ┌───────────────────┐
           │ L298N × 2     │ │buzzer_ctrl│ │   lcd_driver      │
           │ 四路电机      │ │蜂鸣器报警 │ │ LCD1602 4位驱动   │
           └───────────────┘ └───────────┘ └───────────────────┘
```

### 系统数据流

```
TCRT5000(6路) ──→ line_tracking ──→ main_controller ──→ motor_control ──→ L298N ──→ 电机
                    (加权质心)        │   (状态机调度)      (PD差速+PWM)
                                     │
HC-SR04 ──→ ultrasonic_driver ──────┤
DHT11   ──→ dht11_driver ───────────┤
                                     └──→ lcd_driver ──→ LCD1602
                                     └──→ buzzer_ctrl ──→ 蜂鸣器
```

### 时钟域设计

系统采用**单一时钟域 + 时钟使能**策略，避免多时钟域带来的跨时钟域同步问题：

```
50MHz (板载晶振)
  │
  ├──→ clk_divider ──→ tick_1MHz (1μs周期)  → ultrasonic_driver (HC-SR04 时序)
  │                   ├── tick_10kHz (100μs) → motor_control (PWM 载波 + PD 计算)
  │                   ├── tick_1kHz  (1ms)   → lcd_driver, main_controller
  │                   └── tick_1Hz   (1s)    → main_controller (LCD 刷新, DHT11 周期)
  │
  └──→ 各模块 always @(posedge clk) 直接使用 50MHz 时钟
```

### 主状态机 (`main_controller`)

```
        ┌──────────┐
        │  S_INIT  │  ← 上电等待 LCD 初始化完成
        └────┬─────┘
             │ (lcd_busy = 0)
             ▼
        ┌──────────┐
   ┌───→│  S_IDLE  │  ← 空闲等待，循环触发各外设
   │    └────┬─────┘
   │         │
   │    ┌────┴────────────────────────┐
   │    │  ultra_timer ≥ 200ms        │
   │    ├──→ S_ULTRA_TRIG → 发 TRIG   │
   │    │    S_ULTRA_WAIT → 等 ECHO   │
   │    │    └─→ 存距离 → ultra_pending=0
   │    │                              │
   │    │  dht_timer ≥ 1s              │
   │    ├──→ S_DHT_TRIG → 启动读取     │
   │    │    S_DHT_WAIT → 等 DHT11     │
   │    │    └─→ 存温湿度 → dht_pending=0
   │    │                              │
   │    │  连续                        │
   │    ├──→ 转发循迹位置 → motor      │
   │    │                              │
   │    │  lcd_req = 1                 │
   │    └──→ S_LCD_WRITE → 写数据/指令 │
   │         (32 字节, 逐字节写入)      │
   │         └─→ 回到 S_IDLE           │
   └───────────────────────────────────┘
```

---

## 模块详解

### `top.v` — 顶层模块

将各子模块例化并连线，映射 GPIO 引脚到顶层端口。

- 输入：`clk`, `rst_n`, `track_sensor[5:0]`, `ultra_echo`, `dht_data`
- 输出：`m1_in1~m4_in4` (电机), `lcd_rs/en/data`, `ultra_trig`, `buzzer_out`, `en_u1~u2` (使能)
- 调试：`led_test` (LEDR0, 1Hz 心跳), `led_dbg2` (LEDR1, LCD 初始化标志)

### `clk_divider.v` — 时钟分频模块

级联计数器结构，逐级产生各频率时钟使能信号，避免使用取模运算和硬件除法器。

| 输入频率 | 分频系数 | 输出 tick | 用途 |
|:---------|:--------|:----------|:-----|
| 50 MHz | ÷50 | **1 MHz** (1μs) | 超声波时序 |
| 1 MHz | ÷100 | **10 kHz** (100μs) | PWM 载波 |
| 10 kHz | ÷10 | **1 kHz** (1ms) | LCD 驱动、主控调度 |
| 1 kHz | ÷1000 | **1 Hz** (1s) | LCD 刷新、DHT11 周期 |

### `line_tracking.v` — 循迹位置计算模块

**加权质心算法**：将 6 个传感器按物理位置分配权重，计算黑线位置的加权质心。

```
权重分配: S1=1  S2=3  S3=6  S4=10  S5=13  S6=15
         (最左)                              (最右)

position = (Σ 检测到黑线的传感器 × 权重) / (Σ 检测到黑线的传感器数)

输出: position(0~15, 7=居中), line_lost(脱线标志)
```

### `motor_control.v` — 电机驱动与控制模块

**PD 差速控制**，含 PWM 发生器和输出斜率限制器。

- **PWM 发生器**：13 位计数器 (0~4999)，10kHz 载波
- **PD 控制器**：`control_term = error_now × KP + error_delta × KD`
- **死区**：position 在 6~8 范围时不调节，避免直道微调摆动
- **斜率限制**：速度变化率 ≤4/周期，防止急加速/减速
- **四种运行模式**：STOP / FORWARD / BACKWARD / TURN

| 参数 | 值 | 说明 |
|:-----|:---|:-----|
| `base_speed` | 128/255 | 基础占空比 ~50% |
| `KP` | 16 | 比例系数 |
| `KD` | 4 | 微分系数 |
| `PWM_PERIOD` | 5000 | PWM 周期计数 |

控制策略：脱线时立即停车；偏左则减速左轮、维持右轮；偏右则减速右轮、维持左轮。

### `ultrasonic_driver.v` — 超声波测距模块

HC-SR04 时序控制，含超时保护。

- 发送 10μs TRIG 脉冲 → 等待 ECHO 上升沿 → 测量 ECHO 脉宽
- 距离公式：`distance(cm) = echo_width(μs) / 58`
- 超时保护：WAIT_ECHO 30ms 超时，WAIT_MEASURE 50ms 超时

### `dht11_driver.v` — 温湿度采集模块

完整 DHT11 单总线协议实现，采用**边沿检测 + 定时采样**策略。

- 主机拉低 ≥18ms 启动 → 等待 DHT11 响应 → 读取 40 位数据
- bit 判定：拉高后 40μs 采样总线电平（0=short, 1=long）
- 超时保护：每步 200~500μs 超时恢复
- 数据过滤：湿度 ≤90%、温度 ≤60℃、小数位 = 0

### `lcd_driver.v` — LCD1602 显示驱动

HD44780 4 位模式驱动，完整 8 条初始化序列。

- 初始化：Function Set(0x28) → Display On(0x0C) → Clear(0x01) → Entry Mode(0x06)
- 写操作：RS 选择指令/数据 → 发高 4 位 + E 下降沿锁存 → 发低 4 位 + E 下降沿锁存
- **cmd_valid 握手**：每个 50MHz 周期高速采样 cmd_valid，不依赖 tick_1kHz
- 参数化：`INIT_WAIT_MS`（仿真=1, 实物=40）

### `buzzer_ctrl.v` — 蜂鸣器控制

根据超声波距离输出不同频率的 PWM 信号驱动蜂鸣器。

| 距离 | 蜂鸣器行为 |
|:-----|:----------|
| ≥20cm | 不响 |
| 10~20cm | 4Hz 间歇响 |
| <10cm | 连续响 |

### `main_controller.v` — 主控制器状态机

7 状态 FSM 协调所有外设工作节奏（状态图见 [系统架构](#系统架构) 章节）。

- **LCD 显示内容**（32 字节）：

```
第1行 (DDRAM 0x00): T:+28C H:65%  (实时温湿度)
第2行 (DDRAM 0x40): 2024111622 FYN (学号+姓名首字母)
```

### `pll_bypass.v` — PLL 旁路模块（测试用）

DE1-SoC 测试时直接透传时钟，替代安路 PLL IP 核，避免 Quartus 编译依赖安路 IP。

---

## 引脚映射

### DE1-SoC GPIO_0 接口 (JP1)

| 功能 | 信号名 | GPIO_0 | FPGA 引脚 | JP1 脚 |
|:-----|:-------|:------:|:---------:|:------:|
| 循迹 S1 (最左) | `track_sensor[0]` | D0 | PIN_AC18 | 1 |
| 循迹 S2 | `track_sensor[1]` | D1 | PIN_AE19 | 2 |
| 循迹 S3 | `track_sensor[2]` | D2 | PIN_AG16 | 3 |
| 循迹 S4 | `track_sensor[3]` | D3 | PIN_AF17 | 4 |
| 循迹 S5 | `track_sensor[4]` | D4 | PIN_AF16 | 5 |
| 循迹 S6 (最右) | `track_sensor[5]` | D5 | PIN_AG15 | 6 |
| 左前 PWM | `m1_in1` | D6 | PIN_AF15 | 7 |
| 左前 反转 | `m1_in2` | D7 | PIN_AE16 | 8 |
| 右前 反转 | `m2_in3` | D8 | PIN_AE15 | 9 |
| 右前 PWM | `m2_in4` | D9 | PIN_AF18 | 10 |
| 左后 反转 | `m3_in2` | D10 | PIN_AG12 | 13 |
| 左后 PWM | `m3_in1` | D11 | PIN_AJ12 | 14 |
| 右后 反转 | `m4_in3` | D12 | PIN_AJ11 | 15 |
| 右后 PWM | `m4_in4` | D13 | PIN_AH12 | 16 |
| LCD EN | `lcd_en` | D14 | PIN_AE17 | 17 |
| LCD RS | `lcd_rs` | D15 | PIN_AA18 | 18 |
| LCD DB7 | `lcd_data[3]` | D16 | PIN_AH20 | 19 |
| LCD DB6 | `lcd_data[2]` | D17 | PIN_AJ20 | 20 |
| 超声 TRIG | `ultra_trig` | D18 | PIN_AF19 | 21 |
| 超声 ECHO | `ultra_echo` | D19 | PIN_AC20 | 22 |
| 蜂鸣器 | `buzzer_out` | D20 | PIN_AH19 | 23 |
| DHT11 DATA | `dht_data` | D21 | PIN_AJ19 | 24 |
| U1 EN_A | `en_u1_ena` | D24 | PIN_AG11 | 27 |
| U1 EN_B | `en_u1_enb` | D25 | PIN_AJ10 | 28 |
| U2 EN_A | `en_u2_ena` | D26 | PIN_AF11 | 31 |
| U2 EN_B | `en_u2_enb` | D27 | PIN_AE12 | 32 |

> **时钟**: PIN_AF14 (CLK_50, 板载 50MHz 晶振, 50Ω 端接)  
> **复位**: PIN_AA14 (KEY[0], 低有效)

### 安路 EF2L45BG256 引脚

参见 `uisrc/04_pin/fpga_pin.adc` 文件。

---

## 开发环境配置

### 工具链

| 工具 | 版本 | 用途 |
|:-----|:-----|:-----|
| Quartus Prime Lite | 18.1.0 Build 625 | DE1-SoC 全流程编译 |
| ModelSim (Quartus 内嵌) | 10.5b | RTL 仿真 |
| TangDynasty | 5.6.2+ | 安路芯片编译 |
| VSCode | — | 代码编辑 |
| TerosHDL | 7.0.3 | Verilog 格式化 + 语法检查 |

### VSCode 快捷键

VSCode Tasks 配置在 `.vscode/tasks.json`，快捷键在 `.vscode/keybindings.json`：

| 快捷键 | 功能 | 说明 |
|:------|:-----|:-----|
| **Ctrl+Shift+B** | 全流程编译 | 综合→布局布线→生成 .sof |
| **Ctrl+Shift+S** | 仅综合 | 快速语法检查 |
| **Ctrl+Shift+D** | 下载到板子 | 烧录 .sof 到 DE1-SoC @2 |

### 首次使用

```bash
# 1. 创建 Quartus 工程（仅首次）
Ctrl+Shift+B → 选择 "① 创建工程 (一次)"

# 2. 全流程编译
Ctrl+Shift+B

# 3. 连接 USB-Blaster 后下载
Ctrl+Shift+D
```

---

## 快速开始

### 系统接线

```
DE1-SoC GPIO_0 (JP1)
      │
      ├──→ TCRT5000 传感器模块 (Pin 1-6)
      ├──→ L298N × 2 (Pin 7-15, 27-32)
      ├──→ LCD1602 (Pin 17-20, 5V/GND)
      ├──→ HC-SR04 (Pin 21-22)
      ├──→ DHT11 (Pin 24, 需 4.7kΩ 上拉)
      └──→ 蜂鸣器驱动电路 (Pin 23)
```

### 编译烧录

```bash
# 全流程编译
quartus_cmd de1_soc_test -c de1_soc_test

# 下载到 FPGA
quartus_pgm -c "DE-SoC [USB-1]" -m jtag -o "p;fpga_prj/de1_soc_test.sof@2"
```

### 上电行为

1. 板载 LEDR0 以 **1Hz 闪烁**（代码运行指示）
2. LEDR1 ≈ 80ms 后亮起（LCD 初始化完成）
3. LCD 显示温湿度 + 学号，**每秒刷新**
4. 循迹传感器检测到黑线时电机自动差速转向
5. 超声波模块每 200ms 测距一次，蜂鸣器根据距离报警

---

## 仿真验证

### 使用 ModelSim 命令行仿真

```bash
cd C:\Users\11026\AppData\Local\Temp\opencode\sim_work
vlib work
vlog E:\ObsidianVault\01-Projects\Summer Internship\fpga-ctrl-code\uisrc\01_rtl\*.v
vlog E:\ObsidianVault\01-Projects\Summer Internship\fpga-ctrl-code\uisrc\02_sim\tb_*.v
vsim -c -do "run -all; exit" tb_模块名
```

### 仿真覆盖率

| Testbench | 被测模块 | 验证内容 | 状态 |
|:----------|:---------|:---------|:----:|
| `tb_line_tracking` | `line_tracking` | 6 路传感器→位置/脱线(4组用例) | ✅ |
| `tb_ultrasonic` | `ultrasonic_driver` | 100cm 障碍→测距准确 | ✅ |
| `tb_lcd_driver` | `lcd_driver` | 34 字节有序写入 | ✅ |
| `tb_dht11` | `dht11_driver` | 完整 40 位协议模拟 | ✅ |
| `tb_motor_control` | `motor_control` | PD 差速 + 四模式 | ✅ |
| `tb_top` | 全系统 | 模块集成 | ✅ |

---

## 文件结构

```
fpga-ctrl-code/
├── uisrc/                          # 用户源文件
│   ├── 01_rtl/                     # Verilog RTL 源文件
│   │   ├── top.v                   #   顶层模块
│   │   ├── clk_divider.v           #   时钟分频 (50MHz→1Hz)
│   │   ├── line_tracking.v         #   循迹位置计算 (加权质心)
│   │   ├── motor_control.v         #   电机驱动 (PD+PWM)
│   │   ├── ultrasonic_driver.v     #   超声波测距
│   │   ├── dht11_driver.v          #   DHT11 温湿度
│   │   ├── lcd_driver.v            #   LCD1602 驱动
│   │   ├── main_controller.v       #   主状态机
│   │   ├── buzzer_ctrl.v           #   蜂鸣器控制
│   │   └── pll_bypass.v            #   PLL 旁路 (DE1-SoC)
│   ├── 02_sim/                     # 仿真测试文件
│   │   ├── tb_line_tracking.v
│   │   ├── tb_ultrasonic.v
│   │   ├── tb_lcd_driver.v
│   │   ├── tb_dht11.v
│   │   ├── tb_motor_control.v
│   │   ├── tb_main_lcd_handshake.v
│   │   └── tb_top.v
│   ├── 03_ip/                      # IP 核文件夹
│   └── 04_pin/                     # 约束文件
│       ├── de1_soc_pins.qsf        #   DE1-SoC 引脚约束
│       ├── fpga_pin.adc            #   安路引脚约束
│       └── timing.sdc              #   时序约束
├── fpga_prj/                       # Quartus 工程文件夹
│   ├── de1_soc_test.qpf            #   工程文件
│   ├── de1_soc_test.qsf            #   设置 + 引脚约束
│   ├── de1_soc_test.sdc            #   时序约束
│   ├── setup_project.tcl           #   自动化创建工程脚本
│   └── de1_soc_test.sof            #   编译输出 (SRAM 烧录)
├── .vscode/                        # VSCode 配置
│   ├── tasks.json                  #   编译/下载任务
│   ├── keybindings.json            #   快捷键绑定
│   └── settings.json               #   TerosHDL 设置
└── README.md
```

---

## License

本项目为西南交通大学电子工艺实习实训课程设计作品。
