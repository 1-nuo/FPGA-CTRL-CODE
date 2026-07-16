// ============================================================
// pll_bypass.v — PLL 旁路模块 (DE1-SoC 测试用)
// 功能：直接透传时钟，模拟 PLL 行为
// 正式部署 EF2L45BG256 时替换为 TD 生成的 PLL IP
// ============================================================

module pll (
    input  wire  refclk,
    input  wire  reset,
    output wire  clk0_out,
    output wire  extlock
);

assign clk0_out = refclk;
assign extlock  = 1'b1;          // 始终锁定 (bypass)

endmodule
