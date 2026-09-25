/*
 * Copyright (c) 2021, Weidmueller Interface GmbH & Co. KG
 * Copyright (c) 2026 Linumiz
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT xlnx_gem

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>

#include "eth_cdns_gem_priv.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(eth_cdns_gem, CONFIG_ETHERNET_LOG_LEVEL);

#if defined(CONFIG_SOC_FAMILY_XILINX_ZYNQ7000)
/*
 * Zynq-7000 TX clock configuration:
 *
 * GEMx_CLK_CTRL (SLCR) registers:
 * [25 .. 20] Reference clock divisor 1
 * [13 .. 08] Reference clock divisor 0
 * [00]       Clock active bit
 */
#define ETH_XLNX_SLCR_GEMX_CLK_CTRL_DIVISOR_MASK	0x0000003F
#define ETH_XLNX_SLCR_GEMX_CLK_CTRL_DIVISOR1_SHIFT	20
#define ETH_XLNX_SLCR_GEMX_CLK_CTRL_DIVISOR0_SHIFT	8
#elif defined(CONFIG_SOC_XILINX_ZYNQMP)
/*
 * UltraScale TX clock configuration: comp.
 * https://www.xilinx.com/html_docs/registers/ug1087/ug1087-zynq-ultrascale-registers.html
 *
 * CRL_WPROT (CRL_APB) register:
 * [00] CRL APB register space write protection bit
 *
 * GEMx_REF_CTRL (CRL_APB) registers:
 * [30]       RX channel clock active bit
 * [29]       Clock active bit
 * [21 .. 16] Reference clock divisor 1
 * [13 .. 08] Reference clock divisor 0
 */
#define ETH_XLNX_CRL_APB_WPROT_REGISTER_ADDRESS		0xFF5E001C
#define ETH_XLNX_CRL_APB_WPROT_BIT			0x00000001
#define ETH_XLNX_CRL_APB_GEMX_REF_CTRL_DIVISOR_MASK	0x0000003F
#define ETH_XLNX_CRL_APB_GEMX_REF_CTRL_DIVISOR1_SHIFT	16
#define ETH_XLNX_CRL_APB_GEMX_REF_CTRL_DIVISOR0_SHIFT	8
#define ETH_XLNX_CRL_APB_GEMX_REF_CTRL_RX_CLKACT_BIT    0x04000000
#define ETH_XLNX_CRL_APB_GEMX_REF_CTRL_CLKACT_BIT       0x02000000
#endif /* CONFIG_SOC_FAMILY_XILINX_ZYNQ7000 / CONFIG_SOC_XILINX_ZYNQMP */

struct eth_xlnx_gem_platform_cfg {
	uintptr_t clkc_phys;
	uint32_t pll_clock_frequency;
	mm_reg_t *clkc;
};

static inline const struct eth_xlnx_gem_platform_cfg *
eth_xlnx_gem_get_platform_cfg(const struct device *dev)
{
	return ((const struct eth_cdns_gem_dev_cfg *)dev->config)->platform_cfg;
}

int eth_cdns_gem_platform_init(const struct device *dev)
{
	const struct eth_xlnx_gem_platform_cfg *plat =
		eth_xlnx_gem_get_platform_cfg(dev);

#ifdef DEVICE_MMIO_IS_IN_RAM
	device_map(plat->clkc, plat->clkc_phys, sizeof(uint32_t),
		   K_MEM_CACHE_NONE);
#else
	*plat->clkc = plat->clkc_phys;
#endif

	return 0;
}

/**
 * @brief GEM clock configuration function
 * Calculates the pre-scalers for the TX clock to match the current
 * (if an associated PHY is managed) or nominal link speed. Called
 * from within the device initialization function.
 *
 * @param dev Pointer to the device data
 * @param state pointer to the current PHY link state data
 */
void eth_cdns_gem_set_link_speed(const struct device *dev,
				 struct phy_link_state *state)
{
	/*
	 * Clock source configuration for the respective GEM as described
	 * in the Zynq-7000 TRM, chapter 16.3.3, is not tackled here. This
	 * is performed by the PS7Init code. Only the DIVISOR and DIVISOR1
	 * values for the respective GEM's TX clock are calculated here.
	 */

	const struct eth_xlnx_gem_platform_cfg *plat =
		eth_xlnx_gem_get_platform_cfg(dev);
	uint32_t div0;
	uint32_t div1;
	uint32_t target = 2500000; /* default prevents 'may be uninitialized' warning */
	uint32_t tmp;
	uint32_t clk_ctrl_reg;

	if (PHY_LINK_IS_SPEED_1000M(state->speed)) {
		target = 125000000; /* Target frequency: 125 MHz */
	} else if (PHY_LINK_IS_SPEED_100M(state->speed)) {
		target = 25000000;  /* Target frequency: 25 MHz */
	} else {
		target = 2500000;   /* Target frequency: 2.5 MHz */
	}

	/*
	 * Calculate the divisors for the target frequency.
	 * The frequency of the PLL to which the divisors shall be applied are
	 * provided in the respective GEM's device tree data.
	 */
	for (div0 = 1; div0 < 64; div0++) {
		for (div1 = 1; div1 < 64; div1++) {
			tmp = ((plat->pll_clock_frequency / div0) / div1);
			if (tmp >= (target - 10) && tmp <= (target + 10)) {
				break;
			}
		}
		if (tmp >= (target - 10) && tmp <= (target + 10)) {
			break;
		}
	}

#if defined(CONFIG_SOC_XILINX_ZYNQMP)
	/*
	 * ZynqMP register crl_apb.GEMx_REF_CTRL:
	 * RX_CLKACT bit [26]
	 * CLKACT bit [25]
	 * div0 bits [13..8], div1 bits [21..16]
	 * Unlock CRL_APB write access if the write protect bit
	 * is currently set, restore it afterwards.
	 */
	clk_ctrl_reg  = sys_read32(*plat->clkc);
	clk_ctrl_reg &= ~((ETH_XLNX_CRL_APB_GEMX_REF_CTRL_DIVISOR_MASK <<
			ETH_XLNX_CRL_APB_GEMX_REF_CTRL_DIVISOR0_SHIFT) |
			(ETH_XLNX_CRL_APB_GEMX_REF_CTRL_DIVISOR_MASK <<
			ETH_XLNX_CRL_APB_GEMX_REF_CTRL_DIVISOR1_SHIFT));
	clk_ctrl_reg |=	((div0 & ETH_XLNX_CRL_APB_GEMX_REF_CTRL_DIVISOR_MASK) <<
			ETH_XLNX_CRL_APB_GEMX_REF_CTRL_DIVISOR0_SHIFT) |
			((div1 & ETH_XLNX_CRL_APB_GEMX_REF_CTRL_DIVISOR_MASK) <<
			ETH_XLNX_CRL_APB_GEMX_REF_CTRL_DIVISOR1_SHIFT);
	clk_ctrl_reg |=	ETH_XLNX_CRL_APB_GEMX_REF_CTRL_RX_CLKACT_BIT |
			ETH_XLNX_CRL_APB_GEMX_REF_CTRL_CLKACT_BIT;

	/*
	 * Unlock CRL_APB write access if the write protect bit
	 * is currently set, restore it afterwards.
	 */
	tmp = sys_read32(ETH_XLNX_CRL_APB_WPROT_REGISTER_ADDRESS);
	if ((tmp & ETH_XLNX_CRL_APB_WPROT_BIT) > 0) {
		sys_write32((tmp & ~ETH_XLNX_CRL_APB_WPROT_BIT),
			    ETH_XLNX_CRL_APB_WPROT_REGISTER_ADDRESS);
	}
	sys_write32(clk_ctrl_reg, *plat->clkc);
	if ((tmp & ETH_XLNX_CRL_APB_WPROT_BIT) > 0) {
		sys_write32(tmp, ETH_XLNX_CRL_APB_WPROT_REGISTER_ADDRESS);
	}
#elif defined(CONFIG_SOC_FAMILY_XILINX_ZYNQ7000)
	clk_ctrl_reg  = sys_read32(*plat->clkc);
	clk_ctrl_reg &= ~((ETH_XLNX_SLCR_GEMX_CLK_CTRL_DIVISOR_MASK <<
			ETH_XLNX_SLCR_GEMX_CLK_CTRL_DIVISOR0_SHIFT) |
			(ETH_XLNX_SLCR_GEMX_CLK_CTRL_DIVISOR_MASK <<
			ETH_XLNX_SLCR_GEMX_CLK_CTRL_DIVISOR1_SHIFT));
	clk_ctrl_reg |= ((div0 & ETH_XLNX_SLCR_GEMX_CLK_CTRL_DIVISOR_MASK) <<
			ETH_XLNX_SLCR_GEMX_CLK_CTRL_DIVISOR0_SHIFT) |
			((div1 & ETH_XLNX_SLCR_GEMX_CLK_CTRL_DIVISOR_MASK) <<
			ETH_XLNX_SLCR_GEMX_CLK_CTRL_DIVISOR1_SHIFT);

	sys_write32(clk_ctrl_reg, *plat->clkc);
#endif /* CONFIG_SOC_XILINX_ZYNQMP / CONFIG_SOC_FAMILY_XILINX_ZYNQ7000 */

	LOG_DBG("%s set clock dividers div0/1 %u/%u for target "
		"frequency %u Hz", dev->name, div0, div1, target);
}

#define ETH_XLNX_GEM_PLATFORM_CFG_DEFINE(port)						\
static mm_reg_t eth_xlnx_gem##port##_clkc;						\
static const struct eth_xlnx_gem_platform_cfg eth_cdns_gem##port##_platform_cfg = {	\
	.clkc_phys = DT_INST_REG_ADDR_BY_NAME(port, clkc),				\
	.pll_clock_frequency = DT_INST_PROP(port, clock_frequency),			\
	.clkc = &eth_xlnx_gem##port##_clkc,						\
};

DT_INST_FOREACH_STATUS_OKAY(ETH_XLNX_GEM_PLATFORM_CFG_DEFINE)

DT_INST_FOREACH_STATUS_OKAY(ETH_CDNS_GEM_INITIALIZE)

/*
 * If dcache is enabled: RX/TX buffer sizes of all enabled instances
 * must be a multiple of the cache line size
 */

#if CONFIG_QEMU_TARGET ||\
	DT_ANY_INST_HAS_BOOL_STATUS_OKAY(disable_rx_checksum_offload) ||\
	DT_ANY_INST_HAS_BOOL_STATUS_OKAY(disable_tx_checksum_offload)
#warning "cdns_gem: at least one instance has checksum offloading to hardware disabled"
#endif

#ifdef CONFIG_DCACHE

#define ETH_XLNX_GEM_BUFFER_SIZE_CHECK(port) \
BUILD_ASSERT((DT_INST_PROP(port, rx_buffer_size) % CONFIG_DCACHE_LINE_SIZE) == 0,\
	     "RX buffer size is not a multiple of the dcache line size for GEM "\
	     "instance " #port);\
BUILD_ASSERT((DT_INST_PROP(port, tx_buffer_size) % CONFIG_DCACHE_LINE_SIZE) == 0,\
	     "TX buffer size is not a multiple of the dcache line size for GEM "\
	     "instance " #port);

DT_INST_FOREACH_STATUS_OKAY(ETH_XLNX_GEM_BUFFER_SIZE_CHECK)

#endif /* CONFIG_DCACHE */
