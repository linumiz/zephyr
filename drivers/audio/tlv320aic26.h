/*
 * Copyright (c) 2026 Linumiz
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_AUDIO_TLV320AIC26_H_
#define ZEPHYR_DRIVERS_AUDIO_TLV320AIC26_H_

#include <zephyr/sys/util.h>

#define AIC26_CMD_READ              BIT(15)
#define AIC26_CMD_WRITE             0
#define AIC26_CMD_PAGE_SHIFT        11
#define AIC26_CMD_ADDR_SHIFT        5

#define AIC26_BUILD_CMD(rw, page, addr)                                 \
	((rw) | (((page) & 0x0F) << AIC26_CMD_PAGE_SHIFT) |            \
	 (((addr) & 0x3F) << AIC26_CMD_ADDR_SHIFT))

#define AIC26_PAGE1                 1
#define AIC26_PAGE2                 2

#define AIC26_P1_REFERENCE          0x03
#define AIC26_P1_RESET              0x04

/* Reference Control (Page 1, Reg 0x03) */
#define AIC26_VREFM_BIT             BIT(4)
#define AIC26_IREFV_BIT             BIT(0)

#define AIC26_RESET_VALUE           0xBB00

/* Page 2 registers */
#define AIC26_P2_AUDIO_CTL1         0x00
#define AIC26_P2_ADC_GAIN           0x01
#define AIC26_P2_DAC_GAIN           0x02
#define AIC26_P2_POWER_CTL          0x05
#define AIC26_P2_AUDIO_CTL3         0x06
#define AIC26_P2_PLL1               0x1B
#define AIC26_P2_PLL2               0x1C

/* Audio Control 1 (Page 2, Reg 0x00) */
#define AIC26_ADCHPF_SHIFT          14
#define AIC26_ADCHPF_MASK           (0x03 << AIC26_ADCHPF_SHIFT)
#define AIC26_ADCIN_SHIFT           12
#define AIC26_ADCIN_MASK            (0x03 << AIC26_ADCIN_SHIFT)
#define AIC26_WLEN_SHIFT            10
#define AIC26_WLEN_MASK             (0x03 << AIC26_WLEN_SHIFT)
#define AIC26_DATFM_SHIFT           8
#define AIC26_DATFM_MASK            (0x03 << AIC26_DATFM_SHIFT)
#define AIC26_DACFS_SHIFT           3
#define AIC26_DACFS_MASK            (0x07 << AIC26_DACFS_SHIFT)
#define AIC26_ADCFS_SHIFT           0
#define AIC26_ADCFS_MASK            (0x07 << AIC26_ADCFS_SHIFT)

/* ADCHPF field values */
#define AIC26_HPF_DISABLED          0x00
#define AIC26_HPF_0045FS            0x01
#define AIC26_HPF_0125FS            0x02
#define AIC26_HPF_025FS             0x03

/* ADCIN field values */
#define AIC26_ADCIN_MIC             0x00
#define AIC26_ADCIN_AUX             0x01
#define AIC26_ADCIN_DIFF            0x02

#define AIC26_WLEN_16               0x00
#define AIC26_WLEN_20               0x01
#define AIC26_WLEN_24               0x02
#define AIC26_WLEN_32               0x03

#define AIC26_DATFM_I2S             0x00
#define AIC26_DATFM_DSP             0x01
#define AIC26_DATFM_RJ              0x02
#define AIC26_DATFM_LJ              0x03

/* DACFS / ADCFS divisor values */
#define AIC26_FS_DIV_1              0x00
#define AIC26_FS_DIV_1_5            0x01
#define AIC26_FS_DIV_2              0x02
#define AIC26_FS_DIV_3              0x03
#define AIC26_FS_DIV_4              0x04
#define AIC26_FS_DIV_5              0x05
#define AIC26_FS_DIV_5_5            0x06
#define AIC26_FS_DIV_6              0x07

/* ADC Gain Control (Page 2, Reg 0x01) */
#define AIC26_ADMUT_BIT             BIT(15)
#define AIC26_ADPGA_SHIFT           8
#define AIC26_ADPGA_MASK            (0x7F << AIC26_ADPGA_SHIFT)
#define AIC26_AGCTG_SHIFT           5
#define AIC26_AGCTG_MASK            (0x07 << AIC26_AGCTG_SHIFT)
#define AIC26_AGCTC_SHIFT           1
#define AIC26_AGCTC_MASK            (0x0F << AIC26_AGCTC_SHIFT)
#define AIC26_AGCEN_BIT             BIT(0)

/* AGC target level values */
#define AIC26_AGCTG_M5P5            0x00
#define AIC26_AGCTG_M8              0x01
#define AIC26_AGCTG_M10             0x02
#define AIC26_AGCTG_M12             0x03
#define AIC26_AGCTG_M14             0x04
#define AIC26_AGCTG_M17             0x05
#define AIC26_AGCTG_M20             0x06
#define AIC26_AGCTG_M24             0x07

/* AGC time constant presets (attack_ms / decay_ms) */
#define AIC26_AGCTC_8_100           0x00
#define AIC26_AGCTC_11_100          0x01
#define AIC26_AGCTC_16_100          0x02
#define AIC26_AGCTC_20_100          0x03
#define AIC26_AGCTC_8_200           0x04
#define AIC26_AGCTC_11_200          0x05
#define AIC26_AGCTC_16_200          0x06
#define AIC26_AGCTC_20_200          0x07
#define AIC26_AGCTC_8_400           0x08
#define AIC26_AGCTC_11_400          0x09
#define AIC26_AGCTC_16_400          0x0A
#define AIC26_AGCTC_20_400          0x0B
#define AIC26_AGCTC_8_500           0x0C
#define AIC26_AGCTC_11_500          0x0D
#define AIC26_AGCTC_16_500          0x0E
#define AIC26_AGCTC_20_500          0x0F

/* DAC Gain (Page 2, Reg 0x02) */
#define AIC26_DALMU_BIT             BIT(15)
#define AIC26_DALVL_SHIFT           8
#define AIC26_DALVL_MASK            (0x7F << AIC26_DALVL_SHIFT)
#define AIC26_DARMU_BIT             BIT(7)
#define AIC26_DARVL_SHIFT           0
#define AIC26_DARVL_MASK            (0x7F << AIC26_DARVL_SHIFT)

#define AIC26_DAC_MUTE_ALL          (AIC26_DALMU_BIT |                  \
				     (0x7F << AIC26_DALVL_SHIFT) |      \
				     AIC26_DARMU_BIT | 0x7F)

/* Power Control (Page 2, Reg 0x05) */
#define AIC26_PWDNC_BIT             BIT(15)
#define AIC26_DAODRC_BIT            BIT(12)
#define AIC26_DAPWDN_BIT            BIT(10)
#define AIC26_ADPWDN_BIT            BIT(9)
#define AIC26_VGPWDN_BIT            BIT(8)
#define AIC26_ADPWDF_BIT            BIT(7)
#define AIC26_DAPWDF_BIT            BIT(6)
#define AIC26_VBIAS_BIT             BIT(4)

/* Audio Control 3 (Page 2, Reg 0x06) */
#define AIC26_REFFS_BIT             BIT(13)
#define AIC26_SLVMS_BIT             BIT(11)
#define AIC26_AGCNL_SHIFT           4
#define AIC26_AGCNL_MASK            (0x03 << AIC26_AGCNL_SHIFT)
#define AIC26_CLPST_BIT             BIT(3)

/* AGC noise threshold values */
#define AIC26_AGCNL_M60             0x00
#define AIC26_AGCNL_M70             0x01
#define AIC26_AGCNL_M80             0x02
#define AIC26_AGCNL_M90             0x03

/* PLL1 (Page 2, Reg 0x1B) */
#define AIC26_PLLSEL_BIT            BIT(15)
#define AIC26_QVAL_SHIFT            11
#define AIC26_QVAL_MASK             (0x0F << AIC26_QVAL_SHIFT)
#define AIC26_PVAL_SHIFT            8
#define AIC26_PVAL_MASK             (0x07 << AIC26_PVAL_SHIFT)
#define AIC26_JVAL_SHIFT            2
#define AIC26_JVAL_MASK             (0x3F << AIC26_JVAL_SHIFT)

/* PLL2 (Page 2, Reg 0x1C) */
#define AIC26_DVAL_SHIFT            2
#define AIC26_DVAL_MASK             (0x3FFF << AIC26_DVAL_SHIFT)

/* Timing */
#define AIC26_RESET_WAIT_MS         10
#define AIC26_PLL_LOCK_MS           10
#define AIC26_DAC_SETTLE_MS         50
#define AIC26_DAC_POLL_MS           10
#define AIC26_DAC_TIMEOUT_MS        500
#define AIC26_ADC_POLL_MS           10
#define AIC26_ADC_TIMEOUT_MS        500

#endif /* ZEPHYR_DRIVERS_AUDIO_TLV320AIC26_H_ */
