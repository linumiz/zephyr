/*
 * Copyright (c) 2025 Linumiz GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT     infineon_serial_memory_qspi

#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "mtb_serial_memory.h"
#include "cycfg_qspi_memslot.h"

LOG_MODULE_REGISTER(flash_infineon_cat1, CONFIG_FLASH_LOG_LEVEL);

/* A timeout in microseconds for blocking APIs in use */
#define SMIF_INIT_TIMEOUT       (100000UL)

/* Device config structure */
struct ifx_serial_memory_flash_config {
	SMIF_CORE_Type *base;
	const cy_stc_smif_config_t *config;
	uint32_t base_addr;
	uint32_t max_addr;
#if CONFIG_FLASH_PAGE_LAYOUT
	struct flash_pages_layout layout;
#endif
	struct flash_parameters params;
	const mtb_hal_clock_t *clock;
	const struct pinctrl_dev_config *pcfg;
	const cy_stc_smif_block_config_t *smif_block_cfg;
};

/* Data structure */
struct ifx_serial_memory_flash_data {
	struct k_sem sem;
	cy_stc_smif_context_t smif_context;
	mtb_serial_memory_t serial_memory_obj;
};

static inline void ifx_serial_memory_sem_take(const struct device *dev)
{
	struct ifx_serial_memory_flash_data *data = dev->data;

	k_sem_take(&data->sem, K_FOREVER);
}

static inline void ifx_serial_memory_sem_give(const struct device *dev)
{
	struct ifx_serial_memory_flash_data *data = dev->data;

	k_sem_give(&data->sem);
}

static int ifx_serial_memory_flash_read(const struct device *dev, off_t offset, void *data,
					size_t data_len)
{
	struct ifx_serial_memory_flash_data *dev_data = dev->data;
        cy_rslt_t rslt = CY_RSLT_SUCCESS;
	int ret = 0;

	if (!data_len) {
		return 0;
	}

	ifx_serial_memory_sem_take(dev);

	rslt = mtb_serial_memory_read(&dev_data->serial_memory_obj, offset, data_len, data);
	if (rslt != CY_RSLT_SUCCESS) {
		LOG_ERR("Error reading @ %lu (Err:0x%x)", offset, rslt);
		ret = -EIO;
	}

	ifx_serial_memory_sem_give(dev);

	return ret;
}

static int ifx_serial_memory_flash_write(const struct device *dev, off_t offset, const void *data,
					 size_t data_len)
{
	struct ifx_serial_memory_flash_data *dev_data = dev->data;
	cy_rslt_t rslt = CY_RSLT_SUCCESS;
	int ret = 0;

	if (data_len == 0) {
		return 0;
	}

	if (offset < 0) {
		return -EINVAL;
	}

	ifx_serial_memory_sem_take(dev);

	rslt = mtb_serial_memory_write(&dev_data->serial_memory_obj, offset, data_len, data);
	if (rslt != CY_RSLT_SUCCESS) {
		LOG_ERR("Error in writing @ %lu (Err:0x%x)", offset, rslt);
		ret = -EIO;
	}

	ifx_serial_memory_sem_give(dev);

	return ret;
}

static int ifx_serial_memory_flash_erase(const struct device *dev, off_t offset, size_t size)
{
	struct ifx_serial_memory_flash_data *data = dev->data;
	cy_rslt_t rslt;
	int ret = 0;

	if (offset < 0) {
		return -EINVAL;
	}

	ifx_serial_memory_sem_take(dev);

	rslt = mtb_serial_memory_erase(&data->serial_memory_obj, offset, size);
	if (rslt != CY_RSLT_SUCCESS) {
		LOG_ERR("Error in erasing : 0x%x", rslt);
		ret = -EIO;
	}

	ifx_serial_memory_sem_give(dev);

	return ret;
}

static const struct flash_parameters *
ifx_serial_memory_flash_get_parameters(const struct device *dev)
{
	const struct ifx_serial_memory_flash_config *cfg = dev->config;

	return &cfg->params;
}

#if CONFIG_FLASH_PAGE_LAYOUT
static void ifx_serial_memory_flash_page_layout(const struct device *dev,
						const struct flash_pages_layout **layout,
						size_t *layout_size)
{
	const struct ifx_serial_memory_flash_config *cfg = dev->config;

	*layout = &cfg->layout;
	*layout_size = 1;
}
#endif

static int ifx_serial_memory_flash_init(const struct device *dev)
{
	struct ifx_serial_memory_flash_data *data = dev->data;
	const struct ifx_serial_memory_flash_config *cfg = dev->config;
	int ret;
	cy_rslt_t result;

	SystemCoreClockUpdate();

	/* Configure dt provided device signals when available */
	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		return ret;
	}

	result = Cy_SMIF_Init(cfg->base, cfg->config, SMIF_INIT_TIMEOUT, &data->smif_context);
	if (result != CY_SMIF_SUCCESS)
	{
	      LOG_ERR("SMIF Init Failed (result: 0x%x)\n", result);
	      return -EIO;
	}

    	/* Enables the operation of the SMIF */
    	Cy_SMIF_Enable(cfg->base, &data->smif_context);

	/* Set-up serial memory. */
	result = mtb_serial_memory_setup(&data->serial_memory_obj, MTB_SERIAL_MEMORY_CHIP_SELECT_1,
					 cfg->base, cfg->clock,
					 &data->smif_context, cfg->smif_block_cfg);
	if (result != CY_RSLT_SUCCESS) {
		LOG_ERR("Serial memory setup failed (QSPI) : 0x%x", result);
	}

	k_sem_init(&data->sem, 1, 1);

	return result;
}

static const struct flash_driver_api ifx_serial_memory_flash_driver_api = {
	.read = ifx_serial_memory_flash_read,
	.write = ifx_serial_memory_flash_write,
	.erase = ifx_serial_memory_flash_erase,
	.get_parameters = ifx_serial_memory_flash_get_parameters,
#ifdef CONFIG_FLASH_PAGE_LAYOUT
	.page_layout = ifx_serial_memory_flash_page_layout,
#endif
};

#define SOC_NV_FLASH_NODE(n) DT_PARENT(DT_INST(n, fixed_partitions))

#define PAGE_LEN(n) DT_PROP(SOC_NV_FLASH_NODE(n), erase_block_size)

/* Per-instance HF clock numbers */
#if DT_SAME_NODE(DT_DRV_INST(0), DT_NODELABEL(smif_0))
#define IFX_FLASH_HF_INST_NUM_0 8U
#else
#define IFX_FLASH_HF_INST_NUM_0 9U
#endif

#if DT_SAME_NODE(DT_DRV_INST(1), DT_NODELABEL(smif_0))
#define IFX_FLASH_HF_INST_NUM_1 8U
#else
#define IFX_FLASH_HF_INST_NUM_1 9U
#endif

#define IFX_FLASH_HF_INST_NUM(n) UTIL_CAT(IFX_FLASH_HF_INST_NUM_, n)

/* Per-instance block configs */
#if DT_SAME_NODE(DT_DRV_INST(0), DT_NODELABEL(smif_0))
#define IFX_SMIF_BLOCK_CONFIG_0 smif0BlockConfig
#define IFX_SMIF_CONFIG_0       SMIF0_config
#else
#define IFX_SMIF_BLOCK_CONFIG_0 smif1BlockConfig
#define IFX_SMIF_CONFIG_0       SMIF1_config
#endif

#if DT_SAME_NODE(DT_DRV_INST(1), DT_NODELABEL(smif_0))
#define IFX_SMIF_BLOCK_CONFIG_1 smif0BlockConfig
#define IFX_SMIF_CONFIG_1       SMIF0_config
#else
#define IFX_SMIF_BLOCK_CONFIG_1 smif1BlockConfig
#define IFX_SMIF_CONFIG_1       SMIF1_config
#endif

#define IFX_SMIF_BLOCK_CONFIG(n) UTIL_CAT(IFX_SMIF_BLOCK_CONFIG_, n)
#define IFX_SMIF_CONFIG(n)       UTIL_CAT(IFX_SMIF_CONFIG_, n)

#define INFINEON_CAT1_SMIF_INIT(n)                                                                  		\
	PINCTRL_DT_INST_DEFINE(n);                                                                 		\
                                                                                   				\
	extern cy_stc_smif_block_config_t IFX_SMIF_BLOCK_CONFIG(n);						\
                                                                                   				\
	extern cy_stc_smif_config_t IFX_SMIF_CONFIG(n); 							\
														\
	static const mtb_hal_hf_clock_t flash_clock_ref##n = {                       				\
		.inst_num = IFX_FLASH_HF_INST_NUM(n),                                				\
	};                                                                             				\
                                                                                   				\
	static const mtb_hal_clock_t CYBSP_SMIF_CORE_0_XSPI_FLASH_hal_clock##n = {    				\
		.clock_ref = &flash_clock_ref##n,                                     				\
		.interface = &mtb_hal_clock_hf_interface,                             				\
	};                                                                             				\
														\
	static struct ifx_serial_memory_flash_data flash_data##n;						\
														\
	static const struct ifx_serial_memory_flash_config flash_config##n = {					\
		.base =  (SMIF_CORE_Type *)DT_REG_ADDR(DT_DRV_INST(n)),						\
		.config = &IFX_SMIF_CONFIG(n),									\
		.base_addr = DT_REG_ADDR(SOC_NV_FLASH_NODE(n)),							\
		.max_addr = DT_REG_ADDR(SOC_NV_FLASH_NODE(n)) + DT_REG_SIZE(SOC_NV_FLASH_NODE(n)),		\
		.params = {											\
			.write_block_size = 									\
				DT_PROP(SOC_NV_FLASH_NODE(n), write_block_size),				\
			.erase_value = 0xFF,									\
		},												\
		IF_ENABLED(CONFIG_FLASH_PAGE_LAYOUT, (                               				\
		.layout = {                                                          				\
			.pages_count = DT_REG_SIZE(SOC_NV_FLASH_NODE(n)) /          				\
				       PAGE_LEN(n),                                 				\
			.pages_size  = PAGE_LEN(n),                               				\
		},))												\
		.clock = &CYBSP_SMIF_CORE_0_XSPI_FLASH_hal_clock##n,						\
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         		\
		.smif_block_cfg = &IFX_SMIF_BLOCK_CONFIG(n),							\
	};													\
														\
	DEVICE_DT_INST_DEFINE(n, ifx_serial_memory_flash_init, NULL, &flash_data##n, &flash_config##n,		\
		      	      POST_KERNEL, CONFIG_FLASH_INIT_PRIORITY, &ifx_serial_memory_flash_driver_api);
DT_INST_FOREACH_STATUS_OKAY(INFINEON_CAT1_SMIF_INIT)
