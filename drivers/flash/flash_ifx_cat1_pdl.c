/*
 * Copyright 2025 Linumiz GmbH
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT	  infineon_cat1_flash_controller_pdl
#define SOC_NV_FLASH_NODE DT_PARENT(DT_INST(0, fixed_partitions))

#define PAGE_LEN DT_PROP(SOC_NV_FLASH_NODE, write_block_size)
#define SECTOR_LEN DT_PROP(SOC_NV_FLASH_NODE, erase_block_size)

#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/logging/log.h>

#include <cy_pdl.h>
#include <cy_flash.h>
#include <cy_syslib.h>

LOG_MODULE_REGISTER(flash_infineon_cat1, CONFIG_FLASH_LOG_LEVEL);

/* Device config structure */
struct ifx_cat1_flash_config {
	uint32_t base_addr;
	uint32_t max_addr;
};

/* Data structure */
struct ifx_cat1_flash_data {
	struct k_mutex mutex_lock;
};

static struct flash_parameters ifx_cat1_flash_parameters = {
	.write_block_size = PAGE_LEN,
	.erase_value = 0xFF,
};

static inline void flash_ifx_mutex_lock(const struct device *dev)
{
	struct ifx_cat1_flash_data *data = dev->data;

	k_mutex_lock(&data->mutex_lock, K_FOREVER);
}

static inline void flash_ifx_mutex_unlock(const struct device *dev)
{
	struct ifx_cat1_flash_data *data = dev->data;

	k_mutex_unlock(&data->mutex_lock);
}

static int ifx_cat1_flash_read(const struct device *dev, off_t offset, void *data, size_t data_len)
{
	const struct ifx_cat1_flash_config *dev_config = dev->config;
	uint32_t read_offset = dev_config->base_addr + offset;

	flash_ifx_mutex_lock(dev);

	/* As per PDL: Before reading data from previously programmed/erased
	 * flash rows, the user must clear or invalidate the flash cache
	 */
#if defined (ENABLE_CM7_DATA_CACHE) && defined(CONFIG_CACHE_MANAGEMENT) && defined(CONFIG_DCACHE)
	/* FOR M7 CORES */
	SCB_CleanInvalidateDCache_by_Addr((uint32_t *)read_offset, data_len);
#else
	/* FOR M0P Cores */
	FLASHC_FLASH_CMD = FLASHC_FLASH_CMD_INV_Msk;
#endif
	memcpy(data, (uint32_t *)read_offset, data_len);

	flash_ifx_mutex_unlock(dev);

	return 0;
}

static int ifx_cat1_flash_write(const struct device *dev, off_t offset, const void *data,
				size_t data_len)
{
	const struct ifx_cat1_flash_config *dev_config = dev->config;
	uint32_t write_offset = dev_config->base_addr + (uint32_t)offset;
	const uint8_t *data_ptr = (const uint8_t *)data;
	int ret = 0;

	if (data_len == 0) {
		return 0;
	}

	if ((data_len % PAGE_LEN != 0)) {
		LOG_INF("Data Len not aligned with Page len");
		return -EINVAL;
	}

	if ((offset < 0) || (offset % PAGE_LEN != 0)) {
		LOG_INF("Offset not aligned with Page len");
		return -EINVAL;
	}

	flash_ifx_mutex_lock(dev);

	while (data_len) {
		cy_en_flashdrv_status_t rslt = CY_FLASH_DRV_SUCCESS;
		rslt = Cy_Flash_ProgramRow(write_offset, (const uint32_t*)data_ptr);
		if (rslt != CY_FLASH_DRV_SUCCESS) {
			LOG_ERR("Error in writing @ 0x%x (Err:0x%x)", write_offset, rslt);
			ret = -EIO;
			goto out;
		}

		data_ptr += PAGE_LEN;
		write_offset += PAGE_LEN;
		data_len -= PAGE_LEN;
	}

out:
	flash_ifx_mutex_unlock(dev);
	return ret;
}

static int ifx_cat1_flash_erase(const struct device *dev, off_t offset, size_t size)
{
	const struct ifx_cat1_flash_config *config = dev->config;
	uint32_t erase_offset = config->base_addr + (uint32_t)offset;
	cy_en_flashdrv_status_t rslt = CY_FLASH_DRV_SUCCESS;
	int ret = 0;

	if (((erase_offset + size) > config->max_addr)) {
		LOG_ERR("Erase offset greater than maximum available address in Flash");
		return -EINVAL;
	}

	if ((offset < 0) || ((offset % SECTOR_LEN) != 0)) {
		LOG_ERR("Offset not aligned with sector len");
		return -EINVAL;
	}

	if (((size % SECTOR_LEN) != 0)) {
		LOG_ERR("Size not aligned with sector len");
		return -EINVAL;
	}

	flash_ifx_mutex_lock(dev);

	while (size) {
		rslt = Cy_Flash_EraseSector(erase_offset);
		if (rslt != CY_FLASH_DRV_SUCCESS) {
			LOG_ERR("Error in erasing address 0x%x : 0x%x", erase_offset, rslt);
			ret = -EIO;
			goto out;
		}
		size -= SECTOR_LEN;
		erase_offset += SECTOR_LEN;
	}

out:
	flash_ifx_mutex_unlock(dev);
	return ret;
}

#if CONFIG_FLASH_PAGE_LAYOUT
static const struct flash_pages_layout ifx_cat1_flash_pages_layout = {
	.pages_count = DT_REG_SIZE(SOC_NV_FLASH_NODE) / PAGE_LEN,
	.pages_size = PAGE_LEN,
};

static void ifx_cat1_flash_page_layout(const struct device *dev,
				       const struct flash_pages_layout **layout,
				       size_t *layout_size)
{
	*layout = &ifx_cat1_flash_pages_layout;

	/*
	 * For flash memories which have uniform page sizes, this routine
	 * returns an array of length 1, which specifies the page size and
	 * number of pages in the memory.
	 */
	*layout_size = 1;
}
#endif /* CONFIG_FLASH_PAGE_LAYOUT */

static const struct flash_parameters *ifx_cat1_flash_get_parameters(const struct device *dev)
{
	ARG_UNUSED(dev);

	return &ifx_cat1_flash_parameters;
}

static int ifx_cat1_flash_init(const struct device *dev)
{
	struct ifx_cat1_flash_data *data = dev->data;
	const struct ifx_cat1_flash_config *dev_config = dev->config;

	/* Check bounds and see which one to enable */
	if (((CY_FLASH_LG_SBM_TOP <= dev_config->base_addr) && (dev_config->base_addr < CY_FLASH_LG_SBM_END)) ||
	    ((CY_FLASH_SM_SBM_TOP <= dev_config->base_addr) && (dev_config->base_addr < CY_FLASH_SM_SBM_END))) {
		Cy_Flashc_MainWriteEnable();
	} else if (((CY_WFLASH_LG_SBM_TOP <= dev_config->base_addr) && (dev_config->base_addr < CY_WFLASH_LG_SBM_END)) ||
		   ((CY_WFLASH_SM_SBM_TOP <= dev_config->base_addr) && (dev_config->base_addr < CY_WFLASH_SM_SBM_END))) {
		Cy_Flashc_WorkWriteEnable();
	}

	k_mutex_init(&data->mutex_lock);

	return 0;
}

static DEVICE_API(flash, ifx_cat1_flash_driver_api) = {
	.read = ifx_cat1_flash_read,
	.write = ifx_cat1_flash_write,
	.erase = ifx_cat1_flash_erase,
	.get_parameters = ifx_cat1_flash_get_parameters,
#ifdef CONFIG_FLASH_PAGE_LAYOUT
	.page_layout = ifx_cat1_flash_page_layout,
#endif
};

static struct ifx_cat1_flash_data flash_data;

static const struct ifx_cat1_flash_config ifx_cat1_flash_config = {
	.base_addr = DT_REG_ADDR(SOC_NV_FLASH_NODE),
	.max_addr = DT_REG_ADDR(SOC_NV_FLASH_NODE) + DT_REG_SIZE(SOC_NV_FLASH_NODE)};

DEVICE_DT_INST_DEFINE(0, ifx_cat1_flash_init, NULL, &flash_data, &ifx_cat1_flash_config,
		      POST_KERNEL, CONFIG_FLASH_INIT_PRIORITY, &ifx_cat1_flash_driver_api);
