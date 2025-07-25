/*
 * Copyright (c) 2025 Texas Instruments
 * Copyright (c) 2025 Linumiz
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ti_mspm0_trng

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/entropy.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/ring_buffer.h>

/* TI Driverlib includes */
#include <ti/driverlib/dl_trng.h>
#include <ti/devices/msp/peripherals/hw_trng.h>

#define TRNG_BUFFER_SIZE	CONFIG_ENTROPY_MSPM0_TRNG_POOL_SIZE
#define TRNG_DECIMATION_RATE	CONFIG_ENTROPY_MSPM0_TRNG_DECIMATION_RATE
#define TRNG_SAMPLE_SIZE	4

struct entropy_mspm0_trng_config {
	TRNG_Regs *base;
};

struct entropy_mspm0_trng_data {
	struct k_sem sem_lock;
	struct k_sem sem_sync;
	struct ring_buf entropy_pool;
	uint8_t pool_buffer[TRNG_BUFFER_SIZE];
	bool initialized;
};

static void configure_trng_clock_divider(const struct device *dev)
{
	const struct entropy_mspm0_trng_config *cfg = dev->config;
#ifdef CONFIG_ENTROPY_MSPM0_TRNG_CLK_DIV_1
	#define TRNG_SAMPLE_GENERATE_TIME (32 * (TRNG_DECIMATION_RATE+1) \
				/	CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC)
	DL_TRNG_setClockDivider(cfg->base, DL_TRNG_CLOCK_DIVIDE_1);
#elif defined CONFIG_ENTROPY_MSPM0_TRNG_CLK_DIV_2
	#define TRNG_SAMPLE_GENERATE_TIME (32 * (TRNG_DECIMATION_RATE+1) \
				/	(CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC / 2))
	DL_TRNG_setClockDivider(cfg->base, DL_TRNG_CLOCK_DIVIDE_2);
#elif defined CONFIG_ENTROPY_MSPM0_TRNG_CLK_DIV_4
	#define TRNG_SAMPLE_GENERATE_TIME (32 * (TRNG_DECIMATION_RATE+1) \
				/	(CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC / 4))
	DL_TRNG_setClockDivider(cfg->base, DL_TRNG_CLOCK_DIVIDE_4);
#elif defined CONFIG_ENTROPY_MSPM0_TRNG_CLK_DIV_6
	#define TRNG_SAMPLE_GENERATE_TIME (32 * (TRNG_DECIMATION_RATE+1) \
				/	(CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC / 6))
	DL_TRNG_setClockDivider(cfg->base, DL_TRNG_CLOCK_DIVIDE_6);
#elif defined CONFIG_ENTROPY_MSPM0_TRNG_CLK_DIV_8
	#define TRNG_SAMPLE_GENERATE_TIME (32 * (TRNG_DECIMATION_RATE+1) \
				/	(CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC / 8))
	DL_TRNG_setClockDivider(cfg->base, DL_TRNG_CLOCK_DIVIDE_8);
#endif
}

static int run_startup_tests(const struct device *dev)
{
	const struct entropy_mspm0_trng_config *config = dev->config;
	uint8_t dig_test_result, ana_test_result;

	/* Move TRNG to NORM_FUNC state */
	DL_TRNG_sendCommand(config->base, DL_TRNG_CMD_NORM_FUNC);

	while (!DL_TRNG_isCommandDone(config->base)) {
	}

	DL_TRNG_clearInterruptStatus(config->base, DL_TRNG_INTERRUPT_CMD_DONE_EVENT);

	/* Run digital self-test */
	DL_TRNG_sendCommand(config->base, DL_TRNG_CMD_TEST_DIG);

	while (!DL_TRNG_isCommandDone(config->base)) {
	}

	DL_TRNG_clearInterruptStatus(config->base, DL_TRNG_INTERRUPT_CMD_DONE_EVENT);

	/* Check digital test results */
	dig_test_result = DL_TRNG_getDigitalHealthTestResults(config->base);
	if (dig_test_result != DL_TRNG_DIGITAL_HEALTH_TEST_SUCCESS) {
		return -EIO;
	}

	/* Run analog self-test */
	DL_TRNG_sendCommand(config->base, DL_TRNG_CMD_TEST_ANA);
	while (!DL_TRNG_isCommandDone(config->base)) {
	}
	DL_TRNG_clearInterruptStatus(config->base, DL_TRNG_INTERRUPT_CMD_DONE_EVENT);

	/* Check analog test results */
	ana_test_result = DL_TRNG_getAnalogHealthTestResults(config->base);
	if (ana_test_result != DL_TRNG_ANALOG_HEALTH_TEST_SUCCESS) {
		return -EIO;
	}

	return 0;
}

static int configure_normal_operation(const struct device *dev)
{
	const struct entropy_mspm0_trng_config *config = dev->config;
	uint32_t dummy_data;

	/* Clear IRQ_CAPTURED_RDY_IRQ status */
	DL_TRNG_clearInterruptStatus(config->base, DL_TRNG_INTERRUPT_CAPTURE_RDY_EVENT);

	/* Set decimation rate and send NORM_FUNC command */
	DL_TRNG_setDecimationRate(config->base, (DL_TRNG_DECIMATION_RATE)TRNG_DECIMATION_RATE);

	DL_TRNG_sendCommand(config->base, DL_TRNG_CMD_NORM_FUNC);

	while (!DL_TRNG_isCommandDone(config->base)) {
	}

	DL_TRNG_clearInterruptStatus(config->base, DL_TRNG_INTERRUPT_CMD_DONE_EVENT);

	/* Enable health fail interrupt */
	DL_TRNG_enableInterrupt(config->base, DL_TRNG_INTERRUPT_HEALTH_FAIL_EVENT);

	/* Enable data captured interrupt */
	DL_TRNG_enableInterrupt(config->base, DL_TRNG_INTERRUPT_CAPTURE_RDY_EVENT);

	/* Discard first sample as its not a truely random sample */
	while (!DL_TRNG_isCaptureReady(config->base)) {
	}

	dummy_data = DL_TRNG_getCapture(config->base);
	DL_TRNG_clearInterruptStatus(config->base, DL_TRNG_INTERRUPT_CAPTURE_RDY_EVENT);

	return 0;
}

static void entropy_mspm0_trng_isr(const struct device *dev)
{
	const struct entropy_mspm0_trng_config *config = dev->config;
	struct entropy_mspm0_trng_data *data = dev->data;
	uint32_t status;
	uint32_t entropy_data;

	status = DL_TRNG_getEnabledInterruptStatus(config->base,
		 DL_TRNG_INTERRUPT_CAPTURE_RDY_EVENT | DL_TRNG_INTERRUPT_HEALTH_FAIL_EVENT);

	if (status & DL_TRNG_INTERRUPT_HEALTH_FAIL_EVENT) {
		DL_TRNG_clearInterruptStatus(config->base, DL_TRNG_INTERRUPT_HEALTH_FAIL_EVENT);
		return;
	}

	if ((status & DL_TRNG_INTERRUPT_CAPTURE_RDY_EVENT) && DL_TRNG_isCaptureReady(config->base)) {
		entropy_data = DL_TRNG_getCapture(config->base);
		ring_buf_put(&data->entropy_pool, (uint8_t *)&entropy_data, TRNG_SAMPLE_SIZE);
		DL_TRNG_clearInterruptStatus(config->base, DL_TRNG_INTERRUPT_CAPTURE_RDY_EVENT);
		k_sem_give(&data->sem_sync);
	}
}

static int entropy_mspm0_trng_get_entropy(const struct device *dev, uint8_t *buffer, uint16_t length)
{
	struct entropy_mspm0_trng_data *data = dev->data;
	uint16_t bytes_read;

	if (!data->initialized) {
		return -ENODEV;
	}

	while (length) {
		k_sem_take(&data->sem_lock, K_FOREVER);
		bytes_read = ring_buf_get(&data->entropy_pool, buffer, length);
		k_sem_give(&data->sem_lock);

		if (bytes_read) {
			buffer += bytes_read;
			length -= bytes_read;
		} else {
			k_sem_take(&data->sem_sync, K_FOREVER);
		}
	}

	return 0;
}

static int entropy_mspm0_trng_get_entropy_isr(const struct device *dev, uint8_t *buffer,
					      uint16_t length, uint32_t flags)
{
	const struct entropy_mspm0_trng_config *config = dev->config;
	struct entropy_mspm0_trng_data *data = dev->data;
	uint16_t bytes_read = 0;
	uint16_t total_read = 0;
	uint32_t entropy_data;
	unsigned int key;

	if (!data->initialized) {
		return -ENODEV;
	}

	/* Try to get entropy from existing ring buffer */
	key = irq_lock();
	bytes_read = ring_buf_get(&data->entropy_pool, buffer, length);
	irq_unlock(key);

	total_read = bytes_read;

	if ((bytes_read == length) || ((flags & ENTROPY_BUSYWAIT) == 0U)) {
		/* Either we got all requested data, or busy-waiting is not allowed */
		return total_read;
	}

	/* Busy-wait for additional data (only if ENTROPY_BUSYWAIT is set) */
	buffer += bytes_read;
	length -= bytes_read;

	while (length) {
		key = irq_lock();

		/* Check if data is ready by checking IRQ_CAPTURED_RDY */
		if (DL_TRNG_isCaptureReady(config->base)) {
			entropy_data = DL_TRNG_getCapture(config->base);
			DL_TRNG_clearInterruptStatus(config->base, DL_TRNG_INTERRUPT_CAPTURE_RDY_EVENT);
			ring_buf_put(&data->entropy_pool, (uint8_t *)&entropy_data, TRNG_SAMPLE_SIZE);
			bytes_read = ring_buf_get(&data->entropy_pool, buffer, length);
		} else {
			bytes_read = ring_buf_get(&data->entropy_pool, buffer, length);
		}

		irq_unlock(key);

		if (bytes_read) {
			buffer += bytes_read;
			length -= bytes_read;
			total_read += bytes_read;
		} else {
			k_busy_wait(TRNG_SAMPLE_GENERATE_TIME);
		}
	}
	return total_read;
}

static int entropy_mspm0_trng_init(const struct device *dev)
{
	const struct entropy_mspm0_trng_config *config = dev->config;
	struct entropy_mspm0_trng_data *data = dev->data;
	int ret;

	/* Initialize ring buffer for entropy storage */
	ring_buf_init(&data->entropy_pool, sizeof(data->pool_buffer), data->pool_buffer);

	/* Step 1: Enable TRNG power */
	DL_TRNG_enablePower(config->base);

	/* Step 2: Configure TRNG clock divider */
	configure_trng_clock_divider(dev);

	/* Step 3: Verify that the TRNG interrupts are disabled */
	DL_TRNG_disableInterrupt(config->base,
		DL_TRNG_INTERRUPT_HEALTH_FAIL_EVENT |
		DL_TRNG_INTERRUPT_CMD_DONE_EVENT |
		DL_TRNG_INTERRUPT_CMD_FAIL_EVENT |
		DL_TRNG_INTERRUPT_CAPTURE_RDY_EVENT);

	ret = run_startup_tests(dev);
	if (ret < 0) {
		return ret;
	}

	ret = configure_normal_operation(dev);
	if (ret < 0) {
		return ret;
	}

	IRQ_CONNECT(DT_INST_IRQN(0), DT_INST_IRQ(0, priority),
		    entropy_mspm0_trng_isr, DEVICE_DT_INST_GET(0), 0);
	irq_enable(DT_INST_IRQN(0));

	data->initialized = true;

	return 0;
}

static const struct entropy_driver_api entropy_mspm0_trng_driver_api = {
	.get_entropy = entropy_mspm0_trng_get_entropy,
	.get_entropy_isr = entropy_mspm0_trng_get_entropy_isr,
};

static const struct entropy_mspm0_trng_config entropy_mspm0_trng_config = {
	.base = (TRNG_Regs *)DT_INST_REG_ADDR(0),
};

static struct entropy_mspm0_trng_data entropy_mspm0_trng_data = {
	.sem_lock = Z_SEM_INITIALIZER(entropy_mspm0_trng_data.sem_lock, 1, 1),
	.sem_sync = Z_SEM_INITIALIZER(entropy_mspm0_trng_data.sem_sync, 0, 1),
};

DEVICE_DT_INST_DEFINE(0, entropy_mspm0_trng_init, NULL,
		      &entropy_mspm0_trng_data,
		      &entropy_mspm0_trng_config, PRE_KERNEL_1,
		      CONFIG_ENTROPY_INIT_PRIORITY,
		      &entropy_mspm0_trng_driver_api);

