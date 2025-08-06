/*
 * Copyright (c) 2025 Linumiz GmbH
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
#define TRNG_SAMPLE_GENERATE_TIME	(1000000 * (32 * (TRNG_DECIMATION_RATE + 1)) \
		/ (CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC / (config->clock_divider)))

struct entropy_mspm0_trng_config {
	TRNG_Regs *base;
	uint8_t clock_divider;
};

struct entropy_mspm0_trng_data {
	struct k_sem sem_lock;
	struct k_sem sem_sync;
	struct ring_buf entropy_pool;
	uint8_t pool_buffer[TRNG_BUFFER_SIZE];
};

static void entropy_mspm0_trng_isr(const struct device *dev)
{
	const struct entropy_mspm0_trng_config *config = dev->config;
	struct entropy_mspm0_trng_data *data = dev->data;
	uint32_t status;
	uint32_t entropy_data;
	uint8_t dig_test;
	uint8_t ana_test;
	uint32_t bytes_written;

	status = DL_TRNG_getEnabledInterruptStatus(config->base,
						   DL_TRNG_INTERRUPT_CAPTURE_RDY_EVENT |
						   DL_TRNG_INTERRUPT_HEALTH_FAIL_EVENT |
						   DL_TRNG_INTERRUPT_CMD_DONE_EVENT);

	if (status & DL_TRNG_INTERRUPT_HEALTH_FAIL_EVENT) {
		DL_TRNG_clearInterruptStatus(config->base, DL_TRNG_INTERRUPT_HEALTH_FAIL_EVENT);
		return;
	}

	if (status & DL_TRNG_INTERRUPT_CMD_DONE_EVENT) {
		DL_TRNG_clearInterruptStatus(config->base, DL_TRNG_INTERRUPT_CMD_DONE_EVENT);

		/* Run DIG test */
		dig_test = DL_TRNG_getDigitalHealthTestResults(config->base);
		if (!dig_test) {
			DL_TRNG_sendCommand(config->base, DL_TRNG_CMD_TEST_DIG);
			return;
		}

		/* Run ANALOG test */
		ana_test = DL_TRNG_getAnalogHealthTestResults(config->base);
		if (!ana_test) {
			DL_TRNG_sendCommand(config->base, DL_TRNG_CMD_TEST_ANA);
			return;
		}

		/*
		 * If both tests are successful, set DECIM RATE, enable IRQ_CAPTURE_RDY
		 * and discard first sample from DATA_CAPTURE register
		 */
		if ((dig_test == DL_TRNG_DIGITAL_HEALTH_TEST_SUCCESS) &&
		     (ana_test == DL_TRNG_ANALOG_HEALTH_TEST_SUCCESS)) {
			DL_TRNG_clearInterruptStatus(config->base,
						     DL_TRNG_INTERRUPT_CAPTURE_RDY_EVENT);
			DL_TRNG_setDecimationRate(config->base,
						  (DL_TRNG_DECIMATION_RATE)TRNG_DECIMATION_RATE);
			DL_TRNG_disableInterrupt(config->base, DL_TRNG_INTERRUPT_CMD_DONE_EVENT);
			DL_TRNG_enableInterrupt(config->base, DL_TRNG_INTERRUPT_CAPTURE_RDY_EVENT);
			DL_TRNG_getCapture(config->base);
			return;
		}
	}

	if ((status & DL_TRNG_INTERRUPT_CAPTURE_RDY_EVENT)) {
		entropy_data = DL_TRNG_getCapture(config->base);
		DL_TRNG_clearInterruptStatus(config->base,
					     DL_TRNG_INTERRUPT_CAPTURE_RDY_EVENT);

		bytes_written = ring_buf_put(&data->entropy_pool, (uint8_t *)&entropy_data,
					     TRNG_SAMPLE_SIZE);

		/* If the ring buf is exhausted, disable the interrupt in IMASK */
		if (bytes_written != TRNG_SAMPLE_SIZE) {
			DL_TRNG_disableInterrupt(config->base, DL_TRNG_INTERRUPT_CAPTURE_RDY_EVENT);
		}

		k_sem_give(&data->sem_sync);
	}
}

static int entropy_mspm0_trng_get_entropy(const struct device *dev,
					  uint8_t *buffer, uint16_t length)
{
	const struct entropy_mspm0_trng_config *config = dev->config;
	struct entropy_mspm0_trng_data *data = dev->data;
	uint16_t bytes_read;

	while (length) {
		k_sem_take(&data->sem_lock, K_FOREVER);
		bytes_read = ring_buf_get(&data->entropy_pool, buffer, length);
		k_sem_give(&data->sem_lock);

		/*
		 * If no bytes read, i.e ring buf is exhausted, enable the interrupt and
		 * wait until the additonal entropy is available in ring buf.
		 */
		if (bytes_read == 0U) {
			DL_TRNG_enableInterrupt(config->base,
						DL_TRNG_INTERRUPT_CAPTURE_RDY_EVENT);
			k_sem_take(&data->sem_sync, K_FOREVER);
			continue;
		}

		buffer += bytes_read;
		length -= bytes_read;

	}

	return 0;
}

static int entropy_mspm0_trng_get_entropy_isr(const struct device *dev, uint8_t *buffer,
					      uint16_t length, uint32_t flags)
{
	const struct entropy_mspm0_trng_config *config = dev->config;
	struct entropy_mspm0_trng_data *data = dev->data;
	uint16_t bytes_read;
	uint16_t total_read;
	uint32_t entropy_data;
	unsigned int key;

	/* Try to get entropy from existing ring buffer */
	bytes_read = ring_buf_get(&data->entropy_pool, buffer, length);
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
			DL_TRNG_clearInterruptStatus(config->base,
						     DL_TRNG_INTERRUPT_CAPTURE_RDY_EVENT);
			ring_buf_put(&data->entropy_pool, (uint8_t *)&entropy_data,
				     TRNG_SAMPLE_SIZE);
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

	/* Initialize ring buffer for entropy storage */
	ring_buf_init(&data->entropy_pool, sizeof(data->pool_buffer), data->pool_buffer);

	/* Enable TRNG power */
	DL_TRNG_enablePower(config->base);

	/* Configure TRNG clock divider */
	DL_TRNG_setClockDivider(config->base, (DL_TRNG_CLOCK_DIVIDE)(config->clock_divider - 1));

	/* Disable the CAPTURE_RDY IRQ until health tests are complete */
	DL_TRNG_disableInterrupt(config->base, DL_TRNG_INTERRUPT_CAPTURE_RDY_EVENT);
	DL_TRNG_enableInterrupt(config->base, DL_TRNG_INTERRUPT_CMD_DONE_EVENT);
	DL_TRNG_enableInterrupt(config->base, DL_TRNG_INTERRUPT_HEALTH_FAIL_EVENT);

	/* Move TRNG from OFF to NORM FUNC state */
	DL_TRNG_sendCommand(config->base, DL_TRNG_CMD_NORM_FUNC);

	IRQ_CONNECT(DT_INST_IRQN(0), DT_INST_IRQ(0, priority),
		    entropy_mspm0_trng_isr, DEVICE_DT_INST_GET(0), 0);
	irq_enable(DT_INST_IRQN(0));

	return 0;
}

static const struct entropy_driver_api entropy_mspm0_trng_driver_api = {
	.get_entropy = entropy_mspm0_trng_get_entropy,
	.get_entropy_isr = entropy_mspm0_trng_get_entropy_isr,
};

static const struct entropy_mspm0_trng_config entropy_mspm0_trng_config = {
	.base = (TRNG_Regs *)DT_INST_REG_ADDR(0),
	.clock_divider = DT_INST_PROP(0, ti_clk_div),
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
