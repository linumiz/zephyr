#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(spi_master, LOG_LEVEL_INF);

#define SPI_NODE DT_NODELABEL(slave_test)

static const struct device *spi_dev = DEVICE_DT_GET(DT_BUS(SPI_NODE));

static struct spi_config spi_cfg = {
	.frequency = 1000000,  /* 1 MHz */
	.operation = SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB |
		     SPI_WORD_SET(8) | SPI_MODE_CPOL | SPI_MODE_CPHA,  /* Mode 3 */
	.slave = DT_REG_ADDR(SPI_NODE),
	.cs = {
		.gpio = SPI_CS_GPIOS_DT_SPEC_GET(SPI_NODE),
		.delay = 0,
	},
};

static int spi_transaction(uint8_t tx_data)
{
	uint8_t tx_buffer = tx_data;
	uint8_t rx_buffer = 0;
	int ret;

	const struct spi_buf tx_buf = {
		.buf = &tx_buffer,
		.len = 1
	};
	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1
	};

	struct spi_buf rx_buf = {
		.buf = &rx_buffer,
		.len = 1
	};
	const struct spi_buf_set rx = {
		.buffers = &rx_buf,
		.count = 1
	};

	LOG_INF("TX: 0x%02X", tx_data);

	ret = spi_transceive(spi_dev, &spi_cfg, &tx, &rx);
	if (ret < 0) {
		LOG_ERR("SPI transaction failed: %d", ret);
		LOG_INF("RX: 0x%02X", rx_buffer);
		return ret;
	}

	LOG_INF("RX: 0x%02X", rx_buffer);

	return 0;
}

int main(void)
{
	uint8_t counter = 0;
	int ret;

	LOG_INF("  SPI Master - Simple TX/RX Test");

	if (!device_is_ready(spi_dev)) {
		LOG_ERR("SPI device not ready");
		return -ENODEV;
	}

	LOG_INF("SPI device ready");
	k_sleep(K_SECONDS(2));

	while (1) {
		ret = spi_transaction(counter);

		if (ret < 0) {
			LOG_ERR("Transaction error");
		}

		LOG_INF("---");

		counter++;

		k_msleep(500);
	}

	return 0;
}
