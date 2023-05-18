/*
 * Copyright (c) 2021 Linumiz
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT hzgrow_r502a

#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/byteorder.h>

#include <zephyr/drivers/sensor/grow_r502a.h>
#include "grow_r502a.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(GROW_R502A, CONFIG_SENSOR_LOG_LEVEL);

static int transceive_packet(const struct device *dev, union r502a_packet *tx_packet,
				union r502a_packet *rx_packet, uint16_t const data_len)
{
	const struct grow_r502a_config *cfg = dev->config;
	struct grow_r502a_data *drv_data = dev->data;

	if (tx_packet) {
		uint16_t check_sum, pkg_len;

		pkg_len = (data_len >> 8) + (data_len & 0xff) + R502A_CHECKSUM_LEN;
		check_sum = pkg_len + tx_packet->pid;

		tx_packet->start = sys_cpu_to_be16(R502A_STARTCODE);
		tx_packet->addr = sys_cpu_to_be32(cfg->comm_addr);
		tx_packet->len = sys_cpu_to_be16(pkg_len);

		for (int i = 0; i < data_len; i++) {
			check_sum += tx_packet->data[i];
		}
		sys_put_be16(check_sum, &tx_packet->buf[data_len + R502A_HEADER_LEN]);

		drv_data->tx_buf.len = pkg_len + R502A_HEADER_LEN;
		drv_data->tx_buf.data = tx_packet->buf;

		LOG_HEXDUMP_DBG(drv_data->tx_buf.data, drv_data->tx_buf.len, "TX");

		uart_irq_tx_enable(cfg->dev);

		if (k_sem_take(&drv_data->uart_tx_sem, K_MSEC(1500)) != 0) {
			LOG_ERR("Tx data timeout");
			return -ETIMEDOUT;
		}
	}

	if (rx_packet) {
		drv_data->rx_buf.data = rx_packet->buf;
		drv_data->rx_buf.len = 0;
		drv_data->pkt_len = R502A_HEADER_LEN;
		uart_irq_rx_enable(cfg->dev);
		if (k_sem_take(&drv_data->uart_rx_sem, K_MSEC(1500)) != 0) {
			LOG_ERR("Rx data timeout");
			return -ETIMEDOUT;
		}
	}

	return 0;
}

static int r502a_validate_rx_packet(union r502a_packet *rx_packet)
{
	uint16_t recv_cks = 0, calc_cks = 0;
	uint8_t cks_start_idx;

	if (sys_be16_to_cpu(rx_packet->start) == R502A_STARTCODE) {
		LOG_DBG("startcode matched 0x%X", sys_be16_to_cpu(rx_packet->start));
	} else {
		LOG_ERR("startcode didn't match 0x%X", sys_be16_to_cpu(rx_packet->start));
		return -EINVAL;
	}

	if (sys_be32_to_cpu(rx_packet->addr) == R502A_DEFAULT_ADDRESS) {
		LOG_DBG("Address matched 0x%X", sys_be32_to_cpu(rx_packet->addr));
	} else {
		LOG_ERR("Address didn't match 0x%X", sys_be32_to_cpu(rx_packet->addr));
		return -EINVAL;
	}

	switch (rx_packet->pid) {
	case R502A_DATA_PACKET:
		LOG_DBG("Data Packet Received 0x%X", rx_packet->pid);
		break;
	case R502A_END_DATA_PACKET:
		LOG_DBG("End of Data Packet Received 0x%X", rx_packet->pid);
		break;
	case R502A_ACK_PACKET:
		LOG_DBG("Acknowledgment Packet Received 0x%X", rx_packet->pid);
		break;
	default:
		LOG_ERR("Error Package ID 0x%X", rx_packet->pid);
		return -EINVAL;
	}

	cks_start_idx = sys_be16_to_cpu(rx_packet->len) - R502A_CHECKSUM_LEN;

	recv_cks = sys_get_be16(&rx_packet->data[cks_start_idx]);

	calc_cks += rx_packet->pid + (sys_be16_to_cpu(rx_packet->len) >> 8) +
			 (sys_be16_to_cpu(rx_packet->len) & 0xff);

	for (int i = 0; i < cks_start_idx; i++) {
		calc_cks += rx_packet->data[i];
	}

	if (recv_cks == calc_cks) {
		LOG_DBG("Checksum matched calculated 0x%x received 0x%x", calc_cks, recv_cks);
	} else {
		LOG_ERR("Checksum mismatch calculated 0x%x received 0x%x", calc_cks, recv_cks);
		return -EINVAL;
	}

	return 0;
}

static void uart_cb_tx_handler(const struct device *dev)
{
	const struct grow_r502a_config *config = dev->config;
	struct grow_r502a_data *drv_data = dev->data;
	int sent = 0;
	uint8_t retries = 3;

	while (drv_data->tx_buf.len) {
		sent = uart_fifo_fill(config->dev, &drv_data->tx_buf.data[sent],
							drv_data->tx_buf.len);
		drv_data->tx_buf.len -= sent;
	}

	while (retries--) {
		if (uart_irq_tx_complete(config->dev)) {
			uart_irq_tx_disable(config->dev);
			k_sem_give(&drv_data->uart_tx_sem);
			break;
		}
	}
}

static void uart_cb_handler(const struct device *dev, void *user_data)
{
	const struct device *uart_dev = user_data;
	struct grow_r502a_data *drv_data = uart_dev->data;
	int len = 0;
	int offset = drv_data->rx_buf.len;

	if ((uart_irq_update(dev) > 0) && (uart_irq_is_pending(dev) > 0)) {
		if (uart_irq_tx_ready(dev)) {
			uart_cb_tx_handler(uart_dev);
		}

		while (uart_irq_rx_ready(dev)) {
			len = uart_fifo_read(dev, &drv_data->rx_buf.data[offset],
								drv_data->pkt_len);
			offset += len;
			drv_data->rx_buf.len = offset;

			if (drv_data->pkt_len != len) {
				drv_data->pkt_len -= len;
				continue;
			}

			if (offset == R502A_HEADER_LEN) {
				drv_data->pkt_len = sys_get_be16(
							&drv_data->rx_buf.data[R502A_PKG_LEN_IDX]
							);
				continue;
			}

			LOG_HEXDUMP_DBG(drv_data->rx_buf.data, offset, "RX");
			uart_irq_rx_disable(dev);
			k_sem_give(&drv_data->uart_rx_sem);
			break;
		}
	}
}

/**
 * @brief	Set sensor device's basic parameters like baud rate, security level
 *		and data package length.
 */
static int fps_set_sys_param(const struct device *dev, const struct sensor_value *val)
{
	struct grow_r502a_data *drv_data = dev->data;
	union r502a_packet rx_packet = {0};
	int ret = 0;
	char const set_sys_param_len = 3;

	union r502a_packet tx_packet = {
		.pid = R502A_COMMAND_PACKET,
		.data = { R502A_SETSYSPARAM, val->val1, val->val2}
	};

	k_mutex_lock(&drv_data->lock, K_FOREVER);

	ret = transceive_packet(dev, &tx_packet, &rx_packet, set_sys_param_len);
	if (ret != 0) {
		goto unlock;
	}

	ret = r502a_validate_rx_packet(&rx_packet);
	if (ret != 0) {
		goto unlock;
	}

	if (rx_packet.buf[R502A_CC_IDX] == R502A_OK) {
		LOG_DBG("R502A set system parameter success");
	} else {
		LOG_ERR("R502A set system parameter error %d", rx_packet.buf[R502A_CC_IDX]);
		ret = -EIO;
		goto unlock;
	}

unlock:
	k_mutex_unlock(&drv_data->lock);
	return ret;
}

static int fps_read_sys_param(const struct device *dev, struct r502a_sys_param *val)
{
	struct grow_r502a_data *drv_data = dev->data;

	union r502a_packet rx_packet = {0};
	int offset = 0, ret = 0;
	char const read_sys_param_len = 1;

	union r502a_packet tx_packet = {
		.pid = R502A_COMMAND_PACKET,
		.data = {R502A_READSYSPARAM}
	};

	k_mutex_lock(&drv_data->lock, K_FOREVER);

	ret = transceive_packet(dev, &tx_packet, &rx_packet, read_sys_param_len);
	if (ret != 0) {
		goto unlock;
	}

	ret = r502a_validate_rx_packet(&rx_packet);
	if (ret != 0) {
		goto unlock;
	}

	if (rx_packet.buf[R502A_CC_IDX] == R502A_OK) {
		LOG_DBG("R502A read system parameter success");
	} else {
		LOG_ERR("R502A read system parameter error %d", rx_packet.buf[R502A_CC_IDX]);
		ret = -EIO;
		goto unlock;
	}

	val->status_reg = sys_get_be16(
				&rx_packet.data[offsetof(struct r502a_sys_param, status_reg) + 1]
				);
	val->system_id = sys_get_be16(
				&rx_packet.data[offsetof(struct r502a_sys_param, system_id) + 1]
				);
	val->lib_size = sys_get_be16(
				&rx_packet.data[offsetof(struct r502a_sys_param, lib_size) + 1]
				);
	val->sec_level = sys_get_be16(
				&rx_packet.data[offsetof(struct r502a_sys_param, sec_level) + 1]
				);
	val->addr = sys_get_be32(
			&rx_packet.data[offsetof(struct r502a_sys_param, addr) + 1]
			);
	offset = sys_get_be16(
			&rx_packet.data[offsetof(struct r502a_sys_param, data_pkt_size) + 1]
			);
	val->data_pkt_size = 32 * (1 << offset);
	val->baud = sys_get_be16(
			&rx_packet.data[offsetof(struct r502a_sys_param, baud) + 1]
			) * 9600;

unlock:
	k_mutex_unlock(&drv_data->lock);
	return ret;
}

static int fps_led_control(const struct device *dev, struct r502a_led_params *led_control)
{
	struct grow_r502a_data *drv_data = dev->data;
	union r502a_packet rx_packet = {0};
	char const led_ctrl_len = 5;
	int ret = 0;

	union r502a_packet tx_packet = {
		.pid = R502A_COMMAND_PACKET,
		.data = { R502A_LED_CONFIG, led_control->ctrl_code,
				led_control->speed, led_control->color_idx, led_control->cycle}
	};

	ret = transceive_packet(dev, &tx_packet, &rx_packet, led_ctrl_len);
	if (ret != 0) {
		return ret;
	}

	ret = r502a_validate_rx_packet(&rx_packet);
	if (ret != 0) {
		return ret;
	}

	if (rx_packet.buf[R502A_CC_IDX] == R502A_OK) {
		LOG_DBG("R502A LED ON");
		k_sleep(K_MSEC(R502A_DELAY));
	} else {
		LOG_ERR("R502A LED control error %d", rx_packet.buf[R502A_CC_IDX]);
		return -EIO;
	}

	return 0;
}

static int fps_verify_password(const struct device *dev)
{
	struct grow_r502a_data *drv_data = dev->data;
	union r502a_packet rx_packet = {0};
	char const verify_pwd_len = 5;
	int ret = 0;

	union r502a_packet tx_packet = {
		.pid = R502A_COMMAND_PACKET,
		.data[0] = R502A_VERIFYPASSWORD,
	};

	sys_put_be32(R502A_DEFAULT_PASSWORD, &tx_packet.data[1]);

	ret = transceive_packet(dev, &tx_packet, &rx_packet, verify_pwd_len);
	if (ret != 0) {
		return ret;
	}

	ret = r502a_validate_rx_packet(&rx_packet);
	if (ret != 0) {
		return ret;
	}

	if (rx_packet.buf[R502A_CC_IDX] == R502A_OK) {
		LOG_DBG("Correct password, R502A verified");
	} else {
		LOG_ERR("Password verification error 0x%X", rx_packet.buf[R502A_CC_IDX]);
		return -EIO;
	}

	return 0;
}

static int fps_get_template_count(const struct device *dev)
{
	struct grow_r502a_data *drv_data = dev->data;
	union r502a_packet rx_packet = {0};
	char const get_temp_cnt_len = 1;
	int ret = 0;

	union r502a_packet tx_packet = {
		.pid = R502A_COMMAND_PACKET,
		.data = {R502A_TEMPLATECOUNT},
	};

	ret = transceive_packet(dev, &tx_packet, &rx_packet, get_temp_cnt_len);
	if (ret != 0) {
		return ret;
	}

	ret = r502a_validate_rx_packet(&rx_packet);
	if (ret != 0) {
		return ret;
	}

	if (rx_packet.buf[R502A_CC_IDX] == R502A_OK) {
		LOG_DBG("Read success");
		drv_data->template_count = sys_get_be16(&rx_packet.data[1]);
		LOG_INF("Remaining templates count : %d", drv_data->template_count);
	} else {
		LOG_ERR("R502A template count get error");
		return -EIO;
	}

	return 0;
}

static int fps_read_template_table(const struct device *dev, struct sensor_value *val)
{
	struct grow_r502a_data *drv_data = dev->data;
	union r502a_packet rx_packet = {0};
	char const temp_table_len = 2;
	int ret = 0;

	union r502a_packet tx_packet = {
		.pid = R502A_COMMAND_PACKET,
		.data = {R502A_READTEMPLATEINDEX, 0x00}
	};

	k_mutex_lock(&drv_data->lock, K_FOREVER);

	ret = transceive_packet(dev, &tx_packet, &rx_packet, temp_table_len);
	if (ret != 0) {
		goto unlock;
	}

	ret = r502a_validate_rx_packet(&rx_packet);
	if (ret != 0) {
		goto unlock;
	}

	if (rx_packet.buf[R502A_CC_IDX] == R502A_OK) {
		LOG_DBG("Read success");
	} else {
		LOG_ERR("R502A template table get error");
		ret = -EIO;
		goto unlock;
	}

	for (int group_idx = 0; group_idx < R502A_TEMP_TABLE_BUF_SIZE; group_idx++) {
		uint8_t group = rx_packet.data[group_idx + 1];

		/* if group is all occupied */
		if (group == 0xff) {
			continue;
		}

		val->val1 = (group_idx * 8) + find_lsb_set(~group) - 1;
		goto unlock;
	}

unlock:
	k_mutex_unlock(&drv_data->lock);
	return ret;
}

static int fps_get_image(const struct device *dev)
{
	struct grow_r502a_data *drv_data = dev->data;
	union r502a_packet rx_packet = {0};
	char const get_img_len = 1;
	int ret = 0;

	struct r502a_led_params led_ctrl = {
		.ctrl_code = R502A_LED_CTRL_BREATHING,
		.color_idx = R502A_LED_COLOR_BLUE,
		.speed = R502A_LED_SPEED_HALF,
		.cycle = 0x01,
	};

	union r502a_packet tx_packet = {
		.pid = R502A_COMMAND_PACKET,
		.data = {R502A_GENIMAGE},
	};

	ret = transceive_packet(dev, &tx_packet, &rx_packet, get_img_len);
	if (ret != 0) {
		return ret;
	}

	ret = r502a_validate_rx_packet(&rx_packet);
	if (ret != 0) {
		return ret;
	}

	if (rx_packet.buf[R502A_CC_IDX] == R502A_OK) {
		fps_led_control(dev, &led_ctrl);
		LOG_DBG("Image taken");
	} else {
		led_ctrl.ctrl_code = R502A_LED_CTRL_ON_ALWAYS;
		led_ctrl.color_idx = R502A_LED_COLOR_RED;
		fps_led_control(dev, &led_ctrl);
		LOG_ERR("Error getting image 0x%X", rx_packet.buf[R502A_CC_IDX]);
		return -EIO;
	}

	return 0;
}

static int fps_image_to_char(const struct device *dev, uint8_t char_buf_idx)
{
	struct grow_r502a_data *drv_data = dev->data;
	union r502a_packet rx_packet = {0};
	char const img_to_char_len = 2;
	int ret = 0;

	union r502a_packet tx_packet = {
		.pid = R502A_COMMAND_PACKET,
		.data = {R502A_IMAGE2TZ, char_buf_idx}
	};

	ret = transceive_packet(dev, &tx_packet, &rx_packet, img_to_char_len);
	if (ret != 0) {
		return ret;
	}

	ret = r502a_validate_rx_packet(&rx_packet);
	if (ret != 0) {
		return ret;
	}

	if (rx_packet.buf[R502A_CC_IDX] == R502A_OK) {
		LOG_DBG("Image converted");
	} else {
		LOG_ERR("Error converting image 0x%X", rx_packet.buf[R502A_CC_IDX]);
		return -EIO;
	}

	return 0;
}

static int fps_create_model(const struct device *dev)
{
	struct grow_r502a_data *drv_data = dev->data;
	union r502a_packet rx_packet = {0};
	char const create_model_len = 1;
	int ret = 0;

	union r502a_packet tx_packet = {
		.pid = R502A_COMMAND_PACKET,
		.data = {R502A_REGMODEL}
	};

	ret = transceive_packet(dev, &tx_packet, &rx_packet, create_model_len);
	if (ret != 0) {
		return ret;
	}

	ret = r502a_validate_rx_packet(&rx_packet);
	if (ret != 0) {
		return ret;
	}

	if (rx_packet.buf[R502A_CC_IDX] == R502A_OK) {
		LOG_DBG("Model Created");
	} else {
		LOG_ERR("Error creating model 0x%X", rx_packet.buf[R502A_CC_IDX]);
		return -EIO;
	}

	return 0;
}

static int fps_store_model(const struct device *dev, uint16_t id)
{
	struct grow_r502a_data *drv_data = dev->data;
	union r502a_packet rx_packet = {0};
	char const store_model_len = 4;
	int ret = 0;

	struct r502a_led_params led_ctrl = {
		.ctrl_code = R502A_LED_CTRL_BREATHING,
		.color_idx = R502A_LED_COLOR_BLUE,
		.speed = R502A_LED_SPEED_HALF,
		.cycle = 0x01,
	};

	union r502a_packet tx_packet = {
		.pid = R502A_COMMAND_PACKET,
		.data = {R502A_STORE, R502A_CHAR_BUF_1}
	};
	sys_put_be16(id, &tx_packet.data[2]);

	k_mutex_lock(&drv_data->lock, K_FOREVER);

	ret = transceive_packet(dev, &tx_packet, &rx_packet, store_model_len);
	if (ret != 0) {
		goto unlock;
	}

	ret = r502a_validate_rx_packet(&rx_packet);
	if (ret != 0) {
		goto unlock;
	}

	if (rx_packet.buf[R502A_CC_IDX] == R502A_OK) {
		led_ctrl.color_idx = R502A_LED_COLOR_BLUE;
		led_ctrl.ctrl_code = R502A_LED_CTRL_FLASHING;
		led_ctrl.cycle = 0x03;
		fps_led_control(dev, &led_ctrl);
		LOG_INF("Fingerprint stored! at ID #%d", id);
	} else {
		LOG_ERR("Error storing model 0x%X", rx_packet.buf[R502A_CC_IDX]);
		ret = -EIO;
	}
unlock:
	k_mutex_unlock(&drv_data->lock);
	return ret;
}

static int fps_delete_model(const struct device *dev, uint16_t id, uint16_t count)
{
	struct grow_r502a_data *drv_data = dev->data;
	union r502a_packet rx_packet = {0};
	char const delete_model_len = 5;
	int ret = 0;

	union r502a_packet tx_packet = {
		.pid = R502A_COMMAND_PACKET,
		.data = {R502A_DELETE}
	};
	sys_put_be16(id, &tx_packet.data[1]);
	sys_put_be16(count + R502A_DELETE_COUNT_OFFSET, &tx_packet.data[3]);

	k_mutex_lock(&drv_data->lock, K_FOREVER);

	ret = transceive_packet(dev, &tx_packet, &rx_packet, delete_model_len);
	if (ret != 0) {
		goto unlock;
	}

	ret = r502a_validate_rx_packet(&rx_packet);
	if (ret != 0) {
		goto unlock;
	}

	if (rx_packet.buf[R502A_CC_IDX] == R502A_OK) {
		LOG_INF("Fingerprint Deleted from ID #%d to #%d", id, (id + count));
	} else {
		LOG_ERR("Error deleting image 0x%X", rx_packet.buf[R502A_CC_IDX]);
		ret = -EIO;
	}
unlock:
	k_mutex_unlock(&drv_data->lock);
	return ret;
}

static int fps_empty_db(const struct device *dev)
{
	struct grow_r502a_data *drv_data = dev->data;
	union r502a_packet rx_packet = {0};
	char const empty_db_len = 1;
	int ret = 0;

	union r502a_packet tx_packet = {
		.pid = R502A_COMMAND_PACKET,
		.data = {R502A_EMPTYLIBRARY}
	};

	k_mutex_lock(&drv_data->lock, K_FOREVER);

	ret = transceive_packet(dev, &tx_packet, &rx_packet, empty_db_len);
	if (ret != 0) {
		goto unlock;
	}

	ret = r502a_validate_rx_packet(&rx_packet);
	if (ret != 0) {
		goto unlock;
	}

	if (rx_packet.buf[R502A_CC_IDX] == R502A_OK) {
		LOG_INF("Emptied Fingerprint Library");
	} else {
		LOG_ERR("Error emptying fingerprint library 0x%X",
					rx_packet.buf[R502A_CC_IDX]);
		ret = -EIO;
	}

unlock:
	k_mutex_unlock(&drv_data->lock);
	return ret;
}

static int fps_search(const struct device *dev, struct sensor_value *val)
{
	struct grow_r502a_data *drv_data = dev->data;
	union r502a_packet rx_packet = {0};
	char const search_len = 6;
	int ret = 0;

	struct r502a_led_params led_ctrl = {
		.ctrl_code = R502A_LED_CTRL_BREATHING,
		.color_idx = R502A_LED_COLOR_BLUE,
		.speed = R502A_LED_SPEED_HALF,
		.cycle = 0x01,
	};

	union r502a_packet tx_packet = {
		.pid = R502A_COMMAND_PACKET,
		.data = {R502A_SEARCH, R502A_CHAR_BUF_1}
	};
	sys_put_be16(R02A_LIBRARY_START_IDX, &tx_packet.data[2]);
	sys_put_be16(R502A_DEFAULT_CAPACITY, &tx_packet.data[4]);

	k_mutex_lock(&drv_data->lock, K_FOREVER);

	ret = transceive_packet(dev, &tx_packet, &rx_packet, search_len);
	if (ret != 0) {
		goto unlock;
	}

	ret = r502a_validate_rx_packet(&rx_packet);
	if (ret != 0) {
		goto unlock;
	}

	if (rx_packet.buf[R502A_CC_IDX] == R502A_OK) {
		led_ctrl.ctrl_code = R502A_LED_CTRL_FLASHING;
		led_ctrl.color_idx = R502A_LED_COLOR_PURPLE;
		led_ctrl.cycle = 0x01;
		fps_led_control(dev, &led_ctrl);
		val->val1 = sys_get_be16(&rx_packet.data[1]);
		val->val2 = sys_get_be16(&rx_packet.data[3]);
		LOG_INF("Found a matching print! at ID #%d", val->val1);
	} else if (rx_packet.buf[R502A_CC_IDX] == R502A_NOT_FOUND_CC) {
		led_ctrl.ctrl_code = R502A_LED_CTRL_BREATHING;
		led_ctrl.color_idx = R502A_LED_COLOR_RED;
		led_ctrl.cycle = 0x02;
		fps_led_control(dev, &led_ctrl);
		LOG_ERR("Did not find a match");
		ret = -ENOENT;
	} else {
		led_ctrl.ctrl_code = R502A_LED_CTRL_ON_ALWAYS;
		led_ctrl.color_idx = R502A_LED_COLOR_RED;
		fps_led_control(dev, &led_ctrl);
		LOG_ERR("Error searching for image 0x%X", rx_packet.buf[R502A_CC_IDX]);
		ret = -EIO;
	}
unlock:
	k_mutex_unlock(&drv_data->lock);
	return ret;
}

static int fps_load_template(const struct device *dev, uint16_t id)
{
	struct grow_r502a_data *drv_data = dev->data;
	union r502a_packet rx_packet = {0};
	char const load_tmp_len = 4;
	int ret = 0;

	union r502a_packet tx_packet = {
		.pid = R502A_COMMAND_PACKET,
		.data = {R502A_LOAD, R502A_CHAR_BUF_1}
	};
	sys_put_be16(id, &tx_packet.data[2]);

	k_mutex_lock(&drv_data->lock, K_FOREVER);

	ret = transceive_packet(dev, &tx_packet, &rx_packet, load_tmp_len);
	if (ret != 0) {
		goto unlock;
	}

	ret = r502a_validate_rx_packet(&rx_packet);
	if (ret != 0) {
		goto unlock;
	}

	if (rx_packet.buf[R502A_CC_IDX] == R502A_OK) {
		LOG_DBG("Load template data from id #%d to Char_buffer2", id);
	} else if (rx_packet.buf[R502A_CC_IDX] == R502A_INVALID_TEMPLATE) {
		ret = -EINVAL;
	} else {
		LOG_ERR("Error Loading template 0x%X",
					rx_packet.buf[R502A_CC_IDX]);
		ret = -EIO;
	}

unlock:
	k_mutex_unlock(&drv_data->lock);
	return ret;
}

static int fps_match_templates(const struct device *dev, struct sensor_value *val)
{
	struct grow_r502a_data *drv_data = dev->data;
	union r502a_packet rx_packet = {0};
	char const match_templates_len = 1;
	int ret = 0;

	struct r502a_led_params led_ctrl = {
		.ctrl_code = R502A_LED_CTRL_BREATHING,
		.color_idx = R502A_LED_COLOR_BLUE,
		.speed = R502A_LED_SPEED_HALF,
		.cycle = 0x01,
	};

	union r502a_packet tx_packet = {
		.pid = R502A_COMMAND_PACKET,
		.data = {R502A_MATCH}
	};

	k_mutex_lock(&drv_data->lock, K_FOREVER);

	ret = transceive_packet(dev, &tx_packet, &rx_packet, match_templates_len);
	if (ret != 0) {
		goto unlock;
	}

	ret = r502a_validate_rx_packet(&rx_packet);
	if (ret != 0) {
		goto unlock;
	}

	if (rx_packet.buf[R502A_CC_IDX] == R502A_OK) {
		fps_led_control(dev, &led_ctrl);
		val->val1 = R502A_FINGER_MATCH_FOUND;
		val->val2 = sys_get_be16(&rx_packet.data[1]);
		LOG_INF("Fingerprint matched with a score %d", val->val2);
	} else if (rx_packet.buf[R502A_CC_IDX] == R502A_NOT_MATCH_CC) {
		val->val1 = R502A_FINGER_MATCH_NOT_FOUND;
		LOG_ERR("Fingerprint match not found");
		ret = -ENOENT;
	} else {
		led_ctrl.ctrl_code = R502A_LED_CTRL_ON_ALWAYS;
		led_ctrl.color_idx = R502A_LED_COLOR_RED;
		fps_led_control(dev, &led_ctrl);
		LOG_ERR("Error Matching templates 0x%X",
					rx_packet.buf[R502A_CC_IDX]);
		ret = -EIO;
	}
unlock:
	k_mutex_unlock(&drv_data->lock);
	return ret;
}

static int fps_capture(const struct device *dev)
{
	struct grow_r502a_data *drv_data = dev->data;
	int ret;

	k_mutex_lock(&drv_data->lock, K_FOREVER);

	ret = fps_get_image(dev);
	if (ret != 0) {
		goto unlock;
	}

	ret = fps_image_to_char(dev, R502A_CHAR_BUF_1);
	if (ret != 0) {
		goto unlock;
	}

	ret = fps_get_image(dev);
	if (ret != 0) {
		goto unlock;
	}

	ret = fps_image_to_char(dev, R502A_CHAR_BUF_2);
	if (ret != 0) {
		goto unlock;
	}

unlock:
	k_mutex_unlock(&drv_data->lock);
	return ret;
}

/**
 * @brief	upload template from sensor device's RAM buffer 1 to controller.
 *
 * @result	temp->data	holds the template to be uploaded to controller.
 *		temp->len	holds the length of the template.
 */
static int fps_upload_char_buf(const struct device *dev, struct sensor_value_ex *temp)
{
	struct grow_r502a_data *drv_data = dev->data;
	union r502a_packet rx_packet = {0};
	char const upload_temp_len = 2;
	int ret = 0, idx = 0;

	if (!temp->data) {
		LOG_ERR("Invalid temp data");
		return -EINVAL;
	}

	union r502a_packet tx_packet = {
		.pid = R502A_COMMAND_PACKET,
		.data = {R502A_UPCHAR, R502A_CHAR_BUF_1}
	};

	k_mutex_lock(&drv_data->lock, K_FOREVER);

	ret = transceive_packet(dev, &tx_packet, &rx_packet, upload_temp_len);
	if (ret != 0) {
		goto unlock;
	}

	ret = r502a_validate_rx_packet(&rx_packet);
	if (ret != 0) {
		goto unlock;
	}

	if (rx_packet.buf[R502A_CC_IDX] == R502A_OK) {
		LOG_DBG("Upload to host controller");
	} else {
		LOG_ERR("Error uploading template 0x%X",
					rx_packet.buf[R502A_CC_IDX]);
		ret = -EIO;
		goto unlock;
	}

	do {
		ret = transceive_packet(dev, NULL, &rx_packet, 0);
		if (ret != 0) {
			goto unlock;
		}

		ret = r502a_validate_rx_packet(&rx_packet);
		if (ret != 0) {
			goto unlock;
		}

		memcpy(&temp->data[idx], &rx_packet.data,
				sys_be16_to_cpu(rx_packet.len) - R502A_CHECKSUM_LEN);
		idx += sys_be16_to_cpu(rx_packet.len) - R502A_CHECKSUM_LEN;
	} while (rx_packet.pid != R502A_END_DATA_PACKET);

	temp->len = idx;

unlock:
	k_mutex_unlock(&drv_data->lock);
	return ret;
}

/**
 * @brief	download template from controller to sensor device's RAM buffer.
 */
static int fps_download_char_buf(const struct device *dev, uint8_t char_buf_id,
						const struct sensor_value_ex *temp)
{
	struct grow_r502a_data *drv_data = dev->data;
	union r502a_packet rx_packet = {0};
	char const down_temp_len = 2;
	int ret = 0, i = 0;

	char_buf_id = (char_buf_id > R502A_CHAR_BUF_2) ? R502A_CHAR_BUF_2 : R502A_CHAR_BUF_1;

	if (!temp->data || (temp->len < R502A_TEMPLATE_MAX_SIZE)) {
		LOG_ERR("Invalid temp data");
		return -EINVAL;
	}

	union r502a_packet tx_packet = {
		.pid = R502A_COMMAND_PACKET,
		.data = {R502A_DOWNCHAR, char_buf_id}
	};

	k_mutex_lock(&drv_data->lock, K_FOREVER);

	ret = transceive_packet(dev, &tx_packet, &rx_packet, down_temp_len);
	if (ret != 0) {
		goto unlock;
	}

	ret = r502a_validate_rx_packet(&rx_packet);
	if (ret != 0) {
		goto unlock;
	}

	if (rx_packet.buf[R502A_CC_IDX] == R502A_OK) {
		LOG_DBG("Download to R502A sensor");
	} else {
		LOG_ERR("Error downloading template 0x%X",
					rx_packet.buf[R502A_CC_IDX]);
		ret = -EIO;
		goto unlock;
	}

	while (i < (R502A_TEMPLATE_MAX_SIZE - R502A_DOWNLOADABLE_LEN)) {
		tx_packet.pid = R502A_DATA_PACKET;
		memcpy(tx_packet.data, &temp->data[i], R502A_DOWNLOADABLE_LEN);

		ret = transceive_packet(dev, &tx_packet, NULL, R502A_DOWNLOADABLE_LEN);
		if (ret != 0) {
			goto unlock;
		}

		i += R502A_DOWNLOADABLE_LEN;
	}

	memcpy(tx_packet.data, &temp->data[i], (R502A_TEMPLATE_MAX_SIZE - i));
	tx_packet.pid = R502A_END_DATA_PACKET;
	ret = transceive_packet(dev, &tx_packet, NULL, (R502A_TEMPLATE_MAX_SIZE - i));

unlock:
	k_mutex_unlock(&drv_data->lock);
	return ret;
}

static int fps_init(const struct device *dev)
{
	struct grow_r502a_data *drv_data = dev->data;
	int ret;

	struct r502a_led_params led_ctrl = {
		.ctrl_code = R502A_LED_CTRL_FLASHING,
		.color_idx = R502A_LED_COLOR_PURPLE,
		.speed = R502A_LED_SPEED_HALF,
		.cycle = 0x02,
	};

	k_mutex_lock(&drv_data->lock, K_FOREVER);

	ret = fps_verify_password(dev);
	if (ret != 0) {
		goto unlock;
	}

	ret = fps_led_control(dev, &led_ctrl);

unlock:
	k_mutex_unlock(&drv_data->lock);
	return ret;
}

static int grow_r502a_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct grow_r502a_data *drv_data = dev->data;
	int ret;

	k_mutex_lock(&drv_data->lock, K_FOREVER);
	ret = fps_get_template_count(dev);
	k_mutex_unlock(&drv_data->lock);

	return ret;
}

static int grow_r502a_channel_get(const struct device *dev, enum sensor_channel chan,
				  struct sensor_value *val)
{
	struct grow_r502a_data *drv_data = dev->data;

	if ((enum sensor_channel_grow_r502a)chan == SENSOR_CHAN_FINGERPRINT) {
		val->val1 = drv_data->template_count;
	} else {
		LOG_ERR("Invalid channel");
		return -EINVAL;
	}

	return 0;
}

static int grow_r502a_attr_set(const struct device *dev, enum sensor_channel chan,
			       enum sensor_attribute attr, const struct sensor_value *val)
{
	struct grow_r502a_data *drv_data = dev->data;

	if ((enum sensor_channel_grow_r502a)chan != SENSOR_CHAN_FINGERPRINT) {
		LOG_ERR("Channel not supported");
		return -ENOTSUP;
	}

	switch ((enum sensor_attribute_grow_r502a)attr) {
	case SENSOR_ATTR_R502A_CAPTURE:
		return fps_capture(dev);
	case SENSOR_ATTR_R502A_TEMPLATE_CREATE:
		return fps_create_model(dev);
	case SENSOR_ATTR_R502A_RECORD_ADD:
		return fps_store_model(dev, val->val1);
	case SENSOR_ATTR_R502A_RECORD_DEL:
		return fps_delete_model(dev, val->val1, val->val2);
	case SENSOR_ATTR_R502A_RECORD_EMPTY:
		return fps_empty_db(dev);
	case SENSOR_ATTR_R502A_RECORD_LOAD:
		return fps_load_template(dev, val->val1);
	case SENSOR_ATTR_R502A_DEVICE_LED: {
		int ret = 0;
		struct r502a_led_params led_ctrl;
		sys_put_be32(val->val1, (uint8_t *)&led_ctrl);
		k_mutex_lock(&drv_data->lock, K_FOREVER);
		ret = fps_led_control(dev, &led_ctrl);
		if (ret != 0) {
			k_mutex_unlock(&drv_data->lock);
			return -EIO;
		}
		k_mutex_unlock(&drv_data->lock);
		return 0;
	}
	case SENSOR_ATTR_R502A_SYS_PARAM:
		return fps_set_sys_param(dev, val);
	case SENSOR_ATTR_R502A_DOWNLOAD:
		return fps_download_char_buf(dev, val->val1, &val->ex);
	default:
		LOG_ERR("Sensor attribute not supported");
		return -ENOTSUP;
	}

}

static int grow_r502a_attr_get(const struct device *dev, enum sensor_channel chan,
			       enum sensor_attribute attr, struct sensor_value *val)
{
	int ret;
	struct grow_r502a_data *drv_data = dev->data;

	if ((enum sensor_channel_grow_r502a)chan != SENSOR_CHAN_FINGERPRINT) {
		LOG_ERR("Channel not supported");
		return -ENOTSUP;
	}

	switch ((enum sensor_attribute_grow_r502a)attr) {
	case SENSOR_ATTR_R502A_RECORD_FIND:
		ret = fps_search(dev, val);
		break;
	case SENSOR_ATTR_R502A_RECORD_FREE_IDX:
		ret = fps_read_template_table(dev, val);
		break;
	case SENSOR_ATTR_R502A_COMPARE:
		ret = fps_match_templates(dev, val);
		break;
	case SENSOR_ATTR_R502A_SYS_PARAM:
		ret = fps_read_sys_param(dev, (struct r502a_sys_param *)val->ex.data);
		break;
	case SENSOR_ATTR_R502A_UPLOAD:
		ret = fps_upload_char_buf(dev, &val->ex);
		break;
	default:
		LOG_ERR("Sensor attribute not supported");
		ret = -ENOTSUP;
		break;
	}

	return ret;
}

static void grow_r502a_uart_flush(const struct device *dev)
{
	uint8_t c;

	while (uart_fifo_read(dev, &c, 1) > 0) {
		continue;
	}
}

static int grow_r502a_init(const struct device *dev)
{
	const struct grow_r502a_config *cfg = dev->config;
	struct grow_r502a_data *drv_data = dev->data;
	int ret;

	if (!device_is_ready(cfg->dev)) {
		LOG_ERR("%s: grow_r502a device not ready", dev->name);
		return -ENODEV;
	}

	if (IS_ENABLED(CONFIG_GROW_R502A_GPIO_POWER)) {
		if (!device_is_ready(cfg->vin_gpios.port)) {
			LOG_ERR("GPIO port %s not ready", cfg->vin_gpios.port->name);
			return -ENODEV;
		}

		ret = gpio_pin_configure_dt(&cfg->vin_gpios, GPIO_OUTPUT_ACTIVE);

		if (ret < 0) {
			return ret;
		}

		k_sleep(K_MSEC(R502A_DELAY));

		if (!device_is_ready(cfg->act_gpios.port)) {
			LOG_ERR("GPIO port %s not ready", cfg->act_gpios.port->name);
			return -ENODEV;
		}

		ret = gpio_pin_configure_dt(&cfg->act_gpios, GPIO_OUTPUT_ACTIVE);
		if (ret < 0) {
			return ret;
		}

		k_sleep(K_MSEC(R502A_DELAY));
	}

	grow_r502a_uart_flush(cfg->dev);

	k_mutex_init(&drv_data->lock);
	k_sem_init(&drv_data->uart_rx_sem, 0, 1);
	k_sem_init(&drv_data->uart_tx_sem, 0, 1);

	uart_irq_callback_user_data_set(cfg->dev, uart_cb_handler, (void *)dev);

	uart_irq_rx_disable(cfg->dev);
	uart_irq_tx_disable(cfg->dev);

#ifdef CONFIG_GROW_R502A_TRIGGER
	ret = grow_r502a_init_interrupt(dev);

	if (ret < 0) {
		LOG_ERR("Failed to initialize interrupt!");
		return ret;
	}
#endif

	return fps_init(dev);
}

static const struct sensor_driver_api grow_r502a_api = {
	.sample_fetch = grow_r502a_sample_fetch,
	.channel_get = grow_r502a_channel_get,
	.attr_set = grow_r502a_attr_set,
	.attr_get = grow_r502a_attr_get,
#ifdef CONFIG_GROW_R502A_TRIGGER
	.trigger_set = grow_r502a_trigger_set,
#endif
};

#define GROW_R502A_INIT(index)									\
	static struct grow_r502a_data grow_r502a_data_##index;					\
												\
	static struct grow_r502a_config grow_r502a_config_##index = {				\
		.dev = DEVICE_DT_GET(DT_INST_BUS(index)),					\
		.comm_addr = DT_INST_REG_ADDR(index),						\
		IF_ENABLED(CONFIG_GROW_R502A_GPIO_POWER,					\
		(.vin_gpios = GPIO_DT_SPEC_INST_GET_OR(index, vin_gpios, {}),			\
		 .act_gpios = GPIO_DT_SPEC_INST_GET_OR(index, act_gpios, {}),))			\
		IF_ENABLED(CONFIG_GROW_R502A_TRIGGER,						\
		(.int_gpios = GPIO_DT_SPEC_INST_GET_OR(index, int_gpios, {}),))			\
	};											\
												\
	DEVICE_DT_INST_DEFINE(index, &grow_r502a_init, NULL, &grow_r502a_data_##index,		\
			      &grow_r502a_config_##index, POST_KERNEL,				\
			      CONFIG_SENSOR_INIT_PRIORITY, &grow_r502a_api);

DT_INST_FOREACH_STATUS_OKAY(GROW_R502A_INIT)
