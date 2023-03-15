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
#include <math.h>

#include <zephyr/drivers/sensor/grow_r502a.h>
#include "grow_r502a.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(GROW_R502A, CONFIG_SENSOR_LOG_LEVEL);

static void transceive_packet(const struct device *dev, union r502a_packet *tx_packet,
				uint8_t *buf, char const data_len)
{
	const struct grow_r502a_config *cfg = dev->config;
	struct grow_r502a_data *drv_data = dev->data;
	uint16_t check_sum, pkg_len;

	pkg_len = data_len + R502A_CHECKSUM_LEN;
	check_sum = pkg_len + tx_packet->pid;

	tx_packet->start = sys_be16_to_cpu(R502A_STARTCODE);
	tx_packet->addr = sys_be32_to_cpu(cfg->comm_addr);
	tx_packet->len = sys_be16_to_cpu(pkg_len);

	for (int i = 0; i < data_len; i++) {
		check_sum += tx_packet->data[i];
	}
	sys_put_be16(check_sum, &tx_packet->buf[data_len + R502A_HEADER_LEN]);

	drv_data->tx_buf.len = pkg_len + R502A_HEADER_LEN;
	drv_data->tx_buf.data = tx_packet->buf;

	drv_data->rx_data_buf.data = buf;

	LOG_HEXDUMP_DBG(drv_data->tx_buf.data, drv_data->tx_buf.len, "TX");

	uart_irq_rx_disable(cfg->dev);
	uart_irq_tx_enable(cfg->dev);

	k_sem_take(&drv_data->uart_rx_sem, K_FOREVER);
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
			drv_data->rx_data_buf.len = 0;
			if (drv_data->rx_data_buf.data == NULL) {
				k_sem_give(&drv_data->uart_rx_sem);
				break;
			}
			uart_irq_rx_enable(config->dev);
			break;
		}
	}
}

static int validate_header(union r502a_packet *rx_packet)
{
	if (rx_packet->start == R502A_STARTCODE) {
		LOG_DBG("startcode matched 0x%X", rx_packet->start);
	} else {
		LOG_DBG("startcode didn't match 0x%X", rx_packet->start);
		return -EINVAL;
	}

	if (rx_packet->addr == R502A_DEFAULT_ADDRESS) {
		LOG_DBG("Address matched 0x%X", rx_packet->addr);
	} else {
		LOG_DBG("Address didn't match 0x%X", rx_packet->addr);
		return -EINVAL;
	}

	switch (rx_packet->pid) {
	case 0x2:
		LOG_DBG("Data Packet Received 0x%X", rx_packet->pid);
		break;
	case 0x8:
		LOG_DBG("End of Data Packet Received 0x%X", rx_packet->pid);
		break;
	case 0x7:
		LOG_DBG("Acknowledgment Packet Received 0x%X", rx_packet->pid);
		break;
	default:
		LOG_ERR("Error Package ID 0x%X", rx_packet->pid);
		return -EINVAL;
	}

	return 0;
}

static void r502a_verify_crc(uint16_t *crc_bytes, struct grow_r502a_data *drv_data)
{
	uint16_t calc_bytes = 0;
	static int previous;

	if (!drv_data->returns_data_pkt) {
		previous = 0;
	}

	calc_bytes += drv_data->rx_hdr_pkt.pid + drv_data->rx_hdr_pkt.len;

	for (int i = previous; i < drv_data->rx_data_buf.len; i++) {
		calc_bytes += drv_data->rx_data_buf.data[i];
	}

	if (*crc_bytes == calc_bytes) {
		LOG_DBG("calc 0x%x crc_bytes 0x%x", calc_bytes, *crc_bytes);
	} else {
		LOG_ERR("calc 0x%x crc_bytes 0x%x", calc_bytes, *crc_bytes);
		return;
	}

	if (drv_data->returns_data_pkt) {
		previous = drv_data->rx_data_buf.len;
	}
}

static void uart_cb_rx_handler(const struct device *dev)
{
	const struct grow_r502a_config *config = dev->config;
	struct grow_r502a_data *drv_data = dev->data;
	int ret, len = 0;
	int pkt_sz = drv_data->rx_hdr_pkt.len;
	int offset = drv_data->rx_len;
	uint16_t crc_bytes = 0;

	if (!drv_data->rx_header) {
		len = uart_fifo_read(config->dev, &drv_data->rx_hdr_pkt.buf[offset],
								R502A_HEADER_LEN - offset);
		offset += len;
		drv_data->rx_len = offset;

		if (offset < R502A_HEADER_LEN) {
			return;
		}

		drv_data->rx_hdr_pkt.start = sys_be16_to_cpu(drv_data->rx_hdr_pkt.start);
		drv_data->rx_hdr_pkt.addr = sys_be32_to_cpu(drv_data->rx_hdr_pkt.addr);
		drv_data->rx_hdr_pkt.len = sys_be16_to_cpu(drv_data->rx_hdr_pkt.len);

		ret = validate_header(&drv_data->rx_hdr_pkt);
		if (ret != 0) {
			return;
		}

		pkt_sz = drv_data->rx_hdr_pkt.len;
		drv_data->rx_header = true;
		drv_data->rx_len = 0;
		offset = 0;
	} else {
		len = uart_fifo_read(config->dev,
				     &drv_data->rx_data_buf.data[offset+drv_data->rx_data_buf.len],
				     pkt_sz - offset);
		offset += len;
		drv_data->rx_len = offset;

		if (offset < pkt_sz) {
			return;
		}

		drv_data->rx_data_buf.len += offset - 2;

		crc_bytes = sys_get_be16(&drv_data->rx_data_buf.data[drv_data->rx_data_buf.len]);
		r502a_verify_crc(&crc_bytes, drv_data);

		drv_data->rx_header = false;
		drv_data->rx_len = 0;
		offset = 0;

		if (!uart_irq_rx_ready(config->dev)) {
			k_sem_give(&drv_data->uart_rx_sem);
			return;
		}
	}
}

static void uart_cb_handler(const struct device *uart_dev, void *user_data)
{
	const struct device *dev = user_data;
	struct grow_r502a_data *drv_data = dev->data;
	int len = 0;
	int pkt_sz = drv_data->rx_hdr_pkt.len;
	int offset = drv_data->rx_len;

	if ((uart_irq_update(uart_dev) > 0) && (uart_irq_is_pending(uart_dev) > 0)) {
		if (uart_irq_tx_ready(uart_dev)) {
			uart_cb_tx_handler(dev);
		}

		if (uart_irq_rx_ready(uart_dev)) {
			uart_cb_rx_handler(dev);
		}
	}
}

static int fps_set_sys_param(const struct device *dev, const struct sensor_value *val)
{
	struct grow_r502a_data *drv_data = dev->data;
	uint8_t rx_buf;
	int ret = 0;
	char const set_sys_param_len = 3;

	union r502a_packet tx_packet = {
		.pid = R502A_COMMAND_PACKET,
		.data = { R502A_SETSYSPARAM, val->val1, val->val2}
	};

	k_mutex_lock(&drv_data->lock, K_FOREVER);

	transceive_packet(dev, &tx_packet, &rx_buf, set_sys_param_len);

	if (rx_buf == R502A_OK) {
		LOG_DBG("R502A set system parameter success");
	} else {
		LOG_ERR("R502A set system parameter error %d", rx_buf);
		ret = -EIO;
		goto unlock;
	}
unlock:
	k_mutex_unlock(&drv_data->lock);
	return ret;
}

static uint32_t r502a_pow(uint32_t base, uint32_t exp)
{
	int ret = 1;

	while (exp--) {
		ret *= base;
	}

	return ret;
}

static int fps_read_sys_param(const struct device *dev, struct r502a_sys_param *val)
{
	struct grow_r502a_data *drv_data = dev->data;
	uint8_t rx_buf[19] = {0};
	uint8_t offset = 0;
	int ret = 0;
	char const set_sys_param_len = 1;

	union r502a_packet tx_packet = {
		.pid = R502A_COMMAND_PACKET,
		.data = {R502A_READSYSPARAM}
	};

	k_mutex_lock(&drv_data->lock, K_FOREVER);

	transceive_packet(dev, &tx_packet, rx_buf, set_sys_param_len);

	if (rx_buf[R502A_CC_IDX] == R502A_OK) {
		LOG_DBG("R502A read system parameter success");
	} else {
		LOG_ERR("R502A set system parameter error %d", rx_buf[R502A_CC_IDX]);
		ret = -EIO;
		goto unlock;
	}

	val->status_reg = sys_get_be16(&rx_buf[1]);
	val->system_id = sys_get_be16(&rx_buf[3]);
	val->lib_size = sys_get_be16(&rx_buf[5]);
	val->sec_level = sys_get_be16(&rx_buf[7]);
	val->addr = sys_get_be32(&rx_buf[9]);
	offset = (sys_get_be16(&rx_buf[13]) + 5);
	val->data_pkt_size = r502a_pow(2, offset);
	val->baud = sys_get_be16(&rx_buf[15]) * 9600;

unlock:
	k_mutex_unlock(&drv_data->lock);
	return ret;
}

static int fps_led_control(const struct device *dev, struct r502a_led_params *led_control)
{
	struct grow_r502a_data *drv_data = dev->data;
	uint8_t rx_buf;
	char const led_ctrl_len = 5;

	union r502a_packet tx_packet = {
		.pid = R502A_COMMAND_PACKET,
		.data = { R502A_LED_CONFIG, led_control->ctrl_code,
				led_control->speed, led_control->color_idx, led_control->cycle}
	};

	transceive_packet(dev, &tx_packet, &rx_buf, led_ctrl_len);

	if (rx_buf == R502A_OK) {
		LOG_DBG("R502A LED ON");
		k_sleep(K_MSEC(R502A_DELAY));
	} else {
		LOG_ERR("R502A LED control error %d", rx_buf);
		return -EIO;
	}

	return 0;
}

static int fps_verify_password(const struct device *dev)
{
	struct grow_r502a_data *drv_data = dev->data;
	uint8_t rx_buf;
	char const verify_pwd_len = 5;

	union r502a_packet tx_packet = {
		.pid = R502A_COMMAND_PACKET,
		.data[0] = R502A_VERIFYPASSWORD,
	};

	sys_put_be32(R502A_DEFAULT_PASSWORD, &tx_packet.data[1]);

	transceive_packet(dev, &tx_packet, &rx_buf, verify_pwd_len);

	if (rx_buf == R502A_OK) {
		LOG_DBG("Correct password, R502A verified");
	} else {
		LOG_ERR("Package receive error 0x%X", rx_buf);
		return -EIO;
	}

	return 0;
}

static int fps_upload_char_buf(const struct device *dev, uint8_t char_buf_id,
						struct sensor_value_ex *temp)
{
	struct grow_r502a_data *drv_data = dev->data;
	uint8_t *rx_buf = NULL;
	char const upload_temp_len = 2;
	int ret, data_idx = 0;

	char_buf_id = (char_buf_id > R502A_CHAR_BUF_2) ? R502A_CHAR_BUF_2 : R502A_CHAR_BUF_1;

	union r502a_packet tx_packet = {
		.pid = R502A_COMMAND_PACKET,
		.data = {R502A_UPCHAR, char_buf_id}
	};

	drv_data->returns_data_pkt = true;

	rx_buf = k_malloc(R502A_MAX_BUF_SIZE);
	if (!rx_buf) {
		LOG_ERR("memory allocation failure!");
		return -ENOMEM;
	}

	k_mutex_lock(&drv_data->lock, K_FOREVER);

	transceive_packet(dev, &tx_packet, rx_buf, upload_temp_len);

	if (rx_buf[R502A_CC_IDX] == R502A_OK) {
		LOG_DBG("Upload to host controller");
	} else {
		LOG_ERR("Error uploading template 0x%X", rx_buf[R502A_CC_IDX]);
		ret = -EIO;
		goto unlock;
	}

	temp->data = &drv_data->rx_data_buf.data[1];
	temp->len = drv_data->rx_data_buf.len - 1;
	drv_data->returns_data_pkt = false;

unlock:
	k_mutex_unlock(&drv_data->lock);
	return ret;
}

static int fps_download_char_buf(const struct device *dev, struct sensor_value_ex *temp,
						uint8_t char_buf_idx)
{
	struct grow_r502a_data *drv_data = dev->data;
	char const down_temp_len = 2;
	uint8_t rx_buf;
	int ret, i = 0;

	union r502a_packet tx_packet = {
		.pid = R502A_COMMAND_PACKET,
		.data = {R502A_DOWNCHAR, char_buf_idx}
	};

	k_mutex_lock(&drv_data->lock, K_FOREVER);

	transceive_packet(dev, &tx_packet, &rx_buf, down_temp_len);

	if (rx_buf == R502A_OK) {
		LOG_DBG("Download to R502A sensor");
	} else {
		LOG_ERR("Error downloading template 0x%X", rx_buf);
		ret = -EIO;
		goto unlock;
	}

	while (i < (R502A_TEMPLATE_MAX_SIZE - CONFIG_GROW_R502A_DATA_PKT_SIZE)) {
		tx_packet.pid = R502A_DATA_PACKET;
		memcpy(tx_packet.data, &temp->data[i], (char const)CONFIG_GROW_R502A_DATA_PKT_SIZE);
		transceive_packet(dev, &tx_packet, NULL,
						(char const)CONFIG_GROW_R502A_DATA_PKT_SIZE);
		i += CONFIG_GROW_R502A_DATA_PKT_SIZE;
	}

	memcpy(tx_packet.data, &temp->data[i], (char const)CONFIG_GROW_R502A_DATA_PKT_SIZE);
	tx_packet.pid = R502A_END_DATA_PACKET;
	transceive_packet(dev, &tx_packet, NULL, (char const)CONFIG_GROW_R502A_DATA_PKT_SIZE);

unlock:
	k_mutex_unlock(&drv_data->lock);
	return ret;
}

static int fps_get_template_count(const struct device *dev)
{
	struct grow_r502a_data *drv_data = dev->data;
	uint8_t rx_buf[3] = {0};
	char const get_temp_cnt_len = 1;

	union r502a_packet tx_packet = {
		.pid = R502A_COMMAND_PACKET,
		.data = {R502A_TEMPLATECOUNT},
	};

	transceive_packet(dev, &tx_packet, rx_buf, get_temp_cnt_len);

	if (rx_buf[R502A_CC_IDX] == R502A_OK) {
		LOG_DBG("Read success");
		drv_data->template_count = sys_get_be16(&rx_buf[1]);
		LOG_INF("Remaining templates count : %d", drv_data->template_count);
	} else {
		LOG_ERR("R502A template count get error");
		return -EIO;
	}

	return 0;
}

static int fps_read_template_table(const struct device *dev)
{
	struct grow_r502a_data *drv_data = dev->data;
	uint8_t rx_buf[33] = {0};
	char const temp_table_len = 2;
	int ret = 0;

	union r502a_packet tx_packet = {
		.pid = R502A_COMMAND_PACKET,
		.data = {R502A_READTEMPLATEINDEX, 0x00}
	};

	k_mutex_lock(&drv_data->lock, K_FOREVER);

	transceive_packet(dev, &tx_packet, rx_buf, temp_table_len);

	if (rx_buf[R502A_CC_IDX] == R502A_OK) {
		LOG_DBG("Read success");
	} else {
		LOG_ERR("R502A template table get error");
		ret = -EIO;
		goto unlock;
	}

	for (int group_idx = 0; group_idx < R502A_TEMP_TABLE_BUF_SIZE; group_idx++) {
		uint8_t group = rx_buf[group_idx + 1];

		/* if group is all occupied */
		if (group == 0xff) {
			continue;
		}

		drv_data->free_idx = (group_idx * 8) + find_lsb_set(~group) - 1;
		goto unlock;
	}

unlock:
	k_mutex_unlock(&drv_data->lock);
	return ret;
}

static int fps_get_image(const struct device *dev)
{
	struct grow_r502a_data *drv_data = dev->data;
	uint8_t rx_buf;
	char const get_img_len = 1;

	struct r502a_led_params led_ctrl = {
		.ctrl_code = LED_CTRL_BREATHING,
		.color_idx = LED_COLOR_BLUE,
		.speed = LED_SPEED_HALF,
		.cycle = 0x01,
	};

	union r502a_packet tx_packet = {
		.pid = R502A_COMMAND_PACKET,
		.data = {R502A_GENIMAGE},
	};

	transceive_packet(dev, &tx_packet, &rx_buf, get_img_len);

	if (rx_buf == R502A_OK) {
		fps_led_control(dev, &led_ctrl);
		LOG_DBG("Image taken");
	} else {
		led_ctrl.ctrl_code = LED_CTRL_ON_ALWAYS;
		led_ctrl.color_idx = LED_COLOR_RED;
		fps_led_control(dev, &led_ctrl);
		LOG_ERR("Error getting image 0x%X", rx_buf);
		return -EIO;
	}

	return 0;
}

static int fps_image_to_char(const struct device *dev, uint8_t char_buf_idx)
{
	struct grow_r502a_data *drv_data = dev->data;
	uint8_t rx_buf;
	char const img_to_char_len = 2;

	union r502a_packet tx_packet = {
		.pid = R502A_COMMAND_PACKET,
		.data = {R502A_IMAGE2TZ, char_buf_idx}
	};

	transceive_packet(dev, &tx_packet, &rx_buf, img_to_char_len);

	if (rx_buf == R502A_OK) {
		LOG_DBG("Image converted");
	} else {
		LOG_ERR("Error converting image 0x%X", rx_buf);
		return -EIO;
	}

	return 0;
}

static int fps_create_model(const struct device *dev)
{
	struct grow_r502a_data *drv_data = dev->data;
	uint8_t rx_buf;
	char const create_model_len = 1;

	union r502a_packet tx_packet = {
		.pid = R502A_COMMAND_PACKET,
		.data = {R502A_REGMODEL}
	};

	transceive_packet(dev, &tx_packet, &rx_buf, create_model_len);

	if (rx_buf == R502A_OK) {
		LOG_DBG("Model Created");
	} else {
		LOG_ERR("Error creating model 0x%X", rx_buf);
		return -EIO;
	}

	return 0;
}

static int fps_store_model(const struct device *dev, uint16_t id)
{
	struct grow_r502a_data *drv_data = dev->data;
	uint8_t rx_buf;
	char const store_model_len = 4;

	struct r502a_led_params led_ctrl = {
		.ctrl_code = LED_CTRL_BREATHING,
		.color_idx = LED_COLOR_BLUE,
		.speed = LED_SPEED_HALF,
		.cycle = 0x01,
	};

	union r502a_packet tx_packet = {
		.pid = R502A_COMMAND_PACKET,
		.data = {R502A_STORE, R502A_CHAR_BUF_1}
	};
	sys_put_be16(id, &tx_packet.data[2]);

	transceive_packet(dev, &tx_packet, &rx_buf, store_model_len);

	if (rx_buf == R502A_OK) {
		led_ctrl.color_idx = LED_COLOR_BLUE;
		led_ctrl.ctrl_code = LED_CTRL_FLASHING;
		led_ctrl.cycle = 0x03;
		fps_led_control(dev, &led_ctrl);
		LOG_INF("Fingerprint stored! at ID #%d", id);
	} else {
		LOG_ERR("Error storing model 0x%X", rx_buf);
		return -EIO;
	}

	return 0;
}

static int fps_delete_model(const struct device *dev, uint16_t id, uint16_t count)
{
	struct grow_r502a_data *drv_data = dev->data;
	uint8_t rx_buf;
	char const delete_model_len = 5;

	union r502a_packet tx_packet = {
		.pid = R502A_COMMAND_PACKET,
		.data = {R502A_DELETE}
	};
	sys_put_be16(id, &tx_packet.data[1]);
	sys_put_be16(count + R502A_DELETE_COUNT_OFFSET, &tx_packet.data[3]);

	transceive_packet(dev, &tx_packet, &rx_buf, delete_model_len);

	if (rx_buf == R502A_OK) {
		LOG_INF("Fingerprint Deleted from ID #%d to #%d", id, (id + count));
	} else {
		LOG_ERR("Error deleting image 0x%X", rx_buf);
		return -EIO;
	}

	return 0;
}

static int fps_empty_db(const struct device *dev)
{
	struct grow_r502a_data *drv_data = dev->data;
	uint8_t rx_buf;
	char const empty_db_len = 1;
	int ret = 0;

	union r502a_packet tx_packet = {
		.pid = R502A_COMMAND_PACKET,
		.data = {R502A_EMPTYLIBRARY}
	};

	k_mutex_lock(&drv_data->lock, K_FOREVER);

	transceive_packet(dev, &tx_packet, &rx_buf, empty_db_len);

	if (rx_buf == R502A_OK) {
		LOG_INF("Emptied Fingerprint Library");
	} else {
		LOG_ERR("Error emptying fingerprint library 0x%X",
					rx_buf);
		ret = -EIO;
		goto unlock;
	}

unlock:
	k_mutex_unlock(&drv_data->lock);
	return ret;
}

static int fps_search(const struct device *dev, uint8_t char_buf_idx)
{
	struct grow_r502a_data *drv_data = dev->data;
	uint8_t rx_buf[5] = {0};
	char const search_len = 6;

	struct r502a_led_params led_ctrl = {
		.ctrl_code = LED_CTRL_BREATHING,
		.color_idx = LED_COLOR_BLUE,
		.speed = LED_SPEED_HALF,
		.cycle = 0x01,
	};

	union r502a_packet tx_packet = {
		.pid = R502A_COMMAND_PACKET,
		.data = {R502A_SEARCH, char_buf_idx}
	};
	sys_put_be16(R02A_LIBRARY_START_IDX, &tx_packet.data[1]);
	sys_put_be16(R502A_DEFAULT_CAPACITY, &tx_packet.data[3]);

	transceive_packet(dev, &tx_packet, rx_buf, search_len);

	if (rx_buf[R502A_CC_IDX] == R502A_OK) {
		led_ctrl.ctrl_code = LED_CTRL_FLASHING;
		led_ctrl.color_idx = LED_COLOR_PURPLE;
		led_ctrl.cycle = 0x01;
		fps_led_control(dev, &led_ctrl);
		drv_data->finger_id = sys_get_be16(&rx_buf[1]);
		drv_data->matching_score = sys_get_be16(&rx_buf[3]);
		LOG_INF("Found a matching print! at ID #%d", drv_data->finger_id);
	} else if (rx_buf[R502A_CC_IDX] == R502A_NOT_FOUND) {
		led_ctrl.ctrl_code = LED_CTRL_BREATHING;
		led_ctrl.color_idx = LED_COLOR_RED;
		led_ctrl.cycle = 0x02;
		fps_led_control(dev, &led_ctrl);
		LOG_ERR("Did not find a match");
		return -ENOENT;
	} else {
		led_ctrl.ctrl_code = LED_CTRL_ON_ALWAYS;
		led_ctrl.color_idx = LED_COLOR_RED;
		fps_led_control(dev, &led_ctrl);
		LOG_ERR("Error searching for image 0x%X", rx_buf[R502A_CC_IDX]);
		return -EIO;
	}

	return 0;
}

static int fps_load_template(const struct device *dev, uint8_t char_buf_id,
						uint16_t id)
{
	struct grow_r502a_data *drv_data = dev->data;
	uint8_t rx_buf;
	char const load_tmp_len = 4;
	int ret = 0;

	char_buf_id = (char_buf_id > R502A_CHAR_BUF_2) ? R502A_CHAR_BUF_2 : R502A_CHAR_BUF_1;

	union r502a_packet tx_packet = {
		.pid = R502A_COMMAND_PACKET,
		.data = {R502A_LOAD, char_buf_id}
	};
	sys_put_be16(id, &tx_packet.data[2]);

	k_mutex_lock(&drv_data->lock, K_FOREVER);

	transceive_packet(dev, &tx_packet, &rx_buf, load_tmp_len);

	if (rx_buf == R502A_OK) {
		LOG_DBG("Load template data from id #%d to Char_buffer2", id);
	} else {
		LOG_ERR("Error Loading template 0x%X", rx_buf);
		ret = -EIO;
		goto unlock;
	}

unlock:
	k_mutex_unlock(&drv_data->lock);
	return ret;
}

static int fps_match_templates(const struct device *dev)
{
	struct grow_r502a_data *drv_data = dev->data;
	uint8_t rx_buf[3] = {0};
	char const match_templates_len = 1;
	int ret = 0;
	struct r502a_led_params led_ctrl = {
		.ctrl_code = LED_CTRL_FLASHING,
		.color_idx = LED_COLOR_PURPLE,
		.speed = LED_SPEED_HALF,
		.cycle = 0x01,
	};

	union r502a_packet tx_packet = {
		.pid = R502A_COMMAND_PACKET,
		.data = {R502A_MATCH}
	};

	k_mutex_lock(&drv_data->lock, K_FOREVER);

	transceive_packet(dev, &tx_packet, rx_buf, match_templates_len);

	if (rx_buf[R502A_CC_IDX] == R502A_OK) {
		fps_led_control(dev, &led_ctrl);
		drv_data->matching_score = sys_get_be16(&rx_buf[1]);
		LOG_INF("Fingerprint matched with a score %d", drv_data->matching_score);
	} else {
		led_ctrl.ctrl_code = LED_CTRL_ON_ALWAYS;
		led_ctrl.color_idx = LED_COLOR_RED;
		fps_led_control(dev, &led_ctrl);
		LOG_ERR("Error creating model 0x%X", rx_buf[R502A_CC_IDX]);
		ret = -EIO;
		goto unlock;
	}

unlock:
	k_mutex_unlock(&drv_data->lock);
	return ret;
}

static int fps_capture(const struct device *dev, uint8_t char_buf_id)
{
	struct grow_r502a_data *drv_data = dev->data;
	int ret = -1;

	char_buf_id = (char_buf_id > R502A_CHAR_BUF_2) ? R502A_CHAR_BUF_2 : R502A_CHAR_BUF_1;

	k_mutex_lock(&drv_data->lock, K_FOREVER);

	ret = fps_get_image(dev);
	if (ret != 0) {
		goto unlock;
	}

	ret = fps_image_to_char(dev, char_buf_id);

unlock:
	k_mutex_unlock(&drv_data->lock);
	return ret;
}

static int fps_enroll(const struct device *dev, const struct sensor_value *val)
{
	struct grow_r502a_data *drv_data = dev->data;
	int ret = -1;

	if (val->val1 < 0 || val->val1 > R502A_DEFAULT_CAPACITY) {
		LOG_ERR("Invalid ID number");
		return -EINVAL;
	}

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

	ret = fps_create_model(dev);
	if (ret != 0) {
		goto unlock;
	}

	ret = fps_store_model(dev, val->val1);

unlock:
	k_mutex_unlock(&drv_data->lock);
	return ret;
}

static int fps_delete(const struct device *dev, const struct sensor_value *val)
{
	struct grow_r502a_data *drv_data = dev->data;
	int ret = -1;

	k_mutex_lock(&drv_data->lock, K_FOREVER);

	ret = fps_delete_model(dev, val->val1, val->val2);
	if (ret != 0) {
		goto unlock;
	}

	ret = fps_get_template_count(dev);

unlock:
	k_mutex_unlock(&drv_data->lock);
	return ret;
}

static int fps_match(const struct device *dev, struct sensor_value *val)
{
	struct grow_r502a_data *drv_data = dev->data;
	int ret = -1;

	k_mutex_lock(&drv_data->lock, K_FOREVER);

	ret = fps_get_image(dev);
	if (ret != 0) {
		goto unlock;
	}

	ret = fps_image_to_char(dev, R502A_CHAR_BUF_1);
	if (ret != 0) {
		goto unlock;
	}

	ret = fps_search(dev, R502A_CHAR_BUF_1);
	if (ret == 0) {
		val->val1 = drv_data->finger_id;
		val->val2 = drv_data->matching_score;
	}

unlock:
	k_mutex_unlock(&drv_data->lock);
	return ret;
}

static int fps_init(const struct device *dev)
{
	struct grow_r502a_data *drv_data = dev->data;
	int ret;

	struct r502a_led_params led_ctrl = {
		.ctrl_code = LED_CTRL_FLASHING,
		.color_idx = LED_COLOR_PURPLE,
		.speed = LED_SPEED_HALF,
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
	if ((enum sensor_channel_grow_r502a)chan != SENSOR_CHAN_FINGERPRINT) {
		LOG_ERR("Channel not supported");
		return -ENOTSUP;
	}

	switch ((enum sensor_attribute_grow_r502a)attr) {
	case SENSOR_ATTR_R502A_RECORD_ADD:
		return fps_enroll(dev, val);
	case SENSOR_ATTR_R502A_RECORD_DEL:
		return fps_delete(dev, val);
	case SENSOR_ATTR_R502A_RECORD_EMPTY:
		return fps_empty_db(dev);
	case SENSOR_ATTR_R502A_DOWNLOAD:
		return fps_download_char_buf(dev, val->ex, val->val1);
	case SENSOR_ATTR_R502A_RECORD_LOAD:
		return fps_load_template(dev, val->val1, val->val2);
	case SENSOR_ATTR_R502A_RECORD_STORE:
		return fps_store_model(dev, val->val1);
	case SENSOR_ATTR_R502A_CAPTURE:
		return fps_capture(dev, val->val1);
	case SENSOR_ATTR_R502A_DEVICE_LED:
		struct r502a_led_params led_ctrl = {
			.ctrl_code = val[0].val1,
			.color_idx = val[0].val2,
			.speed = val[1].val1,
			.cycle = val[1].val2,
		};
		return fps_led_control(dev, &led_ctrl);
	case SENSOR_ATTR_R502A_SYS_PARAM:
		return fps_set_sys_param(dev, val);
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
		ret = fps_match(dev, val);
		break;
	case SENSOR_ATTR_R502A_RECORD_FREE_IDX:
		ret = fps_read_template_table(dev);
		val->val1 = drv_data->free_idx;
		break;
	case SENSOR_ATTR_R502A_UPLOAD:
		ret = fps_upload_char_buf(dev, val->val1, val->ex);
	case SENSOR_ATTR_R502A_COMPARE:
		ret = fps_match_templates(dev);
		if (!ret) {
			val->val1 = drv_data->matching_score;
		}
		break;
	case SENSOR_ATTR_R502A_SYS_PARAM:
		ret = fps_read_sys_param(dev, (struct r502a_sys_param *)val->ex->data);
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

	uart_irq_callback_user_data_set(cfg->dev, uart_cb_handler, (void *)dev);

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
