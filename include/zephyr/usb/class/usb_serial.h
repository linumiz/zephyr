/*
 * Copyright (c) 2025 Linumiz GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_USB_CLASS_USB_SERIAL_H_
#define ZEPHYR_INCLUDE_USB_CLASS_USB_SERIAL_H_

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/ring_buffer.h>

#define BUF_MAX		512
#define RX_RINGBUF_SIZE BUF_MAX

/**
 * @brief USB Serial API structure
 *
 * Contains function pointers for write and read operations
 */
struct usb_serial_api {
	/**
	 * @brief Write data to USB serial port
	 * @param dev Device pointer
	 * @param buf Data buffer to write
	 * @param len Length of data
	 * @return Number of bytes written or negative error code
	 */
	int (*write)(const struct device *dev, const uint8_t *buf, size_t len);

	/**
	 * @brief Read data from USB serial port
	 * @param dev Device pointer
	 * @param buf Buffer to store read data
	 * @param max_len Maximum bytes to read
	 * @param timeout Timeout for read operation
	 * @return Number of bytes read or negative error code
	 */
	int (*read)(const struct device *dev, uint8_t *buf, size_t max_len,
		    k_timeout_t timeout);
};

/**
 * @brief USB Serial Port structure
 *
 * Thread-safe structure for managing USB serial port state
 */
struct usb_serial_port {
	/* usb device information */
	struct usb_device *udev;
	struct usbh_context *uhs_ctx;

	/* interface and endpoint configuration */
	uint8_t iface_num;
	uint8_t bulk_in_ep;
	uint8_t bulk_out_ep;
	uint16_t bulk_in_mps;
	uint16_t bulk_out_mps;

	/* device state */
	bool in_use;
	bool tx_in_progress;

	struct k_mutex port_mutex;
	struct k_sem sync_sem;

	/* RX data buffering */
	struct ring_buf ringbuf;
	uint8_t rx_buf[RX_RINGBUF_SIZE];
};

/**
 * @brief Write data to USB serial port (thread-safe)
 *
 * This function blocks until previous write completes or timeout occurs.
 * Maximum write size is limited to the endpoint's max packet size.
 *
 * @param dev Device pointer
 * @param buf Data buffer to write
 * @param len Length of data to write
 * @return Number of bytes written, or negative error code:
 *         -EINVAL: Invalid parameters
 *         -ENODEV: Device not ready
 *         -ETIMEDOUT: Write timeout (previous TX pending)
 */
static inline int usb_serial_write(const struct device *dev,
				   const uint8_t *buf, size_t len)
{
	const struct usb_serial_api *api = dev->api;
	return api->write(dev, buf, len);
}

/**
 * @brief Read data from USB serial port (thread-safe)
 *
 * This function reads data from the internal ring buffer. It blocks
 * if no data is available until timeout expires.
 *
 * @param dev Device pointer
 * @param buf Buffer to store read data
 * @param max_len Maximum bytes to read
 * @param timeout Timeout for read operation (K_NO_WAIT, K_FOREVER, or K_MSEC(x))
 * @return Number of bytes read (0 to max_len), or negative error code:
 *         -EINVAL: Invalid parameters
 *         -ENODEV: Device not ready
 *         -EAGAIN: Timeout (no data available)
 */
static inline int usb_serial_read(const struct device *dev, uint8_t *buf,
				  size_t max_len, k_timeout_t timeout)
{
	const struct usb_serial_api *api = dev->api;
	return api->read(dev, buf, max_len, timeout);
}

/**
 * @brief Get USB serial device instance
 *
 * @return Pointer to the USB serial device, or NULL if not available
 */
const struct device *usbh_serial_get_device(void);

#endif /* ZEPHYR_INCLUDE_USB_CLASS_USB_SERIAL_H_ */
