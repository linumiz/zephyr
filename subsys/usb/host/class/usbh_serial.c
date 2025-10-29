/*
 * Copyright (c) 2025 Linumiz GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT	zephyr_usbh_serial

#include <zephyr/usb/usbh.h>
#include <zephyr/usb/usb_ch9.h>
#include <zephyr/logging/log.h>
#include <zephyr/usb/class/usb_serial.h>
#include <zephyr/drivers/usb/uhc.h>

#include "usbh_class.h"
#include "usbh_device.h"
#include "usbh_desc.h"
#include "usbh_ch9.h"

#define USB_DEVICE_AND_IFACE_INFO(vend, prod, cl, sc, pr) 				\
	.vid = (vend),									\
	.pid = (prod),									\
	.class = (cl),									\
	.sub = (sc),									\
	.proto = (pr),									\
	.flags = USBH_CLASS_MATCH_CLASS | USBH_CLASS_MATCH_SUB | USBH_CLASS_MATCH_PROTO|\
	USBH_CLASS_MATCH_VID | USBH_CLASS_MATCH_PID,

#define QUECTEL_VENDOR_ID		0x2c7c
#define QUECTEL_PRODUCT_EG916Q		0x6007

LOG_MODULE_REGISTER(usbh_serial, CONFIG_USBH_SERIAL_LOG_LEVEL);

static int usbh_rx_cb(struct usb_device *udev, struct uhc_transfer *xfer);
static int usbh_tx_cb(struct usb_device *udev, struct uhc_transfer *xfer);

static struct usb_serial_port serial_port_data;

static int get_endpoints(struct usb_device *udev, uint8_t iface_num,
			 struct usb_serial_port *port)
{
	const void *cfg_start = usbh_desc_get_cfg_beg(udev);
	const void *cfg_end = usbh_desc_get_cfg_end(udev);
	const struct usb_desc_header *iface_desc;
	const struct usb_desc_header *desc;

	/* Find iface descriptor  */
	iface_desc = usbh_desc_get_by_iface(cfg_start, cfg_end, iface_num);
	if (iface_desc == NULL) {
		return -ENODEV;
	}

	/* Look for endpoints descriptors in this interface */
	desc = iface_desc;
	while ((desc = usbh_desc_get_next(desc, cfg_end)) != NULL) {
		/* stop at next interface*/
		if (desc->bDescriptorType == USB_DESC_INTERFACE) {
			break;
		}

		/* Process endpoint desc */
		if (desc->bDescriptorType == USB_DESC_ENDPOINT) {
			const struct usb_ep_descriptor *ep = (const struct usb_ep_descriptor *)desc;

			/* Only care about bulk endpoints */
			if ((ep->bmAttributes & USB_EP_TRANSFER_TYPE_MASK) == USB_EP_TYPE_BULK) {
				if (USB_EP_DIR_IS_IN(ep->bEndpointAddress)) {
					port->bulk_in_ep = ep->bEndpointAddress;
					port->bulk_in_mps = ep->wMaxPacketSize;
				} else {
					port->bulk_out_ep = ep->bEndpointAddress;
					port->bulk_out_mps = ep->wMaxPacketSize;
				}
			}
		}
	}

	LOG_INF("IN : 0x%02x, OUT : 0x%02x", port->bulk_in_ep, port->bulk_out_ep);
	LOG_INF("IN MPS : %d, OUT MPS : %d", port->bulk_in_mps, port->bulk_out_mps);
	if (!port->bulk_in_ep || !port->bulk_out_ep) {
		return -ENODEV;
	}

	return 0;
}

static int usbh_rx_queue(struct usb_serial_port *port)
{
	struct uhc_transfer *xfer;
	struct net_buf *buf;
	int ret;

	xfer = usbh_xfer_alloc(port->udev, port->bulk_in_ep,
			       usbh_rx_cb, port);
	if (!xfer) {
		LOG_ERR("Rx xfer alloc failed !!");
		return -ENOMEM;
	}

	buf = usbh_xfer_buf_alloc(port->udev, port->bulk_in_mps);
	if (!buf) {
		LOG_ERR("rx buf alloc failed !!");
		usbh_xfer_free(port->udev, xfer);
		return -ENOMEM;
	}

	ret = usbh_xfer_buf_add(port->udev, xfer, buf);
	if (ret) {
		LOG_ERR("rx buf add err :%d", ret);
		usbh_xfer_buf_free(port->udev, buf);
		usbh_xfer_free(port->udev, xfer);
		return ret;
	}

	ret = usbh_xfer_enqueue(port->udev, xfer);
	if (ret) {
		LOG_ERR("Rx enqueue failed : %d", ret);
		usbh_xfer_buf_free(port->udev, buf);
		usbh_xfer_free(port->udev, xfer);
		return ret;
	}

	return 0;
}

static int usbh_tx_queue(struct usb_serial_port *port,
			 const uint8_t *data, size_t len)
{
	struct uhc_transfer *xfer;
	struct net_buf *buf;
	int ret;

	xfer = usbh_xfer_alloc(port->udev, port->bulk_out_ep, usbh_tx_cb, port);
	if (!xfer) {
		LOG_ERR("Tx xfer alloc failed !!");
		return -ENOMEM;
	}

	buf = usbh_xfer_buf_alloc(port->udev, len);
	if (!buf) {
		LOG_ERR("tx buf alloc failed !!");
		usbh_xfer_free(port->udev, xfer);
		return -ENOMEM;
	}

	memcpy(buf->data, data, len);
	ret = usbh_xfer_buf_add(port->udev, xfer, buf);
	if (ret) {
		LOG_ERR("tx buf add err :%d", ret);
		return ret;
	}

	LOG_HEXDUMP_DBG(xfer->buf->data, xfer->buf->size, "TX data:");
	ret = usbh_xfer_enqueue(port->udev, xfer);
	if (ret) {
		LOG_ERR("Tx enqueue failed : %d", ret);
		usbh_xfer_buf_free(port->udev, buf);
		usbh_xfer_free(port->udev, xfer);
		return ret;
	}

	port->tx_in_progress = true;
	return ret;
}

static int usbh_rx_cb(struct usb_device *udev, struct uhc_transfer *xfer)
{
	struct usb_serial_port *port = (struct usb_serial_port *)xfer->priv;
	uint32_t written;

	LOG_INF("Rx callback err: %d, len: %d", xfer->err, xfer->buf->len);

	if (xfer->err) {
		LOG_ERR("RX cb error: %d", xfer->err);
		goto cleanup;
	}

	if (xfer->buf && xfer->buf->len > 0) {
		LOG_HEXDUMP_DBG(xfer->buf->data, xfer->buf->len, "RX data:");

		written = ring_buf_put(&port->ringbuf, xfer->buf->data, xfer->buf->len);

		if (written < xfer->buf->len) {
			LOG_WRN("Ring buffer full! Lost %d bytes", xfer->buf->len - written);
		}

		LOG_INF("Rx data (%d bytes received, %d buffered)",
			xfer->buf->len, written);
	}

cleanup:
	usbh_xfer_buf_free(port->udev, xfer->buf);
	usbh_xfer_free(port->udev, xfer);

	if (port->in_use && port->udev) {
		usbh_rx_queue(port);
	}
	return 0;
}

static int usbh_tx_cb(struct usb_device *udev, struct uhc_transfer *xfer)
{
	struct usb_serial_port *port = (struct usb_serial_port *)xfer->priv;

	LOG_DBG("Tx callback - ep: 0x%02x, err: %d", xfer->ep, xfer->err);

	if (xfer->err) {
		LOG_ERR("TX TRANSFER FAILED: %d", xfer->err);
	} else {
		LOG_HEXDUMP_DBG(xfer->buf->data, xfer->buf->len, "TX sent:");
		LOG_INF("TX completed: %d bytes", xfer->buf->len);
	}

	/* Clean up transfer resources */
	usbh_xfer_buf_free(port->udev, xfer->buf);
	usbh_xfer_free(port->udev, xfer);

	port->tx_in_progress = false;
	k_sem_give(&port->sync_sem);

	return 0;
}

static int usbh_serial_read(const struct device *dev, uint8_t *buf,
			    size_t max_len, k_timeout_t timeout)
{
	struct usb_serial_port *port = (struct usb_serial_port *)dev->data;
	uint32_t bytes_read;

	if (!buf || max_len == 0) {
		return -EINVAL;
	}

	bytes_read = ring_buf_get(&port->ringbuf, buf, max_len);

	return bytes_read;
}

static int usbh_serial_write(const struct device *dev, const uint8_t *buf, size_t len)
{
	struct usb_serial_port *port = (struct usb_serial_port *)dev->data;
	int ret;
	LOG_INF("write class !");

	if (!buf || len == 0 || len > port->bulk_out_mps) {
		return -EINVAL;
	}

	if (!port->in_use || !port->udev) {
		return -ENODEV;
	}

	if (k_mutex_lock(&port->port_mutex, K_MSEC(1000)) != 0) {
		return -EBUSY;
	}

	ret = usbh_tx_queue(port, buf, len);
	if (ret) {
		LOG_ERR("Tx queue err : %d", ret);
	}

	if (k_sem_take(&port->sync_sem, K_MSEC(5000)) != 0) {
		LOG_ERR("Timeout");
		return -ETIMEDOUT;
	}

	k_mutex_unlock(&port->port_mutex);
	LOG_INF("write class !");
	return len;
}

static int usbh_serial_probe(struct usbh_class_data *const c_data,
			     struct usb_device *const udev,
			     const uint8_t iface)
{
	struct usb_serial_port *port = c_data->priv;
	uint8_t interface = 1;
	int ret;

	struct device *dev = udev->ctx;
	if (port->in_use) {
		k_mutex_unlock(&port->port_mutex);
		LOG_INF("Port Busy, rejecting interface %u", iface);
		return -ENOTSUP;
	}

	ret = get_endpoints(udev, interface, port);
	if (ret != 0) {
		k_mutex_unlock(&port->port_mutex);
		LOG_INF("No bulk endpoints found");
		return ret;
	}
#if 1
	ret = usbh_req_setup(udev, 0x21, 0x22, 0x0003, 0x0002, 0, NULL);
	if (ret == 0) {
		LOG_INF("set control line success");
	} else {
		LOG_INF("set control failed");
	}
#endif

	ret = usbh_req_setup(udev, 0x01, 0x0B, 0x0000, 0x0002, 0, NULL);
	if (ret == 0) {
		LOG_INF("set iface success");
	} else {
		LOG_INF("set iface failed !");
	}

	ret = usbh_req_setup(udev, 0x21, 0x22, 0x0003, 0x0002, 0, NULL);
	if (ret == 0) {
		LOG_INF("set control line success");
	} else {
		LOG_INF("set control failed");
	}

	/* Fill in port information */
	port->udev = udev;
	port->iface_num = iface;
	port->in_use = true;
	port->tx_in_progress = false;

	/* Start RX transfers */
	ret = usbh_rx_queue(port);
	if (ret) {
		LOG_ERR("serial rx init : %d", ret);
		port->in_use = false;
		port->udev = NULL;
		return ret;
	}

	LOG_INF("USB Serial device probed successfully");
	return 0;
}

static int usbh_serial_removed(struct usbh_class_data *c_data)
{
	struct usb_serial_port *port = c_data->priv;

	if (port->in_use) {
		LOG_INF("Removing USB serial device");
		port->in_use = false;
		port->udev = NULL;
		port->tx_in_progress = false;
	}

	return 0;
}

static int usbh_serial_init(struct usbh_class_data *const c_data,
			    struct usbh_context *const uhs_ctx)
{
	struct usb_serial_port *port = (struct usb_serial_port *)c_data->priv;

	LOG_INF("Class init");
	port->uhs_ctx = uhs_ctx;

	k_mutex_init(&port->port_mutex);
	k_sem_init(&port->sync_sem, 0, 1);

	ring_buf_init(&port->ringbuf, RX_RINGBUF_SIZE, port->rx_buf);

	port->tx_in_progress = false;
	return 0;
}

static int usbh_serial_completion_cb(struct usbh_class_data *c_data,
				     struct uhc_transfer *xfer)
{
	return 0;
}

static int usbh_serial_suspended(struct usbh_class_data *const c_data)
{
	return 0;
}

static int usbh_serial_resumed(struct usbh_class_data *const c_data)
{

	return 0;
}

const struct device *usbh_serial_get_device(void)
{
	return device_get_binding("usb_serial0");
}

static struct usbh_class_api usbh_serial_class_api = {
	.init = usbh_serial_init,
	.probe = usbh_serial_probe,
	.removed = usbh_serial_removed,
	.completion_cb = usbh_serial_completion_cb,
	.suspended = usbh_serial_suspended,
	.resumed = usbh_serial_resumed,
};

static struct usb_serial_api usbh_serial_device_api = {
	.write = usbh_serial_write,
	.read = usbh_serial_read,
};

static struct usbh_class_filter usb_serial_filters[] = {
	{ USB_DEVICE_AND_IFACE_INFO(QUECTEL_VENDOR_ID, QUECTEL_PRODUCT_EG916Q,
				    USB_BCC_VENDOR, 0, 0) },
	{ USB_DEVICE_AND_IFACE_INFO(0x2fe3, 0x0001,USB_BCC_CDC_DATA, 0, 0)},
};

#define USBH_SERIAL_DEFINE(n)								\
											\
	DEVICE_DEFINE(usb_serial0, "usb_serial0", NULL, NULL, &serial_port_data,	\
		      NULL, POST_KERNEL, 50, &usbh_serial_device_api);			\
		      									\
	USBH_DEFINE_CLASS(usbh_serial, &usbh_serial_class_api, &serial_port_data,	\
			  usb_serial_filters, ARRAY_SIZE(usb_serial_filters));		\

DT_INST_FOREACH_STATUS_OKAY(USBH_SERIAL_DEFINE)
