/*
 * Copyright (c) 2025 Linumiz GmbH.
 *
 * Infineon Mailbox driver for Zephyr's MBOX model.
 */

#include <zephyr/devicetree.h>
#include <zephyr/drivers/mbox.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>

#include "cy_ipc_drv.h"

LOG_MODULE_REGISTER(mbox_ifx_cat1);

#define DT_DRV_COMPAT infineon_cat1_mbox_pdl

#define MAILBOX_MBOX_SIZE 4
#define MAILBOX_MAX_CHANNELS 8 /* Channels 0 - 7 */
#define CHANNEL_SELECT(x) (1U << x)

struct ifx_cat1_mailbox_data {
	mbox_callback_t cb[MAILBOX_MAX_CHANNELS];
	void *user_data[MAILBOX_MAX_CHANNELS];
	bool channel_enable[MAILBOX_MAX_CHANNELS];
	uint32_t received_data;
};

struct ifx_cat1_mailbox_config {
	uint8_t irq_num;
	uint8_t irq_priority;
};

static void ifx_cat1_mailbox_isr(const struct device *dev)
{
	struct ifx_cat1_mailbox_data *data = dev->data;

      	for (int i_channel = 0; i_channel < MAILBOX_MAX_CHANNELS; i_channel++) {
		/* Continue to next channel if channel is not enabled or callback not registered */
		if (!data->channel_enable[i_channel] || data->cb[i_channel] == NULL) {
			continue;
		}

		Cy_IPC_Drv_ClearInterrupt(Cy_IPC_Drv_GetIntrBaseAddr(i_channel), CY_IPC_NO_NOTIFICATION,
					  CHANNEL_SELECT(i_channel));

		if (Cy_IPC_Drv_ReadMsgWord(Cy_IPC_Drv_GetIpcBaseAddress(i_channel),
		    &data->received_data) != CY_IPC_DRV_SUCCESS) {
			continue;
		}

		struct mbox_msg msg = {(const void *)&data->received_data,
					       MAILBOX_MBOX_SIZE};

		if (data->cb[i_channel]) {
			data->cb[i_channel](dev, i_channel, data->user_data[i_channel],
					    &msg);
		}

		Cy_IPC_Drv_LockRelease(Cy_IPC_Drv_GetIpcBaseAddress(i_channel), CY_IPC_NO_NOTIFICATION);
	}
}

static int ifx_cat1_mailbox_send(const struct device *dev, uint32_t channel, const struct mbox_msg *msg)
{
	IPC_STRUCT_Type* ipc_channel = Cy_IPC_Drv_GetIpcBaseAddress(channel);
	cy_en_ipcdrv_status_t ipc_status = CY_IPC_DRV_SUCCESS;

	if (channel >= MAILBOX_MAX_CHANNELS) {
		return -EINVAL;
	}

	/* Signalling mode */
	if (msg == NULL) {
		LOG_ERR("Signalling mode not supported");
		return -EINVAL;
	}

	if (msg->size != MAILBOX_MBOX_SIZE) {
		/* We can only send this many bytes at a time. */
		return -EMSGSIZE;
	}

	ipc_status = Cy_IPC_Drv_SendMsgWord(ipc_channel, CHANNEL_SELECT(channel), (*((uint32_t *)msg->data)));
	if (ipc_status != CY_IPC_DRV_SUCCESS) {
		LOG_ERR("Failed at sending msg word (Result 0x%x)", ipc_status);
		return -EINVAL;
	}

	return 0;
}

static int ifx_cat1_mailbox_register_callback(const struct device *dev, uint32_t channel,
					      mbox_callback_t cb, void *user_data)
{
	struct ifx_cat1_mailbox_data *data = dev->data;

	if (channel >= MAILBOX_MAX_CHANNELS) {
		return -EINVAL;
	}

	data->cb[channel] = cb;
	data->user_data[channel] = user_data;

	return 0;
}

static int ifx_cat1_mailbox_mtu_get(const struct device *dev)
{
	ARG_UNUSED(dev);
	return MAILBOX_MBOX_SIZE;
}

static uint32_t ifx_cat1_mailbox_max_channels_get(const struct device *dev)
{
	ARG_UNUSED(dev);
	return MAILBOX_MAX_CHANNELS;
}

static int ifx_cat1_mailbox_set_enabled(const struct device *dev, uint32_t channel, bool enable)
{
	const struct ifx_cat1_mailbox_config *cfg = dev->config;
	struct ifx_cat1_mailbox_data *data = dev->data;
	uint32_t notify_mask;
	uint32_t intr_mask;
	IPC_INTR_STRUCT_Type *irq_struct;

	if (channel >= MAILBOX_MAX_CHANNELS) {
		return -EINVAL;
	}

	irq_struct =  Cy_IPC_Drv_GetIntrBaseAddr(channel);
	intr_mask = Cy_IPC_Drv_GetInterruptMask(irq_struct);

	if (enable) {
		data->channel_enable[channel] = enable;
		intr_mask |= (1U << (16U + channel));   /* enable NOTIFY for this channel */
		enable_sys_int(cfg->irq_num + channel, cfg->irq_priority,
			       (void (*)(const void *))(void *)ifx_cat1_mailbox_isr, dev);
	} else {
		intr_mask &= ~(1U << (16U + channel));  /* disable NOTIFY for this channel */
	}

	notify_mask = Cy_IPC_Drv_ExtractAcquireMask(intr_mask);
	Cy_IPC_Drv_SetInterruptMask(irq_struct, CY_IPC_NO_NOTIFICATION, notify_mask);

	return 0;
}

static DEVICE_API(mbox, ifx_cat1_mailbox_driver_api) = {
	.send = ifx_cat1_mailbox_send,
	.register_callback = ifx_cat1_mailbox_register_callback,
	.mtu_get = ifx_cat1_mailbox_mtu_get,
	.max_channels_get = ifx_cat1_mailbox_max_channels_get,
	.set_enabled = ifx_cat1_mailbox_set_enabled,
};

#define INFINEON_MAILBOX_INSTANCE_DEFINE(idx)							\
												\
	struct ifx_cat1_mailbox_data ifx_cat1_mailbox_##idx##_data;				\
												\
	struct ifx_cat1_mailbox_config ifx_cat1_mailbox_##idx##_config = {			\
		.irq_num = DT_INST_PROP_BY_IDX(idx, system_interrupts, 0),                      \
		.irq_priority = DT_INST_PROP_BY_IDX(idx, system_interrupts, 1),			\
	};                                                                                      \
												\
	DEVICE_DT_INST_DEFINE(idx, NULL, NULL, &ifx_cat1_mailbox_##idx##_data,			\
			      &ifx_cat1_mailbox_##idx##_config, POST_KERNEL,			\
			      CONFIG_MBOX_INIT_PRIORITY, &ifx_cat1_mailbox_driver_api)

DT_INST_FOREACH_STATUS_OKAY(INFINEON_MAILBOX_INSTANCE_DEFINE)
