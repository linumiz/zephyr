/*
 * Copyright 2025 Linumiz GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ti_mspm0_aes

#include <zephyr/crypto/cipher.h>
#include <zephyr/crypto/crypto.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <ti/driverlib/dl_aes.h>

#define AES_HW_CAPS (CAP_RAW_KEY | CAP_SEPARATE_IO_BUFS | CAP_SYNC_OPS | CAP_NO_IV_PREFIX)

#define AES_BLOCK_SIZE		16
#define AES_MSPM0		DT_NODELABEL(aes)

/*
 * The block cycle for aes module max is 300 cycles
 * safety margin for this module timeout 300 << 1 cycles
 * K_CYC calculates the time periord for respective clock freq
 * AES_WAIT_TIMEOUT is calculated for worst case time
 */
#define MSPM0_AES_BLOCK_CYC	300
#define MSPM0_AES_BLOCK_TIMEOUT	(MSPM0_AES_BLOCK_CYC << 1)
#define AES_TIMEOUT		K_CYC(MSPM0_AES_BLOCK_TIMEOUT)
#define AES_WAIT_TIMEOUT	K_USEC(10)

#define LOG_LEVEL CONFIG_CRYPTO_LOG_LEVEL
LOG_MODULE_REGISTER(aes);

struct crypto_mspm0_aes_config {
	AES_Regs *regs;
};

struct crypto_mspm0_aes_data {
	DL_AES_KEY_LENGTH keylen;
	enum cipher_op op;
	struct k_mutex device_mutex;
	struct k_sem aes_done;
};

static int validate_pkt(struct cipher_pkt *pkt)
{
	if (!pkt || !pkt->in_buf || !pkt->out_buf) {
		LOG_ERR("Invalid packet or NULL buffers");
		return -EINVAL;
	}

	if (pkt->in_len == 0 || (pkt->in_len % AES_BLOCK_SIZE) != 0) {
		LOG_ERR("Invalid input length");
		return -EINVAL;
	}

	if (pkt->out_buf_max < AES_BLOCK_SIZE) {
		LOG_ERR("Output buffer too small");
		return -ENOMEM;
	}

	return 0;
}

static int crypto_aes_ecb_op(struct cipher_ctx *ctx, struct cipher_pkt *pkt)
{
	const struct device *dev = ctx->device;
	const struct crypto_mspm0_aes_config *config = dev->config;
	struct crypto_mspm0_aes_data *data = dev->data;
	int bytes_processed = 0;
	int ret;

	ret = validate_pkt(pkt);
	if (ret) {
		return ret;
	}

	k_mutex_lock(&data->device_mutex, AES_WAIT_TIMEOUT);
	/* load key */
	ret = DL_AES_setKey(config->regs, ctx->key.bit_stream, data->keylen);
	if (ret != DL_AES_STATUS_SUCCESS) {
		goto cleanup;
	}

	DL_AES_setAllKeyWritten(config->regs);

	do {
		/* load the block */
		ret = DL_AES_loadDataIn(config->regs, &pkt->in_buf[bytes_processed]);
		if (ret != DL_AES_STATUS_SUCCESS) {
			goto cleanup;
		}

		/* wait for AES operation completion */
		ret = k_sem_take(&data->aes_done, AES_TIMEOUT);
		if (ret) {
			goto cleanup;
		}

		/* read the dataout */
		ret = DL_AES_getDataOut(config->regs, &pkt->out_buf[bytes_processed]);
		if (ret != DL_AES_STATUS_SUCCESS) {
			goto cleanup;
		}

		bytes_processed += AES_BLOCK_SIZE;

	} while (bytes_processed < pkt->in_len);

cleanup:
	pkt->out_len = bytes_processed;
	k_mutex_unlock(&data->device_mutex);

	return ret;
}

static int crypto_aes_cbc_op(struct cipher_ctx *ctx, struct cipher_pkt *pkt, uint8_t *iv)
{
	const struct device *dev = ctx->device;
	const struct crypto_mspm0_aes_config *config = dev->config;
	struct crypto_mspm0_aes_data *data = dev->data;
	int in_bytes_processed = 0, out_bytes_processed = 0, data_len;
	int ret;
	bool pregen_key = false;

	ret = validate_pkt(pkt);
	if (ret) {
		return ret;
	}

	switch (data->op) {
	case CRYPTO_CIPHER_OP_DECRYPT:
		pregen_key = true;
		iv = &pkt->in_buf[in_bytes_processed];
		in_bytes_processed += AES_BLOCK_SIZE;
		data_len = pkt->in_len - AES_BLOCK_SIZE;
		break;
	case CRYPTO_CIPHER_OP_ENCRYPT:
		memcpy(pkt->out_buf, iv, AES_BLOCK_SIZE);
		out_bytes_processed += AES_BLOCK_SIZE;
		data_len = pkt->in_len + AES_BLOCK_SIZE;
		pkt->out_len = out_bytes_processed;
		break;
	default:
		return -EINVAL;
	};

	k_mutex_lock(&data->device_mutex, AES_WAIT_TIMEOUT);

	/* load key */
	ret = DL_AES_setKey(config->regs, ctx->key.bit_stream, data->keylen);
	if (ret != DL_AES_STATUS_SUCCESS) {
		goto cleanup;
	}

	DL_AES_setAllKeyWritten(config->regs);

	/* change the mode from pre-gen to use pre-gen key mode */
	if (pregen_key) {
		DL_AES_MODE aesmode;

		ret = k_sem_take(&data->aes_done, AES_TIMEOUT);
		if (ret) {
			goto cleanup;
		}

		aesmode = DL_AES_MODE_DECRYPT_KEY_IS_FIRST_ROUND_KEY_CBC_MODE;

		DL_AES_init(config->regs, aesmode, data->keylen);
		DL_AES_setAllKeyWritten(config->regs);
	}

	/* load iv */
	ret = DL_AES_loadXORDataInWithoutTrigger(config->regs, iv);
	if (ret != DL_AES_STATUS_SUCCESS) {
		goto cleanup;
	}

	do {
		/* load the next block if any */
		if (in_bytes_processed < pkt->in_len) {
			if (data->op == CRYPTO_CIPHER_OP_DECRYPT) {
				ret = DL_AES_loadDataIn(config->regs,
							&pkt->in_buf[in_bytes_processed]);
				if (ret != DL_AES_STATUS_SUCCESS) {
					goto cleanup;
				}
				in_bytes_processed += AES_BLOCK_SIZE;
			} else {
				ret = DL_AES_loadXORDataIn(config->regs,
							   &pkt->in_buf[in_bytes_processed]);
				if (ret != DL_AES_STATUS_SUCCESS) {
					goto cleanup;
				}
				in_bytes_processed += AES_BLOCK_SIZE;
			}
		}

		/* wait for AES operation completion */
		ret = k_sem_take(&data->aes_done, AES_TIMEOUT);
		if (ret) {
			goto cleanup;
		}

		/* xor the iv with internal state */
		if (data->op == CRYPTO_CIPHER_OP_DECRYPT) {
			ret = DL_AES_loadXORDataInWithoutTrigger(config->regs, iv);
			if (ret != DL_AES_STATUS_SUCCESS) {
				goto cleanup;
			}
			iv = &pkt->in_buf[in_bytes_processed - AES_BLOCK_SIZE];
		}
		/* read the dataout */
		ret = DL_AES_getDataOut(config->regs, &pkt->out_buf[out_bytes_processed]);
		if (ret != DL_AES_STATUS_SUCCESS) {
			goto cleanup;
		}
		out_bytes_processed += AES_BLOCK_SIZE;

	} while (out_bytes_processed < data_len);

cleanup:
	pkt->out_len = out_bytes_processed;
	k_mutex_unlock(&data->device_mutex);

	return ret;
}

static int crypto_aes_cfb_op(struct cipher_ctx *ctx, struct cipher_pkt *pkt, uint8_t *iv)
{
	const struct device *dev = ctx->device;
	const struct crypto_mspm0_aes_config *config = dev->config;
	struct crypto_mspm0_aes_data *data = dev->data;
	int bytes_processed = 0;
	int ret;

	ret = validate_pkt(pkt);
	if (ret) {
		return ret;
	}

	k_mutex_lock(&data->device_mutex, AES_WAIT_TIMEOUT);

	/* load key */
	ret = DL_AES_setKey(config->regs, ctx->key.bit_stream, data->keylen);
	if (ret != DL_AES_STATUS_SUCCESS) {
		goto cleanup;
	}
	DL_AES_setAllKeyWritten(config->regs);

	do {
		/* load the next block if any */
		if (bytes_processed < pkt->in_len) {
			ret = DL_AES_loadDataIn(config->regs, iv);
			if (ret != DL_AES_STATUS_SUCCESS) {
				goto cleanup;
			}
		}

		/* wait for AES operation completion */
		ret = k_sem_take(&data->aes_done, AES_TIMEOUT);
		if (ret) {
			goto cleanup;
		}

		/* xor the intput block with internal state */
		if (data->op == CRYPTO_CIPHER_OP_DECRYPT) {
			ret = DL_AES_loadXORDataInWithoutTrigger(config->regs,
								 &pkt->in_buf[bytes_processed]);
			if (ret != DL_AES_STATUS_SUCCESS) {
				goto cleanup;
			}
			iv = &pkt->in_buf[bytes_processed];
		} else {
			ret = DL_AES_loadXORDataInWithoutTrigger(config->regs,
								 &pkt->in_buf[bytes_processed]);
			if (ret != DL_AES_STATUS_SUCCESS) {
				goto cleanup;
			}
			iv = &pkt->out_buf[bytes_processed];
		}
		/* read the dataout */
		ret = DL_AES_getDataOut(config->regs, &pkt->out_buf[bytes_processed]);
		if (ret != DL_AES_STATUS_SUCCESS) {
			goto cleanup;
		}
		bytes_processed += AES_BLOCK_SIZE;

	} while (bytes_processed < pkt->in_len);

cleanup:
	pkt->out_len = bytes_processed;
	k_mutex_unlock(&data->device_mutex);

	return ret;
}

static int crypto_aes_ofb_op(struct cipher_ctx *ctx, struct cipher_pkt *pkt, uint8_t *iv)
{
	const struct device *dev = ctx->device;
	const struct crypto_mspm0_aes_config *config = dev->config;
	struct crypto_mspm0_aes_data *data = dev->data;
	int bytes_processed = 0;
	uint8_t block[AES_BLOCK_SIZE] __aligned(4);
	int ret;

	ret = validate_pkt(pkt);
	if (ret) {
		return ret;
	}

	k_mutex_lock(&data->device_mutex, AES_WAIT_TIMEOUT);

	/* load key */
	ret = DL_AES_setKey(config->regs, ctx->key.bit_stream, data->keylen);
	if (ret != DL_AES_STATUS_SUCCESS) {
		goto cleanup;
	}
	DL_AES_setAllKeyWritten(config->regs);

	do {
		/* load the next block if any */
		if (bytes_processed < pkt->in_len) {
			ret = DL_AES_loadDataIn(config->regs, iv);
			if (ret != DL_AES_STATUS_SUCCESS) {
				goto cleanup;
			}
		}

		/* wait for AES operation completion */
		ret = k_sem_take(&data->aes_done, AES_TIMEOUT);
		if (ret) {
			goto cleanup;
		}

		/* read the dataout for next feedback */
		ret = DL_AES_getDataOut(config->regs, (uint8_t *)&block);
		if (ret != DL_AES_STATUS_SUCCESS) {
			goto cleanup;
		}
		iv = (uint8_t *)&block;

		/* xor the input text with internal state */
		ret = DL_AES_loadXORDataInWithoutTrigger(config->regs,
							 &pkt->in_buf[bytes_processed]);
		if (ret != DL_AES_STATUS_SUCCESS) {
			goto cleanup;
		}

		/* read the dataout */
		ret = DL_AES_getDataOut(config->regs, &pkt->out_buf[bytes_processed]);
		if (ret != DL_AES_STATUS_SUCCESS) {
			goto cleanup;
		}
		bytes_processed += AES_BLOCK_SIZE;

	} while (bytes_processed < pkt->in_len);

cleanup:
	pkt->out_len = bytes_processed;
	k_mutex_unlock(&data->device_mutex);

	return ret;
}

static void crypto_mspm0_aes_isr(const struct device *dev)
{
	const struct crypto_mspm0_aes_config *config = dev->config;
	struct crypto_mspm0_aes_data *data = dev->data;

	if (!DL_AES_getPendingInterrupt(config->regs)) {
		LOG_ERR("No pending Interrupts");
		return;
	}

	k_sem_give(&data->aes_done);
}

static int aes_session_setup(const struct device *dev, struct cipher_ctx *ctx,
			     enum cipher_algo algo, enum cipher_mode mode, enum cipher_op op)
{
	const struct crypto_mspm0_aes_config *config = dev->config;
	struct crypto_mspm0_aes_data *data = dev->data;
	DL_AES_MODE aesconfig;

	if (algo != CRYPTO_CIPHER_ALGO_AES) {
		LOG_ERR("Unsupported algo");
		return -EINVAL;
	}

	if (!ctx) {
		LOG_ERR("Missing context");
		return -EINVAL;
	}

	if (!ctx->key.bit_stream) {
		LOG_ERR("No key provided");
		return -EINVAL;
	}

	switch (ctx->keylen) {
	case 16U:
		data->keylen = DL_AES_KEY_LENGTH_128;
		break;
	case 32U:
		data->keylen = DL_AES_KEY_LENGTH_256;
		break;
	default:
		LOG_ERR("key size is not supported");
		return -EINVAL;
	}

	DL_AES_softwareReset(config->regs);
	switch (mode) {
	case CRYPTO_CIPHER_MODE_ECB:
		if (op == CRYPTO_CIPHER_OP_ENCRYPT) {
			aesconfig = DL_AES_MODE_ENCRYPT_ECB_MODE;
		} else {
			aesconfig = DL_AES_MODE_DECRYPT_SAME_KEY_ECB_MODE;
		}
		ctx->ops.block_crypt_hndlr = crypto_aes_ecb_op;
		break;

	case CRYPTO_CIPHER_MODE_CBC:
		if (op == CRYPTO_CIPHER_OP_ENCRYPT) {
			aesconfig = DL_AES_MODE_ENCRYPT_CBC_MODE;
		} else {
			aesconfig = DL_AES_MODE_GEN_FIRST_ROUND_KEY_CBC_MODE;
		}
		/* enable cipher block mode */
		DL_AES_enableCipherMode(config->regs);
		ctx->ops.cbc_crypt_hndlr = crypto_aes_cbc_op;
		break;

	case CRYPTO_CIPHER_MODE_CFB:
		if (op == CRYPTO_CIPHER_OP_ENCRYPT) {
			aesconfig = DL_AES_MODE_ENCRYPT_CFB_MODE;
		} else {
			aesconfig = DL_AES_MODE_DECRYPT_SAME_KEY_CFB_MODE;
		}
		/* enable cipher block mode */
		DL_AES_enableCipherMode(config->regs);
		ctx->ops.cfb_crypt_hndlr = crypto_aes_cfb_op;
		break;

	case CRYPTO_CIPHER_MODE_OFB:
		if (op == CRYPTO_CIPHER_OP_ENCRYPT) {
			aesconfig = DL_AES_MODE_ENCRYPT_OFB_MODE;
		} else {
			aesconfig = DL_AES_MODE_DECRYPT_SAME_KEY_OFB_MODE;
		}
		/* enable cipher block mode */
		DL_AES_enableCipherMode(config->regs);
		ctx->ops.ofb_crypt_hndlr = crypto_aes_ofb_op;
		break;

	default:
		return -EINVAL;
	}

	DL_AES_init(config->regs, aesconfig, data->keylen);

	ctx->ops.cipher_mode = mode;
	ctx->device = dev;
	data->op = op;

	return 0;
}

static int aes_session_free(const struct device *dev, struct cipher_ctx *ctx)
{
	struct crypto_mspm0_aes_data *data = dev->data;

	ctx->device = NULL;
	data->keylen = 0;
	return 0;
}

static int aes_query_caps(const struct device *dev)
{
	ARG_UNUSED(dev);
	return AES_HW_CAPS;
}

int crypto_aes_init(const struct device *dev)
{
	const struct crypto_mspm0_aes_config *config = dev->config;
	struct crypto_mspm0_aes_data *data = dev->data;

	DL_AES_enablePower(config->regs);

	if (!DL_AES_isPowerEnabled(config->regs)) {
		LOG_ERR("AES Power is not enabled !!");
		return -EIO;
	}

	k_mutex_init(&data->device_mutex);

	k_sem_init(&data->aes_done, 0, 1);

	/* disable interrupt */
	DL_AES_disableInterrupt(config->regs);

	/* clear interrupt status regs */
	DL_AES_clearInterruptStatus(config->regs);

	IRQ_CONNECT(DT_IRQN(AES_MSPM0), DT_IRQ(AES_MSPM0, priority), crypto_mspm0_aes_isr,
		    DEVICE_DT_GET(AES_MSPM0), 0);
	irq_enable(DT_IRQN(AES_MSPM0));

	/* enable interrupt */
	DL_AES_enableInterrupt(config->regs);

	if (!DL_AES_getEnabledInterrupts(config->regs)) {
		LOG_ERR("AES READY INT is not enabled !!");
		return -EIO;
	}

	return 0;
}

static DEVICE_API(crypto, crypto_enc_funcs) = {
	.cipher_begin_session = aes_session_setup,
	.cipher_free_session = aes_session_free,
	.query_hw_caps = aes_query_caps,
};

#define MSPM0_AES_DEFINE(n)									\
												\
static const struct crypto_mspm0_aes_config crypto_aes_config_##n = {				\
	.regs = (AES_Regs *)DT_INST_REG_ADDR(0),						\
};												\
												\
struct crypto_mspm0_aes_data crypto_aes_data_##n;						\
												\
DEVICE_DT_INST_DEFINE(0, crypto_aes_init, NULL, &crypto_aes_data_##n, &crypto_aes_config_##n,	\
		      POST_KERNEL, CONFIG_CRYPTO_INIT_PRIORITY, (void *)&crypto_enc_funcs);

DT_INST_FOREACH_STATUS_OKAY(MSPM0_AES_DEFINE)
