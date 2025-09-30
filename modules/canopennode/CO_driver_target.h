/*
 * Copyright (c) 2019 Vestas Wind Systems A/S
 * Copyright (c) 2025 National Taiwan University Racing Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_MODULES_CANOPENNODE_CO_DRIVER_H
#define ZEPHYR_MODULES_CANOPENNODE_CO_DRIVER_H

/*
 * Zephyr RTOS CAN driver interface and configuration for CANopenNode
 * CANopen protocol stack.
 *
 * See canopennode/301/CO_driver.h for API description.
 */

// glibc includes
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// zephyr includes
#include <zephyr/device.h>
#include <zephyr/dsp/types.h> /* float32_t, float64_t */
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/toolchain.h>
#include <zephyr/types.h>

// canopennode includes
#include <301/CO_config.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Turn on feature for receiving callbacks for some objects */
#define CO_CONFIG_GLOBAL_FLAG_CALLBACK_PRE    (CO_CONFIG_FLAG_CALLBACK_PRE)
#define CO_CONFIG_GLOBAL_RT_FLAG_CALLBACK_PRE (CO_CONFIG_FLAG_CALLBACK_PRE)

/* Turn on feature for calculating next wakeup time */
#define CO_CONFIG_GLOBAL_FLAG_TIMERNEXT (CO_CONFIG_FLAG_TIMERNEXT)

/* Turn on feature for dynamic object dictionary */
#define CO_CONFIG_GLOBAL_FLAG_OD_DYNAMIC (CO_CONFIG_FLAG_OD_DYNAMIC)

/* Network management (NMT) */
#define CO_CONFIG_NMT                                                                              \
	(CO_CONFIG_NMT_CALLBACK_CHANGE | CO_CONFIG_NMT_MASTER |                                    \
	 CO_CONFIG_GLOBAL_FLAG_CALLBACK_PRE | CO_CONFIG_GLOBAL_FLAG_TIMERNEXT)

/* Heartbeat */
#ifdef CONFIG_CANOPENNODE_HB_CONS_DISABLED
#define CO_CONFIG_HB_CONS (0)
#else
#ifdef CONFIG_CANOPENNODE_HB_CONS_SINGLE_CALLBACK
#define CO_CONFIG_HB_CONS_CALLBACK (CO_CONFIG_HB_CONS_CALLBACK_CHANGE)
#else
#define CO_CONFIG_HB_CONS_CALLBACK (CO_CONFIG_HB_CONS_CALLBACK_MULTI)
#endif
#define CO_CONFIG_HB_CONS                                                                          \
	(CO_CONFIG_HB_CONS_ENABLE | CO_CONFIG_HB_CONS_CALLBACK | CO_CONFIG_HB_CONS_QUERY_FUNCT |   \
	 CO_CONFIG_GLOBAL_FLAG_CALLBACK_PRE | CO_CONFIG_GLOBAL_FLAG_TIMERNEXT |                    \
	 CO_CONFIG_GLOBAL_FLAG_OD_DYNAMIC)
#endif

/* Emergency (EM) */
#define CO_CONFIG_EM                                                                               \
	(CO_CONFIG_EM_PRODUCER | CO_CONFIG_EM_PROD_CONFIGURABLE | CO_CONFIG_EM_PROD_INHIBIT |      \
	 CO_CONFIG_EM_HISTORY | CO_CONFIG_EM_CONSUMER | CO_CONFIG_GLOBAL_FLAG_CALLBACK_PRE |       \
	 CO_CONFIG_GLOBAL_FLAG_TIMERNEXT)

#define CO_CONFIG_EM_ERR_STATUS_BITS_COUNT (CONFIG_CANOPENNODE_EM_ERR_STATUS_BITS_COUNT)

/* Service data object (SDO) */
#define CO_CONFIG_SDO_SRV                                                                          \
	(CO_CONFIG_SDO_SRV_SEGMENTED |                                                             \
	 COND_CODE_1(CONFIG_CANOPENNODE_SDO_BLOCK, (CO_CONFIG_SDO_SRV_BLOCK ), (0)) | CO_CONFIG_GLOBAL_FLAG_CALLBACK_PRE |         \
		      CO_CONFIG_GLOBAL_FLAG_TIMERNEXT | CO_CONFIG_GLOBAL_FLAG_OD_DYNAMIC)

#ifdef CONFIG_CANOPENNODE_SDO_CLI
#define CO_CONFIG_SDO_CLI                                                                          \
	(CO_CONFIG_SDO_CLI_ENABLE | CO_CONFIG_SDO_CLI_SEGMENTED |                                  \
	 COND_CODE_1(CONFIG_CANOPENNODE_SDO_BLOCK, (CO_CONFIG_SDO_SRV_BLOCK ), (0)) | CO_CONFIG_SDO_CLI_LOCAL |                    \
		      CO_CONFIG_GLOBAL_FLAG_CALLBACK_PRE | CO_CONFIG_GLOBAL_FLAG_TIMERNEXT |       \
		      CO_CONFIG_GLOBAL_FLAG_OD_DYNAMIC)

/* SDO requires FIFO */
#define CO_CONFIG_FIFO                                                                             \
	(CO_CONFIG_FIFO_ENABLE |                                                                   \
	 COND_CODE_1(CONFIG_CANOPENNODE_SDO_BLOCK, (CO_CONFIG_FIFO_ALT_READ | CO_CONFIG_FIFO_CRC16_CCITT), (0)))
#else
#define CO_CONFIG_SDO_CLI (0)
#endif

#define CO_CONFIG_SDO_SRV_BUFFER_SIZE (CONFIG_CANOPENNODE_SDO_BUFFER_SIZE)
#define CO_CONFIG_SDO_CLI_BUFFER_SIZE (CONFIG_CANOPENNODE_SDO_BUFFER_SIZE)

/* TIME */
#ifdef CONFIG_CANOPENNODE_TIME
#define CO_CONFIG_TIME                                                                             \
	(CO_CONFIG_TIME_ENABLE | CO_CONFIG_GLOBAL_FLAG_CALLBACK_PRE |                              \
	 CO_CONFIG_GLOBAL_FLAG_TIMERNEXT | CO_CONFIG_GLOBAL_FLAG_OD_DYNAMIC)
#else
#define CO_CONFIG_TIME (0)
#endif

/* SYNC */
#ifndef CONFIG_CANOPENNODE_SYNC_DISABLED
#define CO_CONFIG_SYNC                                                                             \
	(CO_CONFIG_SYNC_ENABLE |                                                                   \
	 COND_CODE_1(CONFIG_CANOPENNODE_SYNC_PRODUCER, (CO_CONFIG_SYNC_PRODUCER), (0))| CO_CONFIG_GLOBAL_RT_FLAG_CALLBACK_PRE |       \
		      CO_CONFIG_GLOBAL_FLAG_TIMERNEXT | CO_CONFIG_GLOBAL_FLAG_OD_DYNAMIC)
#else
#define CO_CONFIG_SYNC (0)
#endif

/* Process data object (PDO) */
#define CO_CONFIG_PDO                                                                              \
	(CO_CONFIG_RPDO_ENABLE | CO_CONFIG_TPDO_ENABLE | CO_CONFIG_RPDO_TIMERS_ENABLE |            \
	 CO_CONFIG_TPDO_TIMERS_ENABLE |                                                            \
	 COND_CODE_1(CANOPENNODE_PDO_SYNC, (CO_CONFIG_PDO_SYNC_ENABLE), (0)) | CO_CONFIG_PDO_OD_IO_ACCESS |               \
		      CO_CONFIG_GLOBAL_FLAG_CALLBACK_PRE | CO_CONFIG_GLOBAL_FLAG_TIMERNEXT |       \
		      CO_CONFIG_GLOBAL_FLAG_OD_DYNAMIC)

/* Storage */
#ifdef CONFIG_CANOPENNODE_STORAGE
#define CO_CONFIG_STORAGE (CO_CONFIG_STORAGE_ENABLE)
#else
#define CO_CONFIG_STORAGE (0)
#endif

/* Indicator LEDs */
#ifdef CONFIG_CANOPENNODE_LEDS
#define CO_CONFIG_LEDS (CO_CONFIG_LEDS_ENABLE | CO_CONFIG_GLOBAL_FLAG_TIMERNEXT)
#else
#define CO_CONFIG_LEDS (0)
#endif

/* Layer Settings Service (LSS) */
#ifdef CONFIG_CANOPENNODE_LSS_MASTER
#define CO_CONFIG_LSS (CO_CONFIG_LSS_MASTER | CO_CONFIG_GLOBAL_FLAG_CALLBACK_PRE)
#elif defined(CONFIG_CANOPENNODE_LSS_SLAVE)
#define CO_CONFIG_LSS (CO_CONFIG_LSS_SLAVE | CO_CONFIG_GLOBAL_FLAG_CALLBACK_PRE)
#else
#define CO_CONFIG_LSS (0)
#endif

/* Use Zephyr provided crc16 implementation */
#define CO_CONFIG_CRC16 (CO_CONFIG_CRC16_ENABLE | CO_CONFIG_CRC16_EXTERNAL)

/* Use static variables instead of calloc() */
#define CO_USE_GLOBALS

#ifdef CONFIG_LITTLE_ENDIAN
#define CO_LITTLE_ENDIAN
#else
#define CO_BIG_ENDIAN
#endif

#define CO_SWAP_16(x) sys_le16_to_cpu(x)
#define CO_SWAP_32(x) sys_le32_to_cpu(x)
#define CO_SWAP_64(x) sys_le64_to_cpu(x)

typedef bool bool_t;
typedef char char_t;
typedef unsigned char oChar_t;
typedef unsigned char domain_t;

BUILD_ASSERT(sizeof(float32_t) >= 4);
BUILD_ASSERT(sizeof(float64_t) >= 8);

typedef struct canopen_rx_msg {
	uint8_t data[8];
	uint16_t ident;
	uint8_t DLC;
} CO_CANrxMsg_t;

#define CO_CANrxMsg_readIdent(msg) (((CO_CANrxMsg_t *)msg)->ident)
#define CO_CANrxMsg_readDLC(msg)   (((CO_CANrxMsg_t *)msg)->DLC)
#define CO_CANrxMsg_readData(msg)  (((CO_CANrxMsg_t *)msg)->data)

typedef void (*CO_CANrxBufferCallback_t)(void *object, void *message);

typedef struct canopen_rx {
	int filter_id;
	void *object;
	CO_CANrxBufferCallback_t pFunct;
	uint16_t ident;
	uint16_t mask;
#ifdef CONFIG_CAN_ACCEPT_RTR
	bool rtr;
#endif /* CONFIG_CAN_ACCEPT_RTR */
} CO_CANrx_t;

typedef struct canopen_tx {
	uint8_t data[8];
	uint16_t ident;
	uint8_t DLC;
	bool_t rtr;
	bool_t bufferFull;
	bool_t syncFlag;
} CO_CANtx_t;

typedef struct canopen_module {
	const struct device *can_dev;
	struct k_mutex can_send_mutex;
	struct k_mutex emcy_mutex;
	struct k_mutex od_mutex;
	struct k_work tx_retry_work;
	CO_CANrx_t *rx_array;
	CO_CANtx_t *tx_array;
	uint16_t rx_size;
	uint16_t tx_size;
	uint16_t CANerrorStatus;
	bool_t configured;
	bool_t CANnormal;
	bool_t first_tx_msg;
} CO_CANmodule_t;

typedef struct canopen_storage_entry {
	const char *key;
	void *addr;
	size_t len;
	uint8_t subIndexOD;
	uint8_t attr;
} CO_storage_entry_t;

#define CO_LOCK_CAN_SEND(mod)   k_mutex_lock(&mod->can_send_mutex, K_FOREVER)
#define CO_UNLOCK_CAN_SEND(mod) k_mutex_unlock(&mod->can_send_mutex)

#define CO_LOCK_EMCY(mod)   k_mutex_lock(&mod->emcy_mutex, K_FOREVER)
#define CO_UNLOCK_EMCY(mod) k_mutex_unlock(&mod->emcy_mutex)

#define CO_LOCK_OD(mod)   k_mutex_lock(&mod->od_mutex, K_FOREVER)
#define CO_UNLOCK_OD(mod) k_mutex_unlock(&mod->od_mutex)

/*
 * CANopenNode RX callbacks run in interrupt context, no memory
 * barrier needed.
 */
#define CO_MemoryBarrier()
#define CO_FLAG_READ(rxNew) ((rxNew) != NULL)
#define CO_FLAG_SET(rxNew)                                                                         \
	do {                                                                                       \
		CO_MemoryBarrier();                                                                \
		rxNew = (void *)1L;                                                                \
	} while (0)
#define CO_FLAG_CLEAR(rxNew)                                                                       \
	do {                                                                                       \
		CO_MemoryBarrier();                                                                \
		rxNew = NULL;                                                                      \
	} while (0)

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_MODULES_CANOPENNODE_CO_DRIVER_H */
