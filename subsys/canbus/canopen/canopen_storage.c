/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 Linumiz
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>

#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/crc.h>
#include <zephyr/sys/printk.h>

#include "canopen_internal.h"

LOG_MODULE_DECLARE(canopen, CONFIG_CANOPEN_LOG_LEVEL);

#define CANOPEN_OD_STORE 0x1010U
#define CANOPEN_OD_RPDO  0x1400U
#define CANOPEN_OD_TPDO  0x1800U

#define STORAGE_SUB_ALL     1U
#define STORAGE_SUB_COMM    2U
#define STORAGE_SUB_PROFILE 3U
#define STORAGE_SUB_MANUF   4U

#define STORAGE_MANUF_FIRST   0x2000U
#define STORAGE_MANUF_LAST    0x5FFFU
#define STORAGE_PROFILE_FIRST 0x6000U
#define STORAGE_PROFILE_LAST  0x9FFFU

#define STORAGE_SAVE       0x65766173U
#define STORAGE_LOAD       0x64616F6CU
#define STORAGE_KEY_LEN    16
#define STORAGE_HDR_LEN    3U
#define STORAGE_APP_GROUPS (BIT(STORAGE_SUB_PROFILE) | BIT(STORAGE_SUB_MANUF))
#define STORAGE_GROUPS     (BIT(STORAGE_SUB_COMM) | STORAGE_APP_GROUPS)

struct storage_obj {
	uint16_t index;
	uint8_t sub;
	uint8_t count;
};

enum storage_op {
	STORAGE_CHECK,
	STORAGE_PACK,
	STORAGE_UNPACK,
};

struct storage_rec {
	size_t len;
	uint16_t crc;
	uint8_t groups;
	uint8_t group;
	enum storage_op op;
};

static const uint8_t storage_highest = STORAGE_SUB_MANUF;
static const uint32_t storage_capability = BIT(0);

static const struct storage_obj storage_comm_objs[] = {
	{0x1005U, 0U, 1U},
	{0x1006U, 0U, 1U},
	{0x1007U, 0U, 1U},
	{CANOPEN_OD_GUARD_TIME, 0U, 1U},
	{CANOPEN_OD_LIFE_FACTOR, 0U, 1U},
	{CANOPEN_OD_TIME, 0U, 1U},
	{0x1014U, 0U, 1U},
	{0x1015U, 0U, 1U},
	{CANOPEN_OD_HB_CONSUMER, 1U, CONFIG_CANOPEN_HB_CONSUMER_COUNT},
	{0x1017U, 0U, 1U},
	{0x1019U, 0U, 1U},
	{0x1029U, 1U, 1U},
};

#define STORAGE_COMM_LEN                                                                           \
	(((ARRAY_SIZE(storage_comm_objs) + CONFIG_CANOPEN_HB_CONSUMER_COUNT) * sizeof(uint32_t)) + \
	 ((CONFIG_CANOPEN_TPDO_COUNT + CONFIG_CANOPEN_RPDO_COUNT) *                                \
	  sizeof(struct canopen_pdo_param)))

static uint8_t storage_buf[STORAGE_HDR_LEN + STORAGE_COMM_LEN + CONFIG_CANOPEN_STORAGE_APP_SIZE];
static K_MUTEX_DEFINE(storage_lock);

static int storage_item(struct storage_rec *rec, uint16_t index, uint8_t sub, const void *src,
			void *dst, size_t size)
{
	uint8_t desc[5];

	if (size > (sizeof(storage_buf) - rec->len)) {
		return -ENOMEM;
	}

	sys_put_le16(index, &desc[0]);
	desc[2] = sub;
	sys_put_le16((uint16_t)size, &desc[3]);
	rec->crc = crc16_itu_t(rec->crc, desc, sizeof(desc));

	if ((rec->groups & BIT(rec->group)) != 0U) {
		if (rec->op == STORAGE_PACK) {
			memcpy(&storage_buf[rec->len], src, size);
		} else if ((rec->op == STORAGE_UNPACK) && (dst != NULL)) {
			memcpy(dst, &storage_buf[rec->len], size);
		}
	}

	rec->len += size;

	return 0;
}

static int storage_od(struct canopen_node *node, struct storage_rec *rec, uint16_t index,
		      uint8_t sub, bool persist)
{
	struct canopen_od_ref ref;

	if ((canopen_od_find(node, index, sub, &ref) != CANOPEN_SDO_ABORT_NONE) ||
	    (persist && ((ref.attr & CANOPEN_OD_ATTR_PERSIST) == 0U))) {
		return 0;
	}

	return storage_item(rec, index, sub, ref.src, ref.dst, ref.size);
}

static int storage_pdos(struct storage_rec *rec, struct canopen_pdo *pdos, uint16_t count,
			uint16_t index)
{
	int err = 0;

	for (uint16_t n = 0U; (n < count) && (err == 0); n++) {
		err = storage_item(rec, index + n, 0U, &pdos[n].param, &pdos[n].param,
				   sizeof(pdos[n].param));
	}

	return err;
}

static int storage_comm(struct canopen_node *node, struct storage_rec *rec)
{
	int err = 0;

	for (size_t i = 0U; (i < ARRAY_SIZE(storage_comm_objs)) && (err == 0); i++) {
		const struct storage_obj *obj = &storage_comm_objs[i];

		for (uint8_t sub = obj->sub; (sub < (obj->sub + obj->count)) && (err == 0); sub++) {
			err = storage_od(node, rec, obj->index, sub, false);
		}
	}

	if (err == 0) {
		err = storage_pdos(rec, node->tpdo, CONFIG_CANOPEN_TPDO_COUNT, CANOPEN_OD_TPDO);
	}

	if (err == 0) {
		err = storage_pdos(rec, node->rpdo, CONFIG_CANOPEN_RPDO_COUNT, CANOPEN_OD_RPDO);
	}

	return err;
}

static int storage_app(struct canopen_node *node, struct storage_rec *rec, uint16_t first,
		       uint16_t last)
{
	const struct canopen_od *od = node->cfg->od;
	int err = 0;

	if (od == NULL) {
		return 0;
	}

	for (size_t i = 0U; (i < od->count) && (err == 0); i++) {
		const struct canopen_od_entry *entry = &od->entries[i];

		if ((entry->index < first) || (entry->index > last)) {
			continue;
		}

		for (uint8_t j = 0U; (j < entry->count) && (err == 0); j++) {
			uint8_t sub = (entry->code == CANOPEN_OD_CODE_ARRAY) ? (j + 1U)
									      : entry->subs[j].sub;

			err = storage_od(node, rec, entry->index, sub, true);
		}
	}

	return err;
}

static int storage_walk(struct canopen_node *node, struct storage_rec *rec)
{
	int err;

	rec->len = STORAGE_HDR_LEN;
	rec->crc = 0U;

	rec->group = STORAGE_SUB_COMM;
	err = storage_comm(node, rec);

	if (err == 0) {
		rec->group = STORAGE_SUB_PROFILE;
		err = storage_app(node, rec, STORAGE_PROFILE_FIRST, STORAGE_PROFILE_LAST);
	}

	if (err == 0) {
		rec->group = STORAGE_SUB_MANUF;
		err = storage_app(node, rec, STORAGE_MANUF_FIRST, STORAGE_MANUF_LAST);
	}

	return err;
}

static uint8_t storage_read(struct canopen_node *node, const char *key, struct storage_rec *rec)
{
	ssize_t len;
	int err;

	len = settings_load_one(key, storage_buf, sizeof(storage_buf));
	rec->groups = 0U;
	rec->op = STORAGE_CHECK;
	err = storage_walk(node, rec);
	if ((len <= 0) || (err != 0)) {
		return 0U;
	}

	if (((size_t)len != rec->len) || (sys_get_le16(storage_buf) != rec->crc)) {
		LOG_WRN("stored parameters %s ignored", key);
		return 0U;
	}

	return storage_buf[2] & STORAGE_GROUPS;
}

static void storage_key(struct canopen_node *node, char *key)
{
	snprintk(key, STORAGE_KEY_LEN, "canopen/%u", node->cfg->node_id);
}

void canopen_storage_load(struct canopen_node *node, bool app)
{
	struct storage_rec rec;
	char key[STORAGE_KEY_LEN];

	storage_key(node, key);
	k_mutex_lock(&storage_lock, K_FOREVER);

	rec.groups = storage_read(node, key, &rec) &
		     (app ? STORAGE_APP_GROUPS : BIT(STORAGE_SUB_COMM));
	if (rec.groups != 0U) {
		rec.op = STORAGE_UNPACK;
		(void)storage_walk(node, &rec);
	}

	k_mutex_unlock(&storage_lock);
}

uint32_t canopen_storage_find(uint16_t index, uint8_t sub, struct canopen_od_ref *ref)
{
	if (sub == 0U) {
		canopen_od_comm_ref(ref, index, sub, CANOPEN_OD_TYPE_UNSIGNED8, CANOPEN_OD_CONST,
				    &storage_highest, NULL, sizeof(storage_highest));
	} else if (sub <= STORAGE_SUB_MANUF) {
		canopen_od_comm_ref(ref, index, sub, CANOPEN_OD_TYPE_UNSIGNED32, CANOPEN_OD_RW,
				    &storage_capability, NULL, sizeof(storage_capability));
	} else {
		return CANOPEN_SDO_ABORT_NO_SUB_INDEX;
	}

	return CANOPEN_SDO_ABORT_NONE;
}

uint32_t canopen_storage_write(struct canopen_node *node, uint16_t index, uint8_t sub,
			       uint32_t value)
{
	bool save = index == CANOPEN_OD_STORE;
	uint8_t groups = (sub == STORAGE_SUB_ALL) ? STORAGE_GROUPS : BIT(sub);
	struct storage_rec rec;
	char key[STORAGE_KEY_LEN];
	uint8_t valid;
	int err;

	if (value != (save ? STORAGE_SAVE : STORAGE_LOAD)) {
		return CANOPEN_SDO_ABORT_DATA_TRANSFER;
	}

	storage_key(node, key);
	k_mutex_lock(&storage_lock, K_FOREVER);

	valid = storage_read(node, key, &rec);
	if (save) {
		valid |= groups;
		rec.groups = groups;
		rec.op = STORAGE_PACK;
		err = storage_walk(node, &rec);
	} else {
		valid &= ~groups;
		err = 0;
	}

	if ((err == 0) && (valid == 0U)) {
		err = settings_delete(key);
	} else if (err == 0) {
		sys_put_le16(rec.crc, storage_buf);
		storage_buf[2] = valid;
		err = settings_save_one(key, storage_buf, rec.len);
	}

	k_mutex_unlock(&storage_lock);

	if (err == -ENOMEM) {
		LOG_ERR("parameters do not fit in the storage buffer");
		return CANOPEN_SDO_ABORT_OUT_OF_MEMORY;
	}

	if (err != 0) {
		LOG_ERR("failed to %s parameters (%d)", save ? "store" : "restore", err);
		return CANOPEN_SDO_ABORT_HARDWARE;
	}

	return CANOPEN_SDO_ABORT_NONE;
}
