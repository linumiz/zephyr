/*
 * Copyright (c) 2019 Vestas Wind Systems A/S
 * Copyright (c) 2025 National Taiwan University Racing Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

// glibc includes
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>

// zephyr includes
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/iterable_sections.h>

// canopennode includes
#include <CANopen.h>
#include <storage/CO_storage.h>

#include <canopennode.h>

LOG_MODULE_REGISTER(canopen_storage, CONFIG_CANOPEN_LOG_LEVEL);

/* type ----------------------------------------------------------------------*/
struct canopen_storage_ctx {
	int settings_error;
	CO_storage_t storage;
	CO_storage_entry_t *storage_entries;
	size_t num_entry;
};

/* static function declaration -----------------------------------------------*/
static ODR_t store(CO_storage_entry_t *entry, CO_CANmodule_t *CANmodule);
static ODR_t restore(CO_storage_entry_t *entry, CO_CANmodule_t *CANmodule);

static int canopen_settings_set(const char *key, size_t len_rd, settings_read_cb read_cb,
				void *cb_arg);

/* static variable -----------------------------------------------------------*/
static struct canopen_storage_ctx g_ctx = {
	.settings_error = 0,
	.storage =
		{
			.enabled = false,
		},
};

SETTINGS_STATIC_HANDLER_DEFINE(canopen, CONFIG_CANOPENNODE_STORAGE_SUBTREE, NULL,
			       canopen_settings_set, NULL, NULL);

/* function definition -------------------------------------------------------*/
int canopen_storage_init()
{
	int err;
	OD_entry_t *OD_1010;
	OD_entry_t *OD_1011;

	STRUCT_SECTION_GET(canopen_storage_entry, 0, &g_ctx.storage_entries);
	STRUCT_SECTION_COUNT(canopen_storage_entry, &g_ctx.num_entry);

	OD_1010 = OD_find(OD, OD_H1010_STORE_PARAMETERS);
	if (OD_1010 == NULL) {
		LOG_ERR("object dictionary error at entry 0x1010");
		return -EINVAL;
	}

	OD_1011 = OD_find(OD, OD_H1011_RESTORE_DEFAULT);
	if (OD_1011 == NULL) {
		LOG_ERR("object dictionary error at entry 0x1011");
		return -EINVAL;
	}

	err = CO_storage_init(&g_ctx.storage, CO->CANmodule, OD_1010, OD_1011, store, restore,
			      g_ctx.storage_entries, g_ctx.num_entry);
	if (err != CO_ERROR_NO) {
		LOG_ERR("CO_storage_init failed (err %d)", err);
		err = -EIO;
		goto err;
	}

	err = settings_subsys_init();
	if (err < 0) {
		LOG_ERR("failed to initialize settings subsystem (err %d)", err);
		goto err;
	}

	err = settings_load_subtree(CONFIG_CANOPENNODE_STORAGE_SUBTREE);
	if (err < 0) {
		LOG_ERR("failed to load settings subtree (err %d)", err);
		goto err;
	}

	if (g_ctx.settings_error < 0) {
		err = g_ctx.settings_error;
		goto err;
	}

	g_ctx.storage.enabled = true;

	return 0;

err:
	CO_error(CO->em, true, CO_EM_NON_VOLATILE_MEMORY, CO_EMC_HARDWARE, err);

	return err;
}

int canopen_storage_process()
{
	int err;

	for (size_t i = 0; i < g_ctx.num_entry; i++) {
		if (g_ctx.storage_entries[i].attr & CO_storage_auto) {
			err = store(&g_ctx.storage_entries[i], CO->CANmodule);
			if (err != ODR_OK) {
				return err;
			}
		}
	}

	return 0;
}

/* static function definition ------------------------------------------------*/
static ODR_t store(CO_storage_entry_t *entry, CO_CANmodule_t *CANmodule)
{
	int err;
	char key[100];

	ARG_UNUSED(CANmodule);

	snprintf(key, sizeof(key), CONFIG_CANOPENNODE_STORAGE_SUBTREE "/%s", entry->key);
	err = settings_save_one(key, entry->addr, entry->len);
	if (err < 0) {
		LOG_ERR("failed to save settings data %s (err %d)", entry->key, err);
		return ODR_HW;
	}

	LOG_INF("Saved CANopen storage group %s to settings %s", entry->key, key);

	return ODR_OK;
}

static ODR_t restore(CO_storage_entry_t *entry, CO_CANmodule_t *CANmodule)
{
	int err;
	char key[100];

	ARG_UNUSED(CANmodule);

	snprintf(key, sizeof(key), CONFIG_CANOPENNODE_STORAGE_SUBTREE "/%s", entry->key);
	err = settings_delete(key);
	if (err < 0) {
		LOG_ERR("failed to delete settings data %s (err %d)", entry->key, err);
		return ODR_HW;
	}

	LOG_INF("Deleted CANopen storage group %s from settings %s", entry->key, key);

	return ODR_OK;
}

static int canopen_settings_set(const char *key, size_t len_rd, settings_read_cb read_cb,
				void *cb_arg)
{
	const char *next;
	ssize_t len;
	size_t i;

	for (i = 0; i < g_ctx.num_entry; i++) {
		if (settings_name_steq(key, g_ctx.storage_entries[i].key, &next) && !next) {
			len = read_cb(cb_arg, g_ctx.storage_entries[i].addr,
				      g_ctx.storage_entries[i].len);
			if (len < 0) {
				LOG_ERR("failed to load settings data %s (err %d)",
					g_ctx.storage_entries[i].key, len);
				g_ctx.settings_error = len;
				return len;
			}

			return 0;
		}
	}

	return -ENOENT;
}
