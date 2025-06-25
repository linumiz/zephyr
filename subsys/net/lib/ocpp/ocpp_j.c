/*
 * Copyright (c) 2024 Linumiz
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "ocpp_i.h"
#include "ocpp_j.h"
#include <zephyr/data/json.h>
#include <zephyr/random/random.h>

static int strip_optional_field(char *json, char *key, enum strip_mode mode)
{
	char pattern[STRIP_PATTERN_LEN];
	char *start;
	char *end;

	if (!json || !key) {
		return -EINVAL;
	}

	switch (mode) {
	case STRIP_STRING:
		snprintk(pattern, sizeof(pattern), "\"%s\":\"NULL\"", key);
		start = strstr(json, pattern);
		if (!start) {
			return 0;
		}
		end = start + strlen(pattern);
		break;

	case STRIP_INT:
		snprintk(pattern, sizeof(pattern), "\"%s\":-1", key);
		start = strstr(json, pattern);
		if (!start) {
			return 0;
		}
		end = start + strlen(pattern);
		break;

	case STRIP_ARRAY:
		snprintk(pattern, sizeof(pattern), "\"%s\":[", key);
		start = strstr(json, pattern);
		if (!start) {
			return 0;
		}
		end = strchr(start, ']');
		if (!end) {
			return -EINVAL;
		}
		end++;
		break;

	default:
		return -EINVAL;
	}

	if (*end == ',') {
		end++;
	}

	else if (start > json && *(start - 1) == ',') {
		start--;
	}

	memmove(start, end, strlen(end) + 1);
	return 0;
}

static int extract_string_field(char *out_buf, int outlen, char *token)
{
	char *end;

	if (!out_buf || !token) {
		return -EINVAL;
	}

	strncpy(out_buf, token + 1, outlen - 1);
	end = strchr(out_buf, '"');
	if (end) {
		*end = '\0';
	}

	return 0;
}

static int extract_payload(char *msg, int msglen)
{
	size_t len;
	char *start = strchr(msg, '{');
	char *end = strrchr(msg, '}');

	if (!start || !end || end < start) {
		return -EINVAL;
	}

	len = end - start + 1;
	if (len >= msglen) {
		return -ENOMEM;
	}

	memmove(msg, start, len);
	msg[len] = '\0';

	return 0;
}

static int frame_rpc_call_req(char *rpcbuf, int len, int pdu,
			      uint32_t ses, char *pdumsg)
{
	int ret;
	char uid[MAX_UID_LEN];
	char *action;
	uint32_t rnd = sys_rand32_get();

	snprintk(uid, sizeof(uid), "%u-%d-%u", ses, pdu, rnd);

	action = ocpp_get_pdu_literal(pdu);
	if (!action) {
		return -EINVAL;
	}

	ret = snprintk(rpcbuf, len,
		       "[2,\"%s\",\"%s\",%s]",
		       uid, action, pdumsg);

	if (ret < 0 || ret >= len) {
		return -ENOMEM;
	}

	return 0;
}

static int frame_rpc_call_res(char *rpcbuf, int len,
			      char *uid, char *pdumsg)
{
	int ret;

	ret = snprintk(rpcbuf, len, "[3,\"%s\",%s]", uid, pdumsg);

	if (ret < 0 || ret >= len) {
		return -ENOMEM;
	}

	return 0;
}

static int frame_authorize_msg(char *buf, int len,
			       struct ocpp_session *ses)
{
	int ret;
	char auth_obj[MAX_AUTH_REQ_LEN];

	struct json_obj_descr authorize_descr[] = {
		JSON_OBJ_DESCR_PRIM_NAMED(struct authorize_payload, "idTag",
					  id_tag, JSON_TOK_STRING),
	};

	struct authorize_payload payload = {
		.id_tag = ses->idtag,
	};

	ret = json_obj_encode_buf(authorize_descr,
				  ARRAY_SIZE(authorize_descr),
				  &payload,
				  auth_obj,
				  sizeof(auth_obj));
	if (ret < 0) {
		return ret;
	}

	ret = frame_rpc_call_req(buf, len, PDU_AUTHORIZE,
				 (uint32_t)ses, auth_obj);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

static int frame_heartbeat_msg(char *buf, int len, struct ocpp_session *ses)
{
	int ret;
	char tmp_buf[MAX_HEARTBEAT_REQ_LEN];

	struct json_obj_descr descr[] = {
	};

	struct ocpp_heartbeat_msg msg = {
		.dummy = true,
	};

	ret = json_obj_encode_buf(descr, ARRAY_SIZE(descr), &msg,
				  tmp_buf, sizeof(tmp_buf));
	if (ret < 0) {
		return ret;
	}

	ret = frame_rpc_call_req(buf, len, PDU_HEARTBEAT,
				 (uint32_t)ses, tmp_buf);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

static int frame_bootnotif_msg(char *buf, int len,
			       struct ocpp_session *ses,
			       struct ocpp_cp_info *cpi)
{
	int ret;
	char tmp_buf[MAX_BOOTNOTIF_REQ_LEN];

	struct json_obj_descr bootnotif_descr[] = {
		JSON_OBJ_DESCR_PRIM_NAMED(struct ocpp_bootnotif_msg, "chargePointModel",
					  charge_point_model, JSON_TOK_STRING),
		JSON_OBJ_DESCR_PRIM_NAMED(struct ocpp_bootnotif_msg, "chargePointVendor",
					  charge_point_vendor, JSON_TOK_STRING),
		JSON_OBJ_DESCR_PRIM_NAMED(struct ocpp_bootnotif_msg, "chargeBoxSerialNumber",
					  charge_box_serial_number, JSON_TOK_STRING),
		JSON_OBJ_DESCR_PRIM_NAMED(struct ocpp_bootnotif_msg, "chargePointSerialNumber",
					  charge_point_serial_number, JSON_TOK_STRING),
		JSON_OBJ_DESCR_PRIM_NAMED(struct ocpp_bootnotif_msg, "firmwareVersion",
					  firmware_version, JSON_TOK_STRING),
		JSON_OBJ_DESCR_PRIM_NAMED(struct ocpp_bootnotif_msg, "iccid",
					  iccid, JSON_TOK_STRING),
		JSON_OBJ_DESCR_PRIM_NAMED(struct ocpp_bootnotif_msg, "imsi",
					  imsi, JSON_TOK_STRING),
		JSON_OBJ_DESCR_PRIM_NAMED(struct ocpp_bootnotif_msg, "meterSerialNumber",
					  meter_serial_number, JSON_TOK_STRING),
		JSON_OBJ_DESCR_PRIM_NAMED(struct ocpp_bootnotif_msg, "meterType",
					  meter_type, JSON_TOK_STRING),
	};

	struct ocpp_bootnotif_msg msg = {
		.charge_point_model = cpi->model,
		.charge_point_vendor = cpi->vendor,
		.charge_box_serial_number = cpi->box_sl_no ? cpi->box_sl_no : "NULL",
		.charge_point_serial_number = cpi->sl_no ? cpi->sl_no : "NULL",
		.firmware_version = cpi->fw_ver ? cpi->fw_ver : "NULL",
		.iccid = cpi->iccid ? cpi->iccid : "NULL",
		.imsi = cpi->imsi ? cpi->imsi : "NULL",
		.meter_serial_number = cpi->meter_sl_no ? cpi->meter_sl_no : "NULL",
		.meter_type = cpi->meter_type ? cpi->meter_type : "NULL",
	};

	ret = json_obj_encode_buf(bootnotif_descr, ARRAY_SIZE(bootnotif_descr),
				  &msg, tmp_buf, sizeof(tmp_buf));
	if (ret < 0) {
		return ret;
	}

	if (strcmp(msg.firmware_version, "NULL") == 0) {
		strip_optional_field(tmp_buf, "firmwareVersion", STRIP_STRING);
	};
	if (strcmp(msg.iccid, "NULL") == 0) {
		strip_optional_field(tmp_buf, "iccid", STRIP_STRING);
	};
	if (strcmp(msg.imsi, "NULL") == 0) {
		strip_optional_field(tmp_buf, "imsi", STRIP_STRING);
	};
	if (strcmp(msg.charge_box_serial_number, "NULL") == 0) {
		strip_optional_field(tmp_buf, "chargeBoxSerialNumber", STRIP_STRING);
	};
	if (strcmp(msg.charge_point_serial_number, "NULL") == 0) {
		strip_optional_field(tmp_buf, "chargePointSerialNumber", STRIP_STRING);
	};
	if (strcmp(msg.meter_serial_number, "NULL") == 0) {
		strip_optional_field(tmp_buf, "meterSerialNumber", STRIP_STRING);
	};
	if (strcmp(msg.meter_type, "NULL") == 0) {
		strip_optional_field(tmp_buf, "meterType", STRIP_STRING);
	};

	ret = frame_rpc_call_req(buf, len, PDU_BOOTNOTIFICATION,
				 (uint32_t)ses, tmp_buf);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

static int frame_meter_val_msg(char *buf, int len, struct ocpp_session *ses, char *timestamp,
			       char *val, char *measurand, char *unit)
{
	int ret = 0;
	char tmp_buf[MAX_METERVALUE_REQ_LEN];

	struct json_obj_descr sampled_value_descr[] = {
		JSON_OBJ_DESCR_PRIM_NAMED(struct ocpp_sampled_value, "measurand",
					  measurand, JSON_TOK_STRING),
		JSON_OBJ_DESCR_PRIM_NAMED(struct ocpp_sampled_value, "value",
					  value, JSON_TOK_STRING),
		JSON_OBJ_DESCR_PRIM_NAMED(struct ocpp_sampled_value, "unit",
					  unit, JSON_TOK_STRING),
	};

	struct json_obj_descr meter_value_descr[] = {
		JSON_OBJ_DESCR_PRIM_NAMED(struct ocpp_meter_value, "timestamp",
					  timestamp, JSON_TOK_STRING),
		JSON_OBJ_DESCR_OBJ_ARRAY_NAMED(struct ocpp_meter_value, "sampledValue",
					       sampled_value, 1, sampled_value_len,
					       sampled_value_descr,
					       ARRAY_SIZE(sampled_value_descr)),
	};

	struct json_obj_descr meter_val_msg_descr[] = {
		JSON_OBJ_DESCR_PRIM_NAMED(struct ocpp_meter_val_msg, "connectorId",
					  connector_id, JSON_TOK_NUMBER),
		JSON_OBJ_DESCR_PRIM_NAMED(struct ocpp_meter_val_msg, "transactionId",
					  transaction_id, JSON_TOK_NUMBER),
		JSON_OBJ_DESCR_OBJ_ARRAY_NAMED(struct ocpp_meter_val_msg, "meterValue",
					       meter_value, 1, meter_value_len,
					       meter_value_descr,
					       ARRAY_SIZE(meter_value_descr)),
	};

	struct ocpp_sampled_value sv = {
		.measurand = measurand,
		.value = val,
		.unit = unit ? unit : "NULL",
	};

	struct ocpp_meter_value mv = {
		.timestamp = timestamp,
		.sampled_value_len = 1,
		.sampled_value = { sv },
	};

	struct ocpp_meter_val_msg msg = {
		.connector_id = ses ? ses->idcon : 0,
		.transaction_id = ses ? ses->idtxn : 0,
		.meter_value_len = 1,
		.meter_value = { mv },
	};

	ret = json_obj_encode_buf(meter_val_msg_descr,
				  ARRAY_SIZE(meter_val_msg_descr),
				  &msg, tmp_buf, sizeof(tmp_buf));
	if (ret < 0) {
		return ret;
	}

	if (strcmp(sv.unit, "NULL") == 0) {
		strip_optional_field(tmp_buf, "unit", STRIP_STRING);
	};

	ret = frame_rpc_call_req(buf, len, PDU_METER_VALUES,
				 (uint32_t)ses, tmp_buf);

	if (ret < 0) {
		return ret;
	}

	return 0;
}

static int frame_stop_txn_msg(char *buf, int len, struct ocpp_session *ses,
			      int Wh, char *reason, char *timestamp)
{
	int ret = 0;
	char tmp_buf[MAX_STOPTRANSACTION_REQ_LEN];

	struct json_obj_descr descr[] = {
		JSON_OBJ_DESCR_PRIM_NAMED(struct ocpp_stop_txn_msg, "transactionId",
					  transaction_id, JSON_TOK_NUMBER),
		JSON_OBJ_DESCR_PRIM_NAMED(struct ocpp_stop_txn_msg, "meterStop",
					  meter_stop, JSON_TOK_NUMBER),
		JSON_OBJ_DESCR_PRIM_NAMED(struct ocpp_stop_txn_msg, "timestamp",
					  timestamp, JSON_TOK_STRING),
		JSON_OBJ_DESCR_PRIM_NAMED(struct ocpp_stop_txn_msg, "reason",
					  reason, JSON_TOK_STRING),
		JSON_OBJ_DESCR_PRIM_NAMED(struct ocpp_stop_txn_msg, "idTag",
					  id_tag, JSON_TOK_STRING),
	};

	struct ocpp_stop_txn_msg msg = {
		.transaction_id = ses->idtxn,
		.meter_stop = Wh,
		.timestamp = timestamp,
		.reason = reason ? reason : "NULL",
		.id_tag = ses->idtag[0] ? ses->idtag : "NULL",
	};

	ret = json_obj_encode_buf(descr, ARRAY_SIZE(descr), &msg,
				  tmp_buf, sizeof(tmp_buf));
	if (ret < 0) {
		return ret;
	}

	if (strcmp(msg.reason, "NULL") == 0) {
		strip_optional_field(tmp_buf, "reason", STRIP_STRING);
	};

	if (strcmp(msg.id_tag, "NULL") == 0) {
		strip_optional_field(tmp_buf, "idTag", STRIP_STRING);
	};

	ret = frame_rpc_call_req(buf, len, PDU_STOP_TRANSACTION,
				 (uint32_t)ses, tmp_buf);

	if (ret < 0) {
		return ret;
	}

	return 0;
}

static int frame_start_txn_msg(char *buf, int len, struct ocpp_session *ses,
			       int Wh, int reserv_id, char *timestamp)
{
	int ret = 0;
	char tmp_buf[MAX_STARTTRANSACTION_REQ_LEN];

	struct json_obj_descr descr[] = {
		JSON_OBJ_DESCR_PRIM_NAMED(struct ocpp_start_txn_msg, "connectorId",
					  connector_id, JSON_TOK_NUMBER),
		JSON_OBJ_DESCR_PRIM_NAMED(struct ocpp_start_txn_msg, "idTag",
					  id_tag, JSON_TOK_STRING),
		JSON_OBJ_DESCR_PRIM_NAMED(struct ocpp_start_txn_msg, "meterStart",
					  meter_start, JSON_TOK_NUMBER),
		JSON_OBJ_DESCR_PRIM_NAMED(struct ocpp_start_txn_msg, "timestamp",
					  timestamp, JSON_TOK_STRING),
		JSON_OBJ_DESCR_PRIM_NAMED(struct ocpp_start_txn_msg, "reservationId",
					  reservation_id, JSON_TOK_NUMBER),
	};

	struct ocpp_start_txn_msg msg = {
		.connector_id = ses->idcon,
		.id_tag = ses->idtag,
		.meter_start = Wh,
		.timestamp = timestamp,
		.reservation_id = (reserv_id >= 0) ? reserv_id : -1,
	};

	ret = json_obj_encode_buf(descr, ARRAY_SIZE(descr), &msg,
				  tmp_buf, sizeof(tmp_buf));
	if (ret < 0) {
		return ret;
	}

	if (msg.reservation_id == -1) {
		strip_optional_field(tmp_buf, "reservationId", STRIP_INT);
	};

	ret = frame_rpc_call_req(buf, len, PDU_START_TRANSACTION,
				 (uint32_t)ses, tmp_buf);

	if (ret < 0) {
		return ret;
	}

	return 0;
}

static int frame_getconfig_msg(char *buf, int len, char *key, char *val,
			       bool is_rw, char *uid)
{
	int ret = 0;
	char tmp_buf[MAX_GETCONFIGURATION_RESP_LEN];

	struct json_obj_descr keyval_descr[] = {
		JSON_OBJ_DESCR_PRIM_NAMED(struct ocpp_key_val, "key",
					  key, JSON_TOK_STRING),
		JSON_OBJ_DESCR_PRIM_NAMED(struct ocpp_key_val, "readonly",
					  readonly, JSON_TOK_NUMBER),
		JSON_OBJ_DESCR_PRIM_NAMED(struct ocpp_key_val, "value",
					  value, JSON_TOK_STRING),
	};

	struct json_obj_descr config_descr[] = {
		JSON_OBJ_DESCR_OBJ_ARRAY_NAMED(struct ocpp_getconfig_msg, "configurationKey",
					       configuration_key, 1, configuration_key_len,
					       keyval_descr, ARRAY_SIZE(keyval_descr)),
		JSON_OBJ_DESCR_PRIM_NAMED(struct ocpp_getconfig_msg, "unknownKey",
					  unknown_key, JSON_TOK_STRING),
	};

	struct ocpp_getconfig_msg msg = { 0 };

	msg.configuration_key_len = 0;
	msg.unknown_key = "NULL";

	if (val) {
		msg.configuration_key[0].key = key;
		msg.configuration_key[0].readonly = !is_rw;
		msg.configuration_key[0].value = val;
		msg.configuration_key_len = 1;
	} else {
		msg.unknown_key = key;
	}

	ret = json_obj_encode_buf(config_descr, ARRAY_SIZE(config_descr), &msg,
				  tmp_buf, sizeof(tmp_buf));
	if (ret < 0) {
		return ret;
	}

	if (strcmp(msg.unknown_key, "NULL") == 0) {
		strip_optional_field(tmp_buf, "unknownKey", STRIP_STRING);
	}

	if (msg.configuration_key_len == 0) {
		strip_optional_field(tmp_buf, "configurationKey", STRIP_ARRAY);
	}

	ret = frame_rpc_call_res(buf, len, uid, tmp_buf);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

static int frame_status_resp_msg(char *buf, int len, char *res, char *uid)
{
	int ret = 0;
	char tmp_buf[MAX_UNLOCKCONNECTOR_RESP_LEN];

	struct json_obj_descr descr[] = {
		JSON_OBJ_DESCR_PRIM_NAMED(struct ocpp_status_resp_msg, "status",
					  status, JSON_TOK_STRING),
	};

	struct ocpp_status_resp_msg msg = {
		.status = res,
	};

	ret = json_obj_encode_buf(descr, ARRAY_SIZE(descr), &msg,
				  tmp_buf, sizeof(tmp_buf));
	if (ret < 0) {
		return ret;
	}

	ret = frame_rpc_call_res(buf, len, uid, tmp_buf);

	if (ret < 0) {
		return ret;
	}

	return 0;
}

/* parse msg from server */
int parse_rpc_msg(char *msg, int msglen, char *uid, int uidlen,
		  int *pdu, bool *is_rsp)
{
	int ret = 0;
	char local_buf[MAX_OCPP_RPC_MESSAGE_LEN];
	char action[32];
	char *token;
	int rpc_id = -1;

	if (!msg || !uid || !pdu || !is_rsp) {
		return -EINVAL;
	}

	memcpy(local_buf, msg + 1, sizeof(local_buf) - 1);
	local_buf[sizeof(local_buf) - 1] = '\0';

	token = strtok(local_buf, ",");
	if (!token) {
		return -EINVAL;
	}

	rpc_id = *token - '0';

	token = strtok(NULL, ",");
	if (!token) {
		return -EINVAL;
	}

	ret = extract_string_field(uid, uidlen, token);
	if (ret < 0) {
		return ret;
	}

	switch (rpc_id + '0') {
	case OCPP_WAMP_RPC_REQ:
		token = strtok(NULL, ",");
		if (!token) {
			return -EINVAL;
		}

		ret = extract_string_field(action, sizeof(action), token);
		if (ret < 0) {
			return ret;
		}
		*pdu = ocpp_find_pdu_from_literal(action);
		/* fall through */

	case OCPP_WAMP_RPC_RESP:
		*is_rsp = rpc_id - 2;
		ret = extract_payload(msg, msglen);
		if (ret < 0) {
			return ret;
		}
		break;

	case OCPP_WAMP_RPC_ERR:
		/* fall through */

	default:
		return -EINVAL;
	}

	return 0;
}

static int parse_idtag_info(char *json, struct ocpp_idtag_info *idtag_info)
{
	int ret = 0;
	char *status;

	struct json_obj_descr inner_descr[] = {
		JSON_OBJ_DESCR_PRIM_NAMED(struct idtag_info_inner, "status",
					  status, JSON_TOK_STRING),
		JSON_OBJ_DESCR_PRIM_NAMED(struct idtag_info_inner, "parentIdTag",
					  parent_id_tag, JSON_TOK_STRING),
		JSON_OBJ_DESCR_PRIM_NAMED(struct idtag_info_inner, "expiryDate",
					  expiry_date, JSON_TOK_STRING),
	};

	struct json_obj_descr root_descr[] = {
		JSON_OBJ_DESCR_OBJECT_NAMED(struct idtag_info_root, "idTagInfo",
					    id_tag_info, inner_descr),
	};

	struct idtag_info_inner parsed_inner = { 0 };
	struct idtag_info_root parsed = {
		.id_tag_info = parsed_inner,
	};

	ret = json_obj_parse(json, strlen(json), root_descr,
			     ARRAY_SIZE(root_descr), &parsed);
	if (ret < 0) {
		return ret;
	}

	status = parsed.id_tag_info.status;
	if (!status) {
		return -EINVAL;
	}

	switch (*status) {
	case 'A':
		idtag_info->auth_status = OCPP_AUTH_ACCEPTED;
		break;
	case 'B':
		idtag_info->auth_status = OCPP_AUTH_BLOCKED;
		break;
	case 'E':
		idtag_info->auth_status = OCPP_AUTH_EXPIRED;
		break;
	case 'I':
		idtag_info->auth_status = OCPP_AUTH_INVALID;
		break;
	case 'C':
		idtag_info->auth_status = OCPP_AUTH_CONCURRENT_TX;
		break;
	default:
		return -EINVAL;
	}

	if (parsed.id_tag_info.parent_id_tag) {
		strncpy(idtag_info->p_idtag, parsed.id_tag_info.parent_id_tag,
			sizeof(idtag_info->p_idtag));
	}

	if (parsed.id_tag_info.expiry_date) {
		strncpy(idtag_info->exptime, parsed.id_tag_info.expiry_date,
			sizeof(idtag_info->exptime));
	}

	return 0;
}

static int parse_heartbeat_msg(char *json, struct timeval *date)
{
	int ret = 0;

	struct json_obj_descr descr[] = {
		JSON_OBJ_DESCR_PRIM_NAMED(struct heartbeat_payload, "currentTime",
					  current_time, JSON_TOK_STRING),
	};

	struct heartbeat_payload heartbeat = {0};

	ret = json_obj_parse(json, strlen(json), descr,
			     ARRAY_SIZE(descr), &heartbeat);

	/* todo: convert civil time to epoch and update local time */

	if (ret < 0) {
		return ret;
	}

	return 0;
}

static int parse_authorize_msg(char *json, struct ocpp_idtag_info *idtag_info)
{
	int ret = 0;

	ret = parse_idtag_info(json, idtag_info);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

static int parse_bootnotification_msg(char *json, struct boot_notif *binfo)
{
	int ret = 0;

	struct json_obj_descr descr[] = {
		JSON_OBJ_DESCR_PRIM_NAMED(struct bootnotif_payload, "status",
					  status, JSON_TOK_STRING),
		JSON_OBJ_DESCR_PRIM_NAMED(struct bootnotif_payload, "interval",
					  interval, JSON_TOK_NUMBER),
		JSON_OBJ_DESCR_PRIM_NAMED(struct bootnotif_payload, "currentTime",
					  current_time, JSON_TOK_STRING),
	};

	struct bootnotif_payload msg = { 0 };

	ret = json_obj_parse(json, strlen(json), descr, ARRAY_SIZE(descr), &msg);
	if (ret < 0) {
		return ret;
	}

	if (!msg.status) {
		return -EINVAL;
	}

	switch (*msg.status) {
	case 'A':	/* accepted */
		binfo->status = BOOT_ACCEPTED;
		break;
	case 'P':	/* pending */
		binfo->status = BOOT_PENDING;
		break;
	case 'R':	/* rejected */
		binfo->status = BOOT_REJECTED;
		break;
	default:
		return -EINVAL;
	}

	if (!msg.interval) {
		return -EINVAL;
	}

	binfo->interval = msg.interval;

	if (!msg.current_time) {
		return -EINVAL;
	}

	/* todo: convert civil time to epoch and update local time */
	binfo->date;

	return 0;
}

static int parse_start_txn_msg(char *json,
			       int *idtxn,
			       struct ocpp_idtag_info *idtag_info)
{
	int ret = 0;

	struct json_obj_descr descr[] = {
		JSON_OBJ_DESCR_PRIM_NAMED(struct start_txn_payload, "transactionId",
					  transaction_id, JSON_TOK_NUMBER),
	};

	struct start_txn_payload payload = { 0 };

	ret = json_obj_parse(json, strlen(json), descr, ARRAY_SIZE(descr), &payload);
	if (ret < 0) {
		return ret;
	}

	*idtxn = payload.transaction_id;

	ret = parse_idtag_info(json, idtag_info);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

static int parse_getconfig_msg(char *json, char *key)
{
	int ret = 0;

	struct json_obj_descr descr[] = {
		JSON_OBJ_DESCR_ARRAY_NAMED(struct getconfig_payload, "key",
					   key, 1, key_len, JSON_TOK_STRING),
	};

	struct getconfig_payload payload = { 0 };

	ret = json_obj_parse(json, strlen(json), descr,
			     ARRAY_SIZE(descr), &payload);
	if (ret < 0) {
		return ret;
	}

	if (!payload.key_len) {
		return -EINVAL;
	}

	strcpy(key, payload.key[0]);

	return 0;
}

static int parse_changeconfig_msg(char *json, char *key, char *val)
{
	int ret = 0;

	struct json_obj_descr descr[] = {
		JSON_OBJ_DESCR_PRIM_NAMED(struct changeconfig_payload, "key",
					  key, JSON_TOK_STRING),
		JSON_OBJ_DESCR_PRIM_NAMED(struct changeconfig_payload, "value",
					  value, JSON_TOK_STRING),
	};

	struct changeconfig_payload payload = { 0 };

	ret = json_obj_parse(json, strlen(json), descr, ARRAY_SIZE(descr), &payload);
	if (ret < 0) {
		return ret;
	}

	if (!payload.key || !payload.value) {
		return -EINVAL;
	}

	strncpy(key, payload.key, CISTR50);
	strncpy(val, payload.value, CISTR500);

	return 0;
}

static int parse_remote_start_txn_msg(char *json,
				      int *idcon,
				      char *idtag)
{
	int ret = 0;

	struct json_obj_descr descr[] = {
		JSON_OBJ_DESCR_PRIM_NAMED(struct remote_start_txn_payload, "idTag",
					  id_tag, JSON_TOK_STRING),
		JSON_OBJ_DESCR_PRIM_NAMED(struct remote_start_txn_payload, "connectorId",
					  connector_id, JSON_TOK_NUMBER),
	};

	struct remote_start_txn_payload payload = { 0 };

	ret = json_obj_parse(json, strlen(json), descr, ARRAY_SIZE(descr), &payload);
	if (ret < 0) {
		return ret;
	}

	if (!payload.id_tag) {
		return -EINVAL;
	}

	strncpy(idtag, payload.id_tag, CISTR50);
	*idcon = payload.connector_id;

	return 0;
}

static int parse_remote_stop_txn_msg(char *json, int *idtxn)
{
	int ret = 0;

	struct json_obj_descr descr[] = {
		JSON_OBJ_DESCR_PRIM_NAMED(struct remote_stop_txn_payload, "transactionId",
					  transaction_id, JSON_TOK_NUMBER),
	};

	struct remote_stop_txn_payload payload = { 0 };

	ret = json_obj_parse(json, strlen(json), descr, ARRAY_SIZE(descr), &payload);
	if (ret < 0) {
		return ret;
	}

	*idtxn = payload.transaction_id;

	return 0;
}

static int parse_unlock_connectormsg(char *json, int *idcon)
{
	int ret = 0;

	struct json_obj_descr descr[] = {
	JSON_OBJ_DESCR_PRIM_NAMED(struct unlock_connector_payload, "connectorId",
				  connector_id, JSON_TOK_NUMBER),
	};

	struct unlock_connector_payload payload = { 0 };

	ret = json_obj_parse(json, strlen(json), descr, ARRAY_SIZE(descr), &payload);
	if (ret < 0) {
		return ret;
	}

	if (payload.connector_id == 0) {
		return -EINVAL;
	}

	*idcon = payload.connector_id;

	return 0;
}

static ocpp_msg_fp_t ocpp_json_parser[PDU_MSG_END] = {
	[PDU_BOOTNOTIFICATION]	= (ocpp_msg_fp_t)parse_bootnotification_msg,
	[PDU_AUTHORIZE]		= (ocpp_msg_fp_t)parse_authorize_msg,
	[PDU_START_TRANSACTION]	= (ocpp_msg_fp_t)parse_start_txn_msg,
	[PDU_STOP_TRANSACTION]	= (ocpp_msg_fp_t)parse_authorize_msg,
	[PDU_METER_VALUES]	= NULL,
	[PDU_HEARTBEAT]		= (ocpp_msg_fp_t)parse_heartbeat_msg,
	[PDU_GET_CONFIGURATION]	= (ocpp_msg_fp_t)parse_getconfig_msg,
	[PDU_CHANGE_CONFIGURATION]	= (ocpp_msg_fp_t)parse_changeconfig_msg,
	[PDU_REMOTE_START_TRANSACTION]	= (ocpp_msg_fp_t)parse_remote_start_txn_msg,
	[PDU_REMOTE_STOP_TRANSACTION]	= (ocpp_msg_fp_t)parse_remote_stop_txn_msg,
	[PDU_UNLOCK_CONNECTOR]	= (ocpp_msg_fp_t)parse_unlock_connectormsg,
};

static ocpp_msg_fp_t ocpp_json_frame[PDU_MSG_END] = {
	[PDU_BOOTNOTIFICATION]	=  (ocpp_msg_fp_t)frame_bootnotif_msg,
	[PDU_AUTHORIZE]		=  (ocpp_msg_fp_t)frame_authorize_msg,
	[PDU_START_TRANSACTION]	=  (ocpp_msg_fp_t)frame_start_txn_msg,
	[PDU_STOP_TRANSACTION]	=  (ocpp_msg_fp_t)frame_stop_txn_msg,
	[PDU_METER_VALUES]	=  (ocpp_msg_fp_t)frame_meter_val_msg,
	[PDU_HEARTBEAT]		=  (ocpp_msg_fp_t)frame_heartbeat_msg,
	[PDU_GET_CONFIGURATION]	=  (ocpp_msg_fp_t)frame_getconfig_msg,
	[PDU_CHANGE_CONFIGURATION] (ocpp_msg_fp_t)frame_status_resp_msg,
	[PDU_REMOTE_START_TRANSACTION]	= (ocpp_msg_fp_t)frame_status_resp_msg,
	[PDU_REMOTE_STOP_TRANSACTION]	= (ocpp_msg_fp_t)frame_status_resp_msg,
	[PDU_UNLOCK_CONNECTOR]	= (ocpp_msg_fp_t)frame_status_resp_msg,
};

void ocpp_parser_init(ocpp_msg_fp_t **cfn, ocpp_msg_fp_t **pfn)
{
	*pfn = ocpp_json_parser;
	*cfn = ocpp_json_frame;
}
