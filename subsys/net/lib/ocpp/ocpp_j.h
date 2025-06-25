/*
 * Copyright (c) 2023 Linumiz
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __OCPP_J_
#define __OCPP_J_

#define STRIP_PATTERN_LEN  48
/* CALC:
 * KEY (WITH QUOTES) = 41
    "ChargingScheduleAllowedChargingRateUnit" (41 chars)
 * PATTERN OVERHEAD = 7
 * TOTAL = 48
 */

#define MAX_UID_LEN 38
/* AS PER OCPP 1.6J FORMAT : MAX LEN FOR UID = 36 + 2 (FOR QUOTES) */

#define MAX_AUTH_REQ_LEN 32
/* CALC:
 * KEY (idTag WITH QUOTES) = 7
 * VALUE (MAX WITH QUOTES) = 22
 * COLON = 1
 * BRACES = 2
 * TOTAL = 32
 */

#define MAX_HEARTBEAT_REQ_LEN 2
/* CALC:
 * BRACES = 2
 */

#define MAX_BOOTNOTIF_REQ_LEN 412
/* CALC:
 * KEY (WITH QUOTES) = 145
	"chargePointModel": 18
	"chargePointVendor": 19
	"chargeBoxSerialNumber": 23
	"chargePointSerialNumber": 25
	"firmwareVersion": 17
	"iccid": 7
	"imsi": 6
	"meterSerialNumber": 19
	"meterType": 11
 * VALUE (MAX WITH QUOTES) = 248
	"chargePointModel": 22
	"chargePointVendor": 22
	"chargeBoxSerialNumber": 27
	"chargePointSerialNumber": 27
	"firmwareVersion": 52
	"iccid": 22
	"imsi": 22
	"meterSerialNumber": 27
	"meterType": 27
 * COLON = 9
 * BRACES = 2
 * COMMAS = 8
 * TOTAL = 412
 */

#define MAX_METERVALUE_REQ_LEN 302
/* CALC:
 * KEY (WITH QUOTES) = 131
    "connectorId": 14
    "transactionId": 17
    "meterValue": 12
    "timestamp": 11
    "sampledValue": 14
    "value": 7
    "context": 10
    "format": 9
    "measurand": 12
    "phase": 8
    "location": 11
    "unit": 6
 * VALUE (MAX WITH QUOTES) = 137
    "connectorId": 3
    "transactionId": 7
    "timestamp": 21
    "value": 13
    "context": 18
    "format": 13
    "measurand": 35
    "phase": 7
    "location": 8
    "unit": 12
 * COLON = 11
 * COMMAS = 12
 * BRACES = 10
 * TOTAL = 301 ~302
 */

#define MAX_STOPTRANSACTION_REQ_LEN 140
/* CALC:
 * KEY (WITH QUOTES) = 53
    "idTag": 7
    "meterStop": 11
    "timestamp": 11
    "transactionId": 16
    "reason": 8
 * VALUE (MAX WITH QUOTES) = 75
    "idTag": 22
    "meterStop": 10
    "timestamp": 21
    "transactionId": 7
    "reason": 15
 * COLON = 5
 * COMMAS = 4
 * BRACES = 2
 * TOTAL = 139 ~140
 */

#define MAX_STARTTRANSACTION_REQ_LEN 186
/* CALC:
 * KEY (WITH QUOTES) = 68
   "connectorId": 13
   "idTag": 7
   "meterStart": 12
   "reservationId": 16
   "timestamp": 11
 * VALUE (MAX WITH QUOTES) = 92
   "connectorId": 3
   "idTag": 22
   "meterStart": 10
   "reservationId": 10
   "timestamp": 21
 * COLON = 5
 * COMMAS = 4
 * BRACES = 2
 * TOTAL = 186
 */

#define MAX_UNLOCKCONNECTOR_RESP_LEN 24
/* CALC:
 * KEY (WITH QUOTES) = 8
    "status": 8
 * VALUE (MAX WITH QUOTES) = 13
    "status": 13 ("UnlockFailed")
 * COLON = 1
 * COMMAS = 0
 * BRACES = 2
 * TOTAL = 24
 */

#define MAX_GETCONFIGURATION_RESP_LEN 604
/* CALC:
 * KEY (WITH QUOTES) = 30
    "configurationKey": 18
    "key": 5
    "readonly": 10
    "value": 7
 * VALUE (MAX WITH QUOTES) = 559
    "key": 52
    "readonly": 5
    "value": 502
 * COLONS: 4
 * COMMAS: 2
 * BRACES: 8
 * TOTAL = 30 + 559 + 4 + 2 + 8 = 603 ~604
 */

#define MAX_OCPP_RPC_MESSAGE_LEN 670
/* CALC:
 * STRUCTURE: [MessageTypeId, "UniqueId", "Action", {Payload}]
 * COMPONENTS:
   - MessageTypeId: 1
   - UniqueId: 36
   - Action: 22
   - Payload: 604
   - Commas: 3
   - Square brackets: 2
   - Curly braces: 2
 * TOTAL = 1 + 36 + 22 + 604 + 3 + 2 + 2 = 670
 */

struct authorize_payload {
	char *id_tag;
};

struct ocpp_heartbeat_msg {
	bool dummy;
};

struct heartbeat_payload {
	char *current_time;
};

struct ocpp_bootnotif_msg {
	char *charge_point_model;
	char *charge_point_vendor;
	char *charge_box_serial_number;
	char *charge_point_serial_number;
	char *firmware_version;
	char *iccid;
	char *imsi;
	char *meter_serial_number;
	char *meter_type;
};

struct ocpp_sampled_value {
	char *measurand;
	char *value;
	char *unit;
};

struct ocpp_meter_value {
	char *timestamp;
	struct ocpp_sampled_value sampled_value[1];
	size_t sampled_value_len;
};

struct ocpp_meter_val_msg {
	int connector_id;
	int transaction_id;
	struct ocpp_meter_value meter_value[1];
	size_t meter_value_len;
};

struct ocpp_stop_txn_msg {
	int transaction_id;
	int meter_stop;
	char *timestamp;
	char *reason;
	char *id_tag;
};

struct ocpp_start_txn_msg {
	int connector_id;
	char *id_tag;
	int meter_start;
	char *timestamp;
	int reservation_id;
};

struct ocpp_key_val {
	char *key;
	int readonly;
	char *value;
};

struct ocpp_getconfig_msg {
	struct ocpp_key_val configuration_key[1];
	size_t configuration_key_len;
	char *unknown_key;
};

struct ocpp_status_resp_msg {
	char *status;
};

struct idtag_info_inner {
	char *status;
	char *parent_id_tag;
	char *expiry_date;
};

struct idtag_info_root {
	struct idtag_info_inner id_tag_info;
};

struct bootnotif_payload {
	char *status;
	int interval;
	char *current_time;
};

struct start_txn_payload {
	int transaction_id;
};

struct getconfig_payload {
	char *key[1];
	size_t key_len;
};

struct changeconfig_payload {
	char *key;
	char *value;
};

struct remote_start_txn_payload {
	char *id_tag;
	int connector_id;
};

struct remote_stop_txn_payload {
	int transaction_id;
};

struct unlock_connector_payload {
	int connector_id;
};

enum strip_mode {
	STRIP_STRING,
	STRIP_INT,
	STRIP_ARRAY
};

#endif /* __OCPP_J_ */
