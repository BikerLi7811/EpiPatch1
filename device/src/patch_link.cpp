#include "patch_link.hpp"

#include <atomic>
#include <cstring>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>

#include "patch_ble.hpp"

LOG_MODULE_REGISTER(link, CONFIG_APP_LOG_LEVEL);

namespace dev::link {
namespace {

namespace msg = dev::patch_msg;

const bt_uuid_128 kSvc = BT_UUID_INIT_128(EPIPATCH_UUID_SVC_VAL);
const bt_uuid_128 kToThigh = BT_UUID_INIT_128(EPIPATCH_UUID_TO_THIGH_VAL);
const bt_uuid_128 kToChest = BT_UUID_INIT_128(EPIPATCH_UUID_TO_CHEST_VAL);

ReportFn report_fn;
k_spinlock lock;
bt_conn *conn;              // under lock
uint16_t to_thigh_handle;   // under lock, 0 = not discovered
std::atomic<bool> subscribed{false};
uint32_t seq;

bt_addr_le_t bond_addr;
bool have_bond;

bt_gatt_discover_params disc;
bt_gatt_discover_params ccc_disc;
bt_gatt_subscribe_params sub;
uint16_t svc_end;

void start_scan();

// ---- bonds: we talk only to the patch we were paired with ----------------------

void bond_cb(const bt_bond_info *info, void *)
{
	bond_addr = info->addr;
	have_bond = true;
}

void load_bond()
{
	have_bond = false;
	bt_foreach_bond(BT_ID_DEFAULT, bond_cb, nullptr);
}

bt_security_err pairing_accept(bt_conn *, const bt_conn_pairing_feat *)
{
	load_bond();
	if (have_bond) {
		LOG_WRN("pairing refused: already paired to a thigh patch");
		return BT_SECURITY_ERR_PAIR_NOT_ALLOWED;
	}
	return BT_SECURITY_ERR_SUCCESS;
}

void pairing_complete(bt_conn *, bool bonded)
{
	LOG_INF("paired with the thigh patch (bonded %d)", bonded);
	load_bond();
}

void pairing_failed(bt_conn *, bt_security_err reason) { LOG_WRN("pairing failed (%d)", reason); }

bt_conn_auth_cb auth_cb = {.pairing_accept = pairing_accept};
bt_conn_auth_info_cb auth_info_cb = {.pairing_complete = pairing_complete, .pairing_failed = pairing_failed};

// ---- scanning -------------------------------------------------------------------

bool ad_has_service(bt_data *data, void *user)
{
	auto *found = static_cast<bool *>(user);
	if (data->type == BT_DATA_UUID128_ALL || data->type == BT_DATA_UUID128_SOME) {
		for (uint8_t i = 0; i + 16 <= data->data_len; i += 16) {
			if (std::memcmp(&data->data[i], kSvc.val, 16) == 0) {
				*found = true;
				return false;
			}
		}
	}
	return true;
}

void device_found(const bt_addr_le_t *addr, int8_t, uint8_t type, net_buf_simple *ad)
{
	if (type != BT_GAP_ADV_TYPE_ADV_IND && type != BT_GAP_ADV_TYPE_ADV_DIRECT_IND) {
		return;
	}
	if (have_bond) {
		if (!bt_addr_le_eq(addr, &bond_addr)) {
			return;
		}
	} else {
		bool found = false;
		bt_data_parse(ad, ad_has_service, &found);
		if (!found) {
			return;
		}
	}
	if (bt_le_scan_stop()) {
		return;
	}
	bt_conn *c = nullptr;
	if (bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN, BT_LE_CONN_PARAM_DEFAULT, &c)) {
		start_scan();
		return;
	}
	const k_spinlock_key_t key = k_spin_lock(&lock);
	conn = c;
	k_spin_unlock(&lock, key);
}

void start_scan()
{
	const bt_le_scan_param p = {
		.type = BT_LE_SCAN_TYPE_PASSIVE,
		.options = BT_LE_SCAN_OPT_FILTER_DUPLICATE,
		.interval = BT_GAP_SCAN_SLOW_INTERVAL_1,
		.window = BT_GAP_SCAN_SLOW_WINDOW_1,
	};
	const int err = bt_le_scan_start(&p, device_found);
	if (err && err != -EALREADY) {
		LOG_ERR("scan start failed (%d)", err);
	}
}

// ---- discovery + subscription ----------------------------------------------------

uint8_t on_notify(bt_conn *, bt_gatt_subscribe_params *, const void *data, uint16_t len)
{
	if (!data) {
		subscribed = false;
		return BT_GATT_ITER_STOP;
	}
	if (len == sizeof(msg::Report) && *static_cast<const uint8_t *>(data) ==
						    static_cast<uint8_t>(msg::Type::Report)) {
		msg::Report r;
		std::memcpy(&r, data, sizeof(r));
		if (report_fn) {
			report_fn(r);
		}
	}
	return BT_GATT_ITER_CONTINUE;
}

uint8_t on_discover(bt_conn *c, const bt_gatt_attr *attr, bt_gatt_discover_params *params)
{
	if (!attr) {
		// end of this discovery step
		if (params->type == BT_GATT_DISCOVER_CHARACTERISTIC && sub.value_handle) {
			sub.notify = on_notify;
			sub.value = BT_GATT_CCC_NOTIFY;
			sub.ccc_handle = 0; // found automatically
			sub.end_handle = svc_end;
			sub.disc_params = &ccc_disc;
			const int err = bt_gatt_subscribe(c, &sub);
			subscribed = err == 0 || err == -EALREADY;
			LOG_INF("thigh patch %s", subscribed ? "linked" : "subscribe failed");
		}
		return BT_GATT_ITER_STOP;
	}
	if (params->type == BT_GATT_DISCOVER_PRIMARY) {
		const auto *svc = static_cast<const bt_gatt_service_val *>(attr->user_data);
		svc_end = svc->end_handle;
		params->uuid = nullptr;
		params->start_handle = attr->handle + 1;
		params->end_handle = svc_end;
		params->type = BT_GATT_DISCOVER_CHARACTERISTIC;
		bt_gatt_discover(c, params);
		return BT_GATT_ITER_STOP;
	}
	const auto *chrc = static_cast<const bt_gatt_chrc *>(attr->user_data);
	if (bt_uuid_cmp(chrc->uuid, &kToThigh.uuid) == 0) {
		const k_spinlock_key_t key = k_spin_lock(&lock);
		to_thigh_handle = chrc->value_handle;
		k_spin_unlock(&lock, key);
	} else if (bt_uuid_cmp(chrc->uuid, &kToChest.uuid) == 0) {
		sub.value_handle = chrc->value_handle;
	}
	return BT_GATT_ITER_CONTINUE;
}

void start_discovery(bt_conn *c)
{
	sub = {};
	disc = {};
	disc.uuid = &kSvc.uuid;
	disc.func = on_discover;
	disc.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
	disc.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
	disc.type = BT_GATT_DISCOVER_PRIMARY;
	const int err = bt_gatt_discover(c, &disc);
	if (err) {
		LOG_ERR("discovery failed (%d)", err);
	}
}

// ---- connection events ---------------------------------------------------------

void connected(bt_conn *c, uint8_t err)
{
	if (err) {
		const k_spinlock_key_t key = k_spin_lock(&lock);
		if (conn == c) {
			bt_conn_unref(conn);
			conn = nullptr;
		}
		k_spin_unlock(&lock, key);
		start_scan();
		return;
	}
	// nothing is exchanged until the link is encrypted
	bt_conn_set_security(c, BT_SECURITY_L2);
}

void disconnected(bt_conn *c, uint8_t reason)
{
	const k_spinlock_key_t key = k_spin_lock(&lock);
	if (conn == c) {
		bt_conn_unref(conn);
		conn = nullptr;
	}
	to_thigh_handle = 0;
	k_spin_unlock(&lock, key);
	subscribed = false;
	LOG_INF("thigh patch disconnected (0x%02x)", reason);
	start_scan();
}

void security_changed(bt_conn *c, bt_security_t level, bt_security_err err)
{
	if (err || level < BT_SECURITY_L2) {
		LOG_WRN("link not encrypted (%d), dropping", err);
		bt_conn_disconnect(c, BT_HCI_ERR_AUTH_FAIL);
		return;
	}
	start_discovery(c);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
	.security_changed = security_changed,
};

template <typename T> void send(const T &m)
{
	const k_spinlock_key_t key = k_spin_lock(&lock);
	bt_conn *c = conn ? bt_conn_ref(conn) : nullptr;
	const uint16_t h = to_thigh_handle;
	k_spin_unlock(&lock, key);
	if (!c) {
		return;
	}
	if (h && subscribed) {
		bt_gatt_write_without_response(c, h, &m, sizeof(m), false);
	}
	bt_conn_unref(c);
}

} // namespace

int start(ReportFn on_report)
{
	report_fn = on_report;
	bt_conn_auth_cb_register(&auth_cb);
	bt_conn_auth_info_cb_register(&auth_info_cb);
	int err = bt_enable(nullptr);
	if (err) {
		return err;
	}
	settings_load_subtree("bt");
	load_bond();
	LOG_INF("Bluetooth up; %s", have_bond ? "looking for the paired thigh patch" : "unpaired: pairing with the first thigh patch found");
	start_scan();
	return 0;
}

bool ready() { return subscribed && to_thigh_handle != 0; }

void send_decision(const algo::Assessment &a)
{
	send(msg::Decision{
		.type = msg::Type::Decision,
		.trigger = a.trigger,
		.severe = a.severe,
		.reserved = 0,
		.findings = a.findings,
		.reserved2 = 0,
		.seq = ++seq,
	});
}

void send_cancel() { send(msg::Cancel{.type = msg::Type::Cancel, .reserved = {}, .seq = ++seq}); }

void request_report() { send(msg::ReportNow{.type = msg::Type::ReportNow, .reserved = {}}); }

} // namespace dev::link
