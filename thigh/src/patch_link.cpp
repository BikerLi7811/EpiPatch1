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

Callbacks cbs;
std::atomic<bool> notify_on{false};
std::atomic<bool> encrypted{false};

bt_addr_le_t bond_addr;
bool have_bond;

// ---- GATT service ---------------------------------------------------------------

ssize_t on_write(bt_conn *, const bt_gatt_attr *, const void *buf, uint16_t len, uint16_t offset, uint8_t)
{
	if (offset != 0 || len < 1) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}
	switch (static_cast<msg::Type>(*static_cast<const uint8_t *>(buf))) {
	case msg::Type::Decision:
		if (len == sizeof(msg::Decision) && cbs.decision) {
			msg::Decision d;
			std::memcpy(&d, buf, sizeof(d));
			cbs.decision(d);
		}
		break;
	case msg::Type::Cancel:
		if (len == sizeof(msg::Cancel) && cbs.cancel) {
			cbs.cancel();
		}
		break;
	case msg::Type::ReportNow:
		if (cbs.report_now) {
			cbs.report_now();
		}
		break;
	default:
		break;
	}
	return len;
}

void ccc_changed(const bt_gatt_attr *, uint16_t value) { notify_on = value == BT_GATT_CCC_NOTIFY; }

// attrs: [0] service, [1]/[2] to-thigh decl/value, [3]/[4] to-chest decl/value, [5] CCC
BT_GATT_SERVICE_DEFINE(epipatch_svc, BT_GATT_PRIMARY_SERVICE(EPIPATCH_UUID_SVC),
		       BT_GATT_CHARACTERISTIC(EPIPATCH_UUID_TO_THIGH, BT_GATT_CHRC_WRITE_WITHOUT_RESP,
					      BT_GATT_PERM_WRITE_ENCRYPT, nullptr, on_write, nullptr),
		       BT_GATT_CHARACTERISTIC(EPIPATCH_UUID_TO_CHEST, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE,
					      nullptr, nullptr, nullptr),
		       BT_GATT_CCC(ccc_changed, BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT), );

// ---- bonds: only the paired chest patch -----------------------------------------

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
		LOG_WRN("pairing refused: already paired to a chest patch");
		return BT_SECURITY_ERR_PAIR_NOT_ALLOWED;
	}
	return BT_SECURITY_ERR_SUCCESS;
}

void pairing_complete(bt_conn *, bool bonded)
{
	LOG_INF("paired with the chest patch (bonded %d)", bonded);
	load_bond();
}

void pairing_failed(bt_conn *, bt_security_err reason) { LOG_WRN("pairing failed (%d)", reason); }

bt_conn_auth_cb auth_cb = {.pairing_accept = pairing_accept};
bt_conn_auth_info_cb auth_info_cb = {.pairing_complete = pairing_complete, .pairing_failed = pairing_failed};

// ---- advertising ------------------------------------------------------------------

const bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, EPIPATCH_UUID_SVC_VAL),
};
const bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

void start_advertising()
{
	load_bond();
	uint32_t options = BT_LE_ADV_OPT_CONN;
	if (have_bond) {
		// once paired, only the chest patch may connect
		bt_le_filter_accept_list_clear();
		bt_le_filter_accept_list_add(&bond_addr);
		options |= BT_LE_ADV_OPT_FILTER_CONN;
	}
	const bt_le_adv_param p =
		BT_LE_ADV_PARAM_INIT(options, BT_GAP_ADV_SLOW_INT_MIN, BT_GAP_ADV_SLOW_INT_MAX, nullptr);
	const int err = bt_le_adv_start(&p, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err && err != -EALREADY) {
		LOG_ERR("advertising failed (%d)", err);
	}
}

// ---- connection events ---------------------------------------------------------

void connected(bt_conn *c, uint8_t err)
{
	if (err) {
		return;
	}
	// nothing is accepted until the link is encrypted (characteristic permissions)
	bt_conn_set_security(c, BT_SECURITY_L2);
}

void disconnected(bt_conn *, uint8_t reason)
{
	encrypted = false;
	notify_on = false;
	LOG_INF("chest patch disconnected (0x%02x)", reason);
}

void recycled() { start_advertising(); }

void security_changed(bt_conn *c, bt_security_t level, bt_security_err err)
{
	if (err || level < BT_SECURITY_L2) {
		LOG_WRN("link not encrypted (%d), dropping", err);
		bt_conn_disconnect(c, BT_HCI_ERR_AUTH_FAIL);
		return;
	}
	encrypted = true;
	LOG_INF("chest patch linked");
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
	.recycled = recycled,
	.security_changed = security_changed,
};

} // namespace

int start(const Callbacks &cb)
{
	cbs = cb;
	bt_conn_auth_cb_register(&auth_cb);
	bt_conn_auth_info_cb_register(&auth_info_cb);
	int err = bt_enable(nullptr);
	if (err) {
		return err;
	}
	settings_load_subtree("bt");
	start_advertising();
	LOG_INF("Bluetooth up; %s", have_bond ? "waiting for the paired chest patch" : "unpaired: open for pairing");
	return 0;
}

bool ready() { return encrypted && notify_on; }

void send_report(const msg::Report &r)
{
	if (ready()) {
		bt_gatt_notify(nullptr, &epipatch_svc.attrs[4], &r, sizeof(r));
	}
}

} // namespace dev::link
