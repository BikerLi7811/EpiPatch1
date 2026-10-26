// Keeps the number of deployments in RRAM (Zephyr settings over ZMS), so a
// reboot after firing does not re-arm a spent mechanism.
#include "persist.hpp"

#include <cstring>

#include <zephyr/kernel.h>
#include <zephyr/settings/settings.h>

namespace dev::persist {

#if IS_ENABLED(CONFIG_APP_PERSIST_DEPLOYMENTS)

namespace {

uint8_t g_deployments;

int set(const char *key, size_t len, settings_read_cb read_cb, void *cb_arg)
{
	if (std::strcmp(key, "deployments") == 0 && len == sizeof(g_deployments)) {
		const ssize_t n = read_cb(cb_arg, &g_deployments, sizeof(g_deployments));
		return n < 0 ? static_cast<int>(n) : 0;
	}
	return -ENOENT;
}

char g_name[] = "dev"; // non-const: older Zephyr declares settings_handler::name as char *
settings_handler g_handler;

} // namespace

int load(uint8_t &deployments)
{
	int err = settings_subsys_init();
	if (!err) {
		g_handler.name = g_name;
		g_handler.h_set = set;
		err = settings_register(&g_handler);
	}
	if (!err) {
		err = settings_load_subtree("dev");
	}
	deployments = g_deployments;
	return err;
}

int save(uint8_t deployments)
{
	g_deployments = deployments;
	return settings_save_one("dev/deployments", &g_deployments, sizeof(g_deployments));
}

#else

int load(uint8_t &deployments)
{
	deployments = 0;
	return 0;
}

int save(uint8_t) { return 0; }

#endif

} // namespace dev::persist
