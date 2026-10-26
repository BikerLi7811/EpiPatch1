/*
 * Minimal integration of the wheeze detector: start it, react to events.
 * Replace the callback body with your own output (BLE notify, store, ...).
 */
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <wheeze/wheeze.h>

LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

static void on_wheeze(const struct wheeze_event *evt, void *user_data)
{
	ARG_UNUSED(user_data);

	if (evt->type == WHEEZE_EVT_END) {
		LOG_INF("wheeze #%u: %u ms (p=%d%%)", evt->count, evt->duration_ms,
			(int)(evt->probability * 100));
	}
}

int main(void)
{
	int err = wheeze_start(on_wheeze, NULL);

	if (err) {
		LOG_ERR("wheeze_start failed: %d", err);
	}
	return 0; /* the detector keeps running in its own thread */
}
