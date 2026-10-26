/*
 * On-device wheeze detector for nRF54L15 (PDM MEMS mic -> log-mel -> int8 CNN).
 *
 * Public C API. Enable with CONFIG_WHEEZE_DETECTOR=y; the detector owns the PDM
 * microphone (DT alias "wheeze-mic", or "dmic0") and runs in its own thread.
 * Everything else (BLE, storage, UI) stays in your application.
 */
#ifndef WHEEZE_WHEEZE_H_
#define WHEEZE_WHEEZE_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum wheeze_event_type {
	WHEEZE_EVT_START, /* wheezing began (smoothed P(wheeze) crossed the on-threshold) */
	WHEEZE_EVT_END,   /* a wheeze event finished and was counted (>= min duration) */
};

struct wheeze_event {
	enum wheeze_event_type type;
	uint32_t timestamp_ms; /* k_uptime_get_32() when detected */
	uint32_t count;        /* total counted events since wheeze_start() */
	uint32_t duration_ms;  /* WHEEZE_EVT_END only: length of this event */
	float probability;     /* smoothed P(wheeze) at the time of the event */
};

/*
 * Called from the detector thread. Keep it short (queue work / set a flag /
 * notify over BLE); audio blocks keep arriving while it runs.
 */
typedef void (*wheeze_callback_t)(const struct wheeze_event *evt, void *user_data);

struct wheeze_stats {
	uint32_t events;          /* counted wheeze events */
	uint32_t wheeze_ms_total; /* summed duration of counted events */
	bool wheezing;            /* currently inside a wheeze */
	bool mic_on;              /* microphone currently powered */
	bool continuous;          /* duty-cycler in continuous (vs sampling) mode */
	uint32_t mic_rate_hz;     /* PCM rate in use (8000 low-power, 16000 fallback); 0 if off */
};

/*
 * Start detection. Returns 0, -EALREADY if running, -ENODEV if the PDM device
 * is not ready. cb may be NULL (poll wheeze_get_stats() instead).
 */
int wheeze_start(wheeze_callback_t cb, void *user_data);

/* Stop detection and power the microphone down. Blocks until the thread exits. */
int wheeze_stop(void);

/*
 * Keep the microphone on (continuous listening) for the next ms milliseconds,
 * waking it from a duty-cycle sleep if needed. For applications that schedule
 * their own sensor reads. Returns -ENODEV if the detector is not running.
 */
int wheeze_listen(uint32_t ms);

/* Snapshot of the counters. Safe to call from any thread. */
int wheeze_get_stats(struct wheeze_stats *out);

#ifdef __cplusplus
}
#endif

#endif /* WHEEZE_WHEEZE_H_ */
