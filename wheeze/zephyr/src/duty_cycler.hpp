/*
 * Adaptive microphone duty-cycling - the biggest battery lever.
 *
 * The PDM mic + PDM20 + its clock draw current continuously while listening,
 * far more than the duty-cycled CPU. Wheezing in asthma/COPD persists over many
 * breaths, so while nothing is happening we only *sample*: listen for
 * listen_ms out of every listen_ms + sleep_ms. The moment a wheeze candidate
 * shows up we switch to continuous listening and stay there until hold_ms pass
 * with no candidate.
 *
 * Trade-off: in sampling mode an episode is noticed up to sleep_ms late, and an
 * isolated single wheeze during a sleep gap is missed. Set enabled = false for
 * gap-free monitoring.
 */
#pragma once

#include <cstdint>

namespace wheeze {

class DutyCycler {
public:
	struct Config {
		bool enabled = true;
		uint32_t listen_ms = 8000;  /* ~2-3 breaths per look (incl. ~0.8 s warm-up) */
		uint32_t sleep_ms = 22000;  /* mic powered down; 27 % listening duty */
		uint32_t hold_ms = 60000;   /* stay continuous this long after last candidate */
	};

	DutyCycler() = default;
	explicit DutyCycler(const Config &cfg) : cfg_(cfg) {}

	/* Mic just (re)started. */
	void on_listen_start(uint32_t now_ms) { listen_start_ = now_ms; }

	/* A wheeze candidate (smoothed p above the release threshold, or an
	 * ongoing wheeze) was seen: listen continuously. */
	void on_candidate(uint32_t now_ms)
	{
		last_candidate_ = now_ms;
		have_candidate_ = true;
	}

	/* The application asked for the mic until until_ms (wheeze_listen()). */
	void force_until(uint32_t until_ms)
	{
		force_until_ = until_ms;
		have_force_ = true;
	}

	bool continuous(uint32_t now_ms) const
	{
		return !cfg_.enabled || (have_candidate_ && now_ms - last_candidate_ < cfg_.hold_ms) ||
		       (have_force_ && static_cast<int32_t>(force_until_ - now_ms) > 0);
	}

	/* Should the mic be powered down now? */
	bool should_stop(uint32_t now_ms) const
	{
		return !continuous(now_ms) && now_ms - listen_start_ >= cfg_.listen_ms;
	}

	uint32_t sleep_ms() const { return cfg_.sleep_ms; }

private:
	Config cfg_{};
	uint32_t listen_start_ = 0;
	uint32_t last_candidate_ = 0;
	bool have_candidate_ = false;
	uint32_t force_until_ = 0;
	bool have_force_ = false;
};

} // namespace wheeze
