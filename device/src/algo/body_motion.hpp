// Posture, collapse and fall detection from the chest and thigh IMUs.
//
// Accelerometer streams (12.5 Hz, from each LSM6DSO FIFO) give gravity
// direction and activity; the IMUs' own wake-up (impact) and free-fall
// interrupts give the moment of a fall.
#pragma once

#include <cstdint>
#include <span>

#include "algo/vitals.hpp"

namespace dev::algo {

struct Vec3 {
	float x, y, z;
};

// Which sensor axis points toward the head when the wearer stands.
enum class Axis : uint8_t { PosX, NegX, PosY, NegY, PosZ, NegZ };

class BodyMotion {
public:
	struct Config {
		Axis chest_up = Axis::PosX;
		Axis thigh_up = Axis::PosX;
		float upright_deg = 35.0f;            // tilt below: upright
		float horizontal_deg = 60.0f;         // tilt above: horizontal
		uint32_t collapse_transition_ms = 800;  // 35 deg -> 60 deg trunk tilt faster than this
		uint32_t collapse_still_ms = 15000;     // ...then motionless this long
		uint32_t fall_window_ms = 5000;         // impact <-> lying within this
		uint32_t fall_confirm_ms = 10000;       // still lying this long after impact
		uint32_t recover_ms = 5000;             // upright this long clears the flags
		float still_g = 0.05f;
	};

	void reset(const Config &cfg);

	// Samples in g; t_last_ms is the time of the last sample, dt_ms the spacing.
	void chest(std::span<const Vec3> s, uint32_t t_last_ms, uint32_t dt_ms);
	void thigh(std::span<const Vec3> s, uint32_t t_last_ms, uint32_t dt_ms);
	void impact(uint32_t t_ms);   // wake-up interrupt above the impact threshold
	void freefall(uint32_t t_ms); // free-fall interrupt

	// once per second
	void update(uint32_t now_ms);

	Posture posture() const { return posture_; }
	float activity_g() const;
	bool collapsed() const { return collapsed_; }
	bool fall() const { return fall_; }
	float chest_tilt_deg() const { return chest_.tilt; }

private:
	struct Sensor {
		Vec3 g{};          // low-passed gravity
		float tilt{};      // deg between "up" axis and vertical
		float activity{};  // smoothed |a - g| [g]
		bool seen{};
		uint32_t last_ms{};
	};

	void feed(Sensor &s, Axis up, std::span<const Vec3> v, uint32_t t_last_ms, uint32_t dt_ms,
		  bool is_chest);
	void chest_tilt_at(uint32_t t);

	Config cfg_{};
	Sensor chest_{}, thigh_{};
	bool was_horizontal_{};
	uint32_t last_upright_ms_{};
	uint32_t lying_since_ms_{};
	bool fast_drop_{};
	uint32_t last_active_ms_{};
	uint32_t trigger_ms_{};   // last impact / free-fall
	bool have_trigger_{};
	uint32_t upright_since_ms_{};
	Posture posture_{};
	bool collapsed_{}, fall_{};
};

} // namespace dev::algo
