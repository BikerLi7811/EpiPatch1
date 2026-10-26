// Deployment state machine: alert countdown, triple-tap cancel, motor firing.
//
//   Monitoring --trigger--> Alert --countdown elapsed--> Deploying --motor done--> Deployed
//                             |                                       \--motor fault--> Fault
//                             +--triple tap--> Cancelled --cooldown--> Monitoring
//   Cancelled --severe--> Alert   (a cancel does not silence a collapse with breathing trouble)
//   Deployed --still triggered after redose_ms, doses left--> Alert
//
// The controller only decides; the caller drives the alert and the motor
// from the returned Action and reports back with motor_done().
#pragma once

#include <cstdint>

#include "algo/anaphylaxis.hpp"

namespace dev::algo {

enum class DeployState : uint8_t { Monitoring, Alert, Deploying, Deployed, Cancelled, Fault };

class DeployController {
public:
	enum class Action : uint8_t { None, StartAlert, Cancelled, Fire, Deployed, Fault, Resume };

	struct Config {
		uint32_t countdown_ms = 20000;
		uint32_t cooldown_ms = 300000;
		uint8_t max_deployments = 1;
		uint32_t redose_ms = 600000;
	};

	void reset(const Config &cfg, uint8_t deployments_done = 0)
	{
		cfg_ = cfg;
		done_ = deployments_done;
		state_ = done_ > 0 ? DeployState::Deployed : DeployState::Monitoring;
		t0_ = 0;
		last_deploy_ms_ = 0;
		redose_timer_ = false; // after a reboot we don't know when it fired
	}

	Action update(const Assessment &a, uint32_t now_ms)
	{
		switch (state_) {
		case DeployState::Monitoring:
			if (a.trigger) {
				return start_alert(now_ms);
			}
			break;
		case DeployState::Cancelled:
			if (a.trigger && a.severe) {
				return start_alert(now_ms);
			}
			if (now_ms - t0_ >= cfg_.cooldown_ms) {
				state_ = DeployState::Monitoring;
				return Action::Resume;
			}
			break;
		case DeployState::Alert:
			if (now_ms - t0_ >= cfg_.countdown_ms) {
				state_ = DeployState::Deploying;
				t0_ = now_ms;
				return Action::Fire;
			}
			break;
		case DeployState::Deployed:
			if (a.trigger && done_ < cfg_.max_deployments && redose_timer_ &&
			    now_ms - last_deploy_ms_ >= cfg_.redose_ms) {
				return start_alert(now_ms);
			}
			break;
		case DeployState::Deploying:
		case DeployState::Fault:
			break;
		}
		return Action::None;
	}

	// The N-tap gesture. Only cancels a running countdown.
	Action cancel(uint32_t now_ms)
	{
		if (state_ != DeployState::Alert) {
			return Action::None;
		}
		state_ = DeployState::Cancelled;
		t0_ = now_ms;
		return Action::Cancelled;
	}

	Action motor_done(bool ok, uint32_t now_ms)
	{
		if (state_ != DeployState::Deploying) {
			return Action::None;
		}
		++done_;
		last_deploy_ms_ = now_ms;
		redose_timer_ = true;
		state_ = ok ? DeployState::Deployed : DeployState::Fault;
		return ok ? Action::Deployed : Action::Fault;
	}

	DeployState state() const { return state_; }
	uint8_t deployments() const { return done_; }
	uint32_t countdown_left_ms(uint32_t now_ms) const
	{
		if (state_ != DeployState::Alert) {
			return 0;
		}
		const uint32_t el = now_ms - t0_;
		return el >= cfg_.countdown_ms ? 0 : cfg_.countdown_ms - el;
	}

private:
	Action start_alert(uint32_t now_ms)
	{
		if (done_ >= cfg_.max_deployments) {
			return Action::None;
		}
		state_ = DeployState::Alert;
		t0_ = now_ms;
		return Action::StartAlert;
	}

	Config cfg_;
	DeployState state_ = DeployState::Monitoring;
	uint8_t done_ = 0;
	uint32_t t0_ = 0;
	uint32_t last_deploy_ms_ = 0;
	bool redose_timer_ = false;
};

} // namespace dev::algo
