#include "opt_flow.hpp"
#include <cmath>

PositionPLL::PositionPLL(float bandwidth_hz, float zero_snap_threshold,
                         float max_jump_mm, uint8_t jump_confirm_frames)
    : pos_est_mm_(0.0f), vel_est_mm_s_(0.0f),
      kp_(0.0f), ki_(0.0f),
      zero_snap_threshold_(zero_snap_threshold),
      max_jump_mm_(max_jump_mm),
      jump_confirm_frames_(jump_confirm_frames),
      jump_count_(0),
      initialized_(false)
{
    kp_ = 2.0f * bandwidth_hz;
    ki_ = 0.25f * kp_ * kp_;
}

void PositionPLL::reset(float measured_pos_mm) {
    pos_est_mm_   = measured_pos_mm;
    vel_est_mm_s_ = 0.0f;
    jump_count_   = 0;
    initialized_  = true;
}

void PositionPLL::update(float measured_pos_mm, float dt_s) {
    if (dt_s <= 0.0f) return;

    if (!initialized_) {
        reset(measured_pos_mm);
        return;
    }

    const float kp = std::fmin(kp_, .95f / dt_s);
    const float ki = .25f * kp * kp;
    const float pos_pred = pos_est_mm_ + dt_s * vel_est_mm_s_;
    const float pos_err  = measured_pos_mm - pos_pred;

    if (fabsf(pos_err) > max_jump_mm_) {
        jump_count_++;
        if (jump_count_ >= jump_confirm_frames_) {
            reset(measured_pos_mm);
            jump_count_ = 0;
        } else {
            pos_est_mm_ = pos_pred;
        }
        return;
    }
    jump_count_ = 0;

    pos_est_mm_   = pos_pred + dt_s * kp * pos_err;
    vel_est_mm_s_ += dt_s * ki * pos_err;

    if (fabsf(vel_est_mm_s_) < zero_snap_threshold_) {
        vel_est_mm_s_ = 0.0f;
    }
}

bool OptFlow::process(const Data_t& data) {
    state_.valid = false;
    float vx = 0, vy = 0;
    unsigned count = 0;
    bool updated = false;
    std::uint32_t oldest_age = 0;
    for (unsigned i = 0; i < 2; ++i) {
        const auto tick = i ? data.right_tick_ms : data.left_tick_ms;
        const float x = i ? data.right_x : data.left_x;
        const float y = i ? data.right_y : data.left_y;
        const auto age = static_cast<std::uint32_t>(data.tick_ms - tick);
        if (!(data.valid_mask & (1u << i)) || age > 100 || !std::isfinite(x) || !std::isfinite(y)) continue;
        auto& sensor = sensors_[i];
        if (!sensor.initialized) {
            sensor.x.reset(x); sensor.y.reset(y);
            sensor.tick = tick; sensor.initialized = true;
            continue;
        }
        const auto elapsed = static_cast<std::uint32_t>(tick - sensor.tick);
        if (elapsed >= 0x80000000u) continue;
        if (elapsed > 100) {
            sensor.x.reset(x); sensor.y.reset(y); sensor.tick = tick;
            continue;
        }
        if (elapsed != 0) {
            sensor.x.update(x, elapsed * .001f); sensor.y.update(y, elapsed * .001f);
            sensor.tick = tick; updated = true;
        }
        vx += sensor.x.vel(); vy += sensor.y.vel(); ++count;
        if (age > oldest_age) oldest_age = age;
    }
    if (count && updated) {
        state_.body_vx = vx / count; state_.body_vy = vy / count;
        state_.sample_time_ms = data.tick_ms - oldest_age;
        state_.valid = true;
    }
    return state_.valid;
}
