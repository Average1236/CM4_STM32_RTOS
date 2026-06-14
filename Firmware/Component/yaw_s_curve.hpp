#ifndef __YAW_S_CURVE_HPP
#define __YAW_S_CURVE_HPP

#include <algorithm>
#include <cmath>

/// Trapezoidal yaw angle planner.
///
/// The class name is kept for call-site compatibility. The implementation uses
/// the measured yaw/omega as the initial state. If the current omega is moving
/// away from the target, or cannot stop before the target, the planner first
/// brakes to zero and allows the yaw reference to overshoot, then plans a normal
/// trapezoid back to the target. Linear interpolation is kept only for invalid
/// parameter or numeric fallback cases.
class YawSCurve {
public:
    struct Config {
        float vmax;               // max angular velocity (rad/s)
        float amax;               // max angular acceleration (rad/s^2)
        float fallback_min_time;  // minimum linear fallback duration (s)
    };

    explicit YawSCurve(const Config& cfg) : cfg_(cfg) {}

    void set_config(const Config& cfg) { cfg_ = cfg; }

    /// Call when target changes. current_rad = measured yaw (unwrapped, rad).
    /// current_vel = measured angular velocity (rad/s).
    void set_target(float target_rad, float current_rad, float current_vel) {
        start_pos_ = current_rad;
        const float dq = wrap_pm_pi(target_rad - current_rad);
        target_pos_ = current_rad + dq;
        elapsed_ = 0.0f;
        mode_ = Mode::Idle;
        has_brake_ = false;
        has_trapezoid_ = false;

        if (!try_build_plan(dq, current_rad, current_vel)) {
            build_linear_fallback(dq);
            mode_ = Mode::LinearFallback;
            return;
        }

        if (total_time_ > kTimeEps) {
            mode_ = Mode::Trapezoid;
        }
    }

    void step(float dt_s) {
        if (!active() || dt_s <= 0.0f) {
            return;
        }

        elapsed_ += dt_s;
        if (elapsed_ >= total_time_) {
            elapsed_ = total_time_;
            mode_ = Mode::Idle;
        }
    }

    float position() const {
        if (mode_ == Mode::Trapezoid) {
            return sample_plan_pos(elapsed_);
        }
        if (mode_ == Mode::LinearFallback) {
            return start_pos_ + fallback_vel_ * elapsed_;
        }
        return target_pos_;
    }

    float velocity() const {
        if (mode_ == Mode::Trapezoid) {
            return sample_plan_vel(elapsed_);
        }
        if (mode_ == Mode::LinearFallback) {
            return fallback_vel_;
        }
        return 0.0f;
    }

    bool active() const { return mode_ != Mode::Idle; }

private:
    enum class Mode {
        Idle,
        Trapezoid,
        LinearFallback,
    };

    struct TrapSegment {
        float start_pos = 0.0f;
        float direction = 1.0f;
        float distance = 0.0f;
        float start_vel = 0.0f;
        float peak_vel = 0.0f;
        float t_acc = 0.0f;
        float t_flat = 0.0f;
        float t_dec = 0.0f;
        float total_time = 0.0f;
        float q_acc_end = 0.0f;
        float q_flat_end = 0.0f;
    };

    Config cfg_;
    Mode mode_ = Mode::Idle;

    float start_pos_ = 0.0f;
    float target_pos_ = 0.0f;
    float elapsed_ = 0.0f;
    float total_time_ = 0.0f;

    bool has_brake_ = false;
    float brake_start_pos_ = 0.0f;
    float brake_start_vel_ = 0.0f;
    float brake_acc_ = 0.0f;
    float brake_time_ = 0.0f;

    bool has_trapezoid_ = false;
    TrapSegment trapezoid_;

    float fallback_vel_ = 0.0f;

    static constexpr float kPositionEps = 1e-6f;
    static constexpr float kVelocityEps = 1e-5f;
    static constexpr float kTimeEps = 1e-6f;

    static float wrap_pm_pi(float x) {
        while (x > 3.1415926535f)  x -= 6.283185307f;
        while (x < -3.1415926535f) x += 6.283185307f;
        return x;
    }

    static float sign_nonzero(float x) {
        return (x >= 0.0f) ? 1.0f : -1.0f;
    }

    bool try_build_plan(float dq, float current_rad, float current_vel) {
        const float vmax = fabsf(cfg_.vmax);
        const float amax = fabsf(cfg_.amax);
        if (vmax <= kVelocityEps || amax <= kVelocityEps || !std::isfinite(current_vel)) {
            return false;
        }

        const float current_vel_limited = std::clamp(current_vel, -vmax, vmax);
        if (fabsf(dq) < kPositionEps && fabsf(current_vel_limited) < kVelocityEps) {
            total_time_ = 0.0f;
            return true;
        }

        float plan_start_pos = current_rad;
        float plan_start_vel = current_vel_limited;
        const float target_direction =
            (fabsf(dq) >= kPositionEps) ? sign_nonzero(dq) : sign_nonzero(current_vel_limited);
        const float start_vel_to_target = target_direction * current_vel_limited;
        const float stop_distance =
            (start_vel_to_target > 0.0f) ? (start_vel_to_target * start_vel_to_target) / (2.0f * amax) : 0.0f;
        const bool moving_away = start_vel_to_target < -kVelocityEps;
        const bool cannot_stop_before_target = stop_distance > fabsf(dq) + kPositionEps;

        if (moving_away || cannot_stop_before_target) {
            build_brake_segment(current_rad, current_vel_limited, amax);
            plan_start_pos = sample_brake_pos(brake_time_);
            plan_start_vel = 0.0f;
        }

        if (fabsf(target_pos_ - plan_start_pos) >= kPositionEps) {
            if (!build_trapezoid_segment(plan_start_pos, target_pos_, plan_start_vel, vmax, amax, trapezoid_)) {
                return false;
            }
            has_trapezoid_ = true;
        }

        total_time_ = brake_time_ + (has_trapezoid_ ? trapezoid_.total_time : 0.0f);
        return true;
    }

    void build_brake_segment(float current_rad, float current_vel, float amax) {
        if (fabsf(current_vel) < kVelocityEps) {
            return;
        }

        has_brake_ = true;
        brake_start_pos_ = current_rad;
        brake_start_vel_ = current_vel;
        brake_acc_ = -sign_nonzero(current_vel) * amax;
        brake_time_ = fabsf(current_vel) / amax;
    }

    bool build_trapezoid_segment(
        float from_pos,
        float to_pos,
        float start_vel,
        float vmax,
        float amax,
        TrapSegment& segment
    ) const {
        const float dq = to_pos - from_pos;
        const float distance = fabsf(dq);
        if (distance < kPositionEps) {
            return false;
        }

        segment = {};
        segment.start_pos = from_pos;
        segment.direction = sign_nonzero(dq);
        segment.distance = distance;
        segment.start_vel = std::clamp(segment.direction * start_vel, 0.0f, vmax);

        const float d_acc_to_vmax =
            (vmax * vmax - segment.start_vel * segment.start_vel) / (2.0f * amax);
        const float d_dec_from_vmax = (vmax * vmax) / (2.0f * amax);

        if (d_acc_to_vmax + d_dec_from_vmax <= distance) {
            segment.peak_vel = vmax;
            segment.t_acc = (segment.peak_vel - segment.start_vel) / amax;
            segment.t_flat = (distance - d_acc_to_vmax - d_dec_from_vmax) / segment.peak_vel;
        } else {
            segment.peak_vel = sqrtf(fmaxf(amax * distance + 0.5f * segment.start_vel * segment.start_vel, 0.0f));
            segment.peak_vel = std::min(segment.peak_vel, vmax);
            if (segment.peak_vel + kVelocityEps < segment.start_vel) {
                return false;
            }
            segment.t_acc = (segment.peak_vel - segment.start_vel) / amax;
            segment.t_flat = 0.0f;
        }

        segment.t_acc = fmaxf(segment.t_acc, 0.0f);
        segment.t_dec = segment.peak_vel / amax;
        segment.total_time = segment.t_acc + segment.t_flat + segment.t_dec;
        if (segment.total_time <= kTimeEps || !std::isfinite(segment.total_time)) {
            return false;
        }

        segment.q_acc_end =
            segment.start_vel * segment.t_acc + 0.5f * amax * segment.t_acc * segment.t_acc;
        segment.q_flat_end = segment.q_acc_end + segment.peak_vel * segment.t_flat;
        return true;
    }

    void build_linear_fallback(float dq) {
        const float vmax = fabsf(cfg_.vmax);
        const float min_time = fmaxf(cfg_.fallback_min_time, kTimeEps);
        if (vmax <= kVelocityEps) {
            total_time_ = min_time;
        } else {
            total_time_ = fmaxf(fabsf(dq) / vmax, min_time);
        }
        fallback_vel_ = dq / total_time_;
    }

    float sample_plan_pos(float t) const {
        const float tc = std::clamp(t, 0.0f, total_time_);
        if (has_brake_ && tc < brake_time_) {
            return sample_brake_pos(tc);
        }

        if (has_trapezoid_) {
            return sample_trapezoid_pos(trapezoid_, tc - brake_time_);
        }

        return target_pos_;
    }

    float sample_plan_vel(float t) const {
        const float tc = std::clamp(t, 0.0f, total_time_);
        if (has_brake_ && tc < brake_time_) {
            return brake_start_vel_ + brake_acc_ * tc;
        }

        if (has_trapezoid_) {
            return sample_trapezoid_vel(trapezoid_, tc - brake_time_);
        }

        return 0.0f;
    }

    float sample_brake_pos(float t) const {
        const float tc = std::clamp(t, 0.0f, brake_time_);
        return brake_start_pos_ + brake_start_vel_ * tc + 0.5f * brake_acc_ * tc * tc;
    }

    float sample_trapezoid_pos(const TrapSegment& segment, float t) const {
        const float tc = std::clamp(t, 0.0f, segment.total_time);
        const float amax = fabsf(cfg_.amax);
        float q = 0.0f;

        if (tc < segment.t_acc) {
            q = segment.start_vel * tc + 0.5f * amax * tc * tc;
        } else if (tc < segment.t_acc + segment.t_flat) {
            q = segment.q_acc_end + segment.peak_vel * (tc - segment.t_acc);
        } else {
            const float td = tc - segment.t_acc - segment.t_flat;
            q = segment.q_flat_end + segment.peak_vel * td - 0.5f * amax * td * td;
        }

        return segment.start_pos + segment.direction * q;
    }

    float sample_trapezoid_vel(const TrapSegment& segment, float t) const {
        const float tc = std::clamp(t, 0.0f, segment.total_time);
        const float amax = fabsf(cfg_.amax);
        float vel = 0.0f;

        if (tc < segment.t_acc) {
            vel = segment.start_vel + amax * tc;
        } else if (tc < segment.t_acc + segment.t_flat) {
            vel = segment.peak_vel;
        } else {
            const float td = tc - segment.t_acc - segment.t_flat;
            vel = fmaxf(segment.peak_vel - amax * td, 0.0f);
        }

        return segment.direction * vel;
    }
};

#endif // __YAW_S_CURVE_HPP
