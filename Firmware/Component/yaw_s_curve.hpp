#ifndef __YAW_S_CURVE_HPP
#define __YAW_S_CURVE_HPP

#include <algorithm>
#include <cmath>
#include "Task/utils.hpp"

static constexpr float kYawControlDtSec = 0.002f;  // = kControlDtSec (TIM2-driven)

/// Incremental yaw reference tracker.
///
/// A tracking differentiator (TD) estimates the smoothed target angle and its
/// velocity from the wrapped target measurement.  The internal reference state
/// (q_ref, v_ref, a_ref) rolls forward every control cycle, rate-limited by
/// the acceleration limit.  Position correction uses e/dt_s + target-velocity
/// feedforward.
///
/// All angles are kept wrapped to [-pi, pi].
class YawSCurve {
public:
    struct Config {
        float vmax;                  // max angular velocity (rad/s)
        float amax;                  // max angular acceleration (rad/s^2)
        float stop_band_rad;         // snap-to-zero deadband (rad)
        float target_vel_zero_eps;   // threshold to judge target as stationary (rad/s)
    };

    explicit YawSCurve(const Config& cfg) : cfg_(cfg) {}

    ~YawSCurve() { delete td_; }

    void set_config(const Config& cfg) { cfg_ = cfg; }

    /// One-time initialisation.  Must be called before step().
    void reset(float current_yaw, float current_omega,
               float target_wrapped, float /*dt_s*/) {
        q_ref_ = wrap_pm_pi(current_yaw);
        v_ref_ = current_omega;
        a_ref_ = 0.0f;
        target_meas_ = wrap_pm_pi(target_wrapped);

        TD::Parameter_t td_param{};
        td_param.r  = 200.0f;
        td_param.h  = 0.06f;
        td_param.dt = kYawControlDtSec;
        td_param.is_cycle   = true;
        td_param.cycle_low  = -kPi;
        td_param.cycle_high =  kPi;

        delete td_;
        td_ = new TD(td_param, target_meas_);
        initialized_ = true;
    }

    /// Update the target measurement without resetting internal state.
    void set_target_measurement(float target_wrapped) {
        target_meas_ = wrap_pm_pi(target_wrapped);
    }

    /// Advance the reference by one control period.
    void step(float dt_s) {
        if (!initialized_ || !td_ || dt_s <= 1e-6f) {
            return;
        }

        const float vmax = fabsf(cfg_.vmax);
        const float amax = fabsf(cfg_.amax);

        // 1. Feed target measurement to TD
        td_->calc(target_meas_);

        // 2. Read TD estimates
        const float target_pos_est = td_->get_data();   // wrapped to [-pi, pi]
        const float target_vel_est = td_->get_diff();    // rad/s

        // 3. Position error (wrapped)
        const float e = wrap_pm_pi(target_pos_est - q_ref_);

        // 4. Desired velocity = feedforward + immediate position correction.
        //    e/dt_s closes the gap in one step; the accel limit naturally
        //    damps the response.
        float v_des = target_vel_est + e / dt_s;
        v_des = std::clamp(v_des, -vmax, vmax);

        // 5. Conditional stop envelope — only when target is (nearly) stationary
        if (fabsf(target_vel_est) < cfg_.target_vel_zero_eps) {
            const float v_stop = sqrtf(fmaxf(0.0f, 2.0f * amax * fabsf(e)));
            v_des = std::clamp(v_des, -v_stop, v_stop);
        }

        // 6. Acceleration limit
        a_ref_ = (v_des - v_ref_) / dt_s;
        a_ref_ = std::clamp(a_ref_, -amax, amax);

        // 7. Integrate velocity
        v_ref_ += a_ref_ * dt_s;
        v_ref_ = std::clamp(v_ref_, -vmax, vmax);

        // 8. Integrate position (wrap to [-pi, pi])
        q_ref_ += v_ref_ * dt_s;
        q_ref_ = wrap_pm_pi(q_ref_);

        // 9. Snap-to-zero when target is stationary and reference is close
        if (fabsf(target_vel_est) < cfg_.target_vel_zero_eps &&
            fabsf(e) < cfg_.stop_band_rad &&
            fabsf(v_ref_) < sqrtf(2.0f * amax * cfg_.stop_band_rad)) {
            q_ref_ = target_pos_est;
            v_ref_ = 0.0f;
            a_ref_ = 0.0f;
        }
    }

    float position()     const { return q_ref_; }
    float velocity()     const { return v_ref_; }
    float acceleration() const { return a_ref_; }
    bool  active()       const { return initialized_; }

private:
    Config cfg_;
    bool initialized_ = false;

    float q_ref_ = 0.0f;
    float v_ref_ = 0.0f;
    float a_ref_ = 0.0f;

    float target_meas_ = 0.0f;

    TD* td_ = nullptr;

    static float wrap_pm_pi(float x) {
        while (x >  kPi) x -= 2.0f * kPi;
        while (x < -kPi) x += 2.0f * kPi;
        return x;
    }
};

#endif // __YAW_S_CURVE_HPP
