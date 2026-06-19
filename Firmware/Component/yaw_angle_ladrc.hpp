#ifndef __YAW_ANGLE_LADRC_HPP
#define __YAW_ANGLE_LADRC_HPP

#include "Task/utils.hpp"  // for wrap_to_pi, kPi

class YawAngleLadrc {
public:
    struct Config {
        float leso_bw;  // LESO bandwidth (rad/s)
        float ctrl_bw;  // controller bandwidth (rad/s)
    };

    explicit YawAngleLadrc(const Config& cfg) : cfg_(cfg) {}

    void set_config(const Config& cfg) { cfg_ = cfg; }

    void reset(float init_angle_wrapped) {
        z1_ = init_angle_wrapped;
        z2_ = 0.0f;
        prev_omega_ = 0.0f;
    }

    float step(float target_wrapped, float measured_yaw_unwrapped,
               float yaw_max_vel, float dt_s) {
        const float wo = cfg_.leso_bw;
        const float wc = cfg_.ctrl_bw;

        // Gains: pole placement at wo
        const float L1 = 2.0f * wo;
        const float L2 = wo * wo;

        // Wrap measured yaw and compute observer error
        const float yaw_wrapped = wrap_to_pi(measured_yaw_unwrapped);
        const float err = wrap_to_pi(yaw_wrapped - z1_);

        // LESO update with prev_omega feedforward
        // Model: dq/dt = prev_omega + z2  (z2 = unknown velocity disturbance)
        z1_ += dt_s * (z2_ + L1 * err + prev_omega_);
        z1_ = wrap_to_pi(z1_);
        z2_ += dt_s * L2 * err;

        // Control: wrap_to_pi(target - z1) for shortest-path angle error
        const float angle_err = wrap_to_pi(target_wrapped - z1_);
        float omega_ref = wc * angle_err - z2_;  // P + disturbance rejection

        // Clamp to velocity limit
        if (omega_ref > yaw_max_vel) {
            omega_ref = yaw_max_vel;
        } else if (omega_ref < -yaw_max_vel) {
            omega_ref = -yaw_max_vel;
        }

        prev_omega_ = omega_ref;
        return omega_ref;
    }

    float z1() const { return z1_; }
    float z2() const { return z2_; }

private:
    Config cfg_;
    float z1_ = 0.0f;   // wrapped angle estimate (LESO state 1, rad)
    float z2_ = 0.0f;   // lumped velocity disturbance (LESO state 2, rad/s)
    float prev_omega_ = 0.0f;
};

#endif // __YAW_ANGLE_LADRC_HPP
