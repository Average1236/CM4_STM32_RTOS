#include "chassis_estimator.hpp"
#include "control_params.hpp"
#include <algorithm>
#include <cmath>

// Debug
volatile float chassis_vx_debug = 0;
volatile float chassis_vy_debug = 0;
volatile float chassis_yaw_debug = 0;
volatile float chassis_omega_z_debug = 0;
volatile float wheel_vel_0_debug = 0;
volatile float wheel_vx_debug = 0;
volatile float wheel_vy_debug = 0;
volatile float of_vx_debug = 0;
volatile float of_vy_debug = 0;
volatile float fusion_r_wheel_x_debug = 0;
volatile float fusion_r_wheel_y_debug = 0;
volatile float fusion_wheel_residual_x_debug = 0;
volatile float fusion_wheel_residual_y_debug = 0;
volatile float fusion_acc_penalty_x_debug = 0;
volatile float fusion_acc_penalty_y_debug = 0;
volatile float vision_vx_debug = 0;
volatile float vision_vy_debug = 0;
volatile float velocity_source_debug = 0;

namespace {

constexpr float kDegToRad = 3.1415926535f / 180.0f;
constexpr float kRadToDeg = 180.0f / 3.1415926535f;
constexpr float kRpmToRadPerSec = 2.0f * 3.1415926535f / 60.0f;

float accel_penalty_multiplier(float acc_abs, float threshold, float gain) {
    return 1.0f + gain * fmaxf(0.0f, acc_abs - threshold);
}

float slip_residual_penalty_multiplier(
    float residual_abs,
    float acc_abs,
    float residual_threshold,
    float accel_threshold,
    float gain
) {
    if (acc_abs <= accel_threshold || residual_abs <= residual_threshold) {
        return 1.0f;
    }

    return 1.0f + gain * (residual_abs - residual_threshold);
}

float limit_velocity_axis(float target, float last, float dt_s,
                          float max_velocity, float max_accel) {
    target = std::clamp(target, -max_velocity, max_velocity);
    const float max_delta = max_accel * dt_s;
    return last + std::clamp(target - last, -max_delta, max_delta);
}

} // namespace

ChassisEstimator::ChassisEstimator()
    : vision_vx_filter_({control_config::kVisionVelocityButterworthCutoffHz,
                         control_config::kControlDtSec},
                        0.0f),
      vision_vy_filter_({control_config::kVisionVelocityButterworthCutoffHz,
                         control_config::kControlDtSec},
                        0.0f) {
    precompute_mappings();
}

void ChassisEstimator::step(float dt_s) {
    if (dt_s <= 0.0f) {
        return;
    }

    float wheel_vel_rad_s[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    for (int i = 0; i < 4; ++i) {
        const std::optional<float> wheel_vel_rpm = wheel_velocity_input_ports_[i].any();
        const float wheel_rpm = wheel_vel_rpm.has_value() ? *wheel_vel_rpm : last_wheel_vel_rpm_[i];
        last_wheel_vel_rpm_[i] = wheel_rpm;
        wheel_vel_rad_s[i] = wheel_rpm * kRpmToRadPerSec;
    }
    wheel_vel_0_debug = wheel_vel_rad_s[0];

    float chassis_vel_meas[3] = {0.0f, 0.0f, 0.0f};

    // Compute wheel-based vx/vy once (used by source==0 and source==2)
    float wheel_vx = 0.0f, wheel_vy = 0.0f;
    for (int col = 0; col < 4; ++col) {
        wheel_vx += j2_pinv_[0][col] * wheel_vel_rad_s[col];
        wheel_vy += j2_pinv_[1][col] * wheel_vel_rad_s[col];
    }
    wheel_vx *= control_config::kWheelChassisVxScale;
    wheel_vy *= control_config::kWheelChassisVyScale;

    if (!wheel_chassis_limiter_initialized_) {
        limited_wheel_chassis_vx_ = std::clamp(wheel_vx,
                                               -control_config::kWheelChassisVelocityLimitMS,
                                               control_config::kWheelChassisVelocityLimitMS);
        limited_wheel_chassis_vy_ = std::clamp(wheel_vy,
                                               -control_config::kWheelChassisVelocityLimitMS,
                                               control_config::kWheelChassisVelocityLimitMS);
        wheel_chassis_limiter_initialized_ = true;
    } else {
        limited_wheel_chassis_vx_ = limit_velocity_axis(
            wheel_vx, limited_wheel_chassis_vx_, dt_s,
            control_config::kWheelChassisVelocityLimitMS,
            control_config::kWheelChassisAccelLimitMS2);
        limited_wheel_chassis_vy_ = limit_velocity_axis(
            wheel_vy, limited_wheel_chassis_vy_, dt_s,
            control_config::kWheelChassisVelocityLimitMS,
            control_config::kWheelChassisAccelLimitMS2);
    }
    wheel_vx = limited_wheel_chassis_vx_;
    wheel_vy = limited_wheel_chassis_vy_;

    wheel_chassis_vx_output_port_ = wheel_vx;
    wheel_chassis_vy_output_port_ = wheel_vy;

    const auto vision_source_opt = vision_source_input_port_.any();
    uint8_t velocity_source = control_config::kChassisVelocitySource;
    if (vision_source_opt.has_value() && *vision_source_opt > 0.5f) {
        velocity_source = static_cast<uint8_t>(*vision_source_opt);
    }
    velocity_source_debug = static_cast<float>(velocity_source);

    if (velocity_source == 3) {
        // --- Fused: optflow velocity + raw vision velocity.
        // Vision velocity is trusted more strongly via lower measurement noise.
        const auto of_vx = optflow_vx_input_port_.any();
        const auto of_vy = optflow_vy_input_port_.any();
        const auto vision_vx = vision_vx_input_port_.any();
        const auto vision_vy = vision_vy_input_port_.any();
        const float flow_vx = of_vx.has_value() ? *of_vx * 0.001f : kf_vision_vx_.v_est;
        const float flow_vy = of_vy.has_value() ? *of_vy * 0.001f : kf_vision_vy_.v_est;
        const float raw_vision_vx = vision_vx.has_value() ? *vision_vx * 0.001f : kf_vision_vx_.v_est;
        const float raw_vision_vy = vision_vy.has_value() ? *vision_vy * 0.001f : kf_vision_vy_.v_est;

        wheel_vx_debug = wheel_vx;
        wheel_vy_debug = wheel_vy;
        of_vx_debug = flow_vx;
        of_vy_debug = flow_vy;
        vision_vx_debug = raw_vision_vx;
        vision_vy_debug = raw_vision_vy;

        kf_vision_vx_.predict(control_config::kVisionOptflowFusionQX);
        if (of_vx.has_value()) {
            kf_vision_vx_.update(flow_vx, control_config::kVisionOptflowFusionROptflowX);
        }
        if (vision_vx.has_value()) {
            kf_vision_vx_.update(raw_vision_vx, control_config::kVisionOptflowFusionRVisionX);
        }

        kf_vision_vy_.predict(control_config::kVisionOptflowFusionQY);
        if (of_vy.has_value()) {
            kf_vision_vy_.update(flow_vy, control_config::kVisionOptflowFusionROptflowY);
        }
        if (vision_vy.has_value()) {
            kf_vision_vy_.update(raw_vision_vy, control_config::kVisionOptflowFusionRVisionY);
        }

        chassis_vel_meas[0] = kf_vision_vx_.v_est;
        chassis_vel_meas[1] = kf_vision_vy_.v_est;
        fused_chassis_vx_output_port_ = kf_vision_vx_.v_est;
        fused_chassis_vy_output_port_ = kf_vision_vy_.v_est;

    } else if (velocity_source == 4) {
        // --- Pure vision velocity with Butterworth low-pass filter (no Kalman).
        const auto vision_vx = vision_vx_input_port_.any();
        const auto vision_vy = vision_vy_input_port_.any();
        const float raw_vision_vx = vision_vx.has_value() ? *vision_vx * 0.001f : last_vision_vx_;
        const float raw_vision_vy = vision_vy.has_value() ? *vision_vy * 0.001f : last_vision_vy_;
        last_vision_vx_ = raw_vision_vx;
        last_vision_vy_ = raw_vision_vy;

        const float filt_vx = vision_vx_filter_.filter(raw_vision_vx);
        const float filt_vy = vision_vy_filter_.filter(raw_vision_vy);

        wheel_vx_debug = wheel_vx;
        wheel_vy_debug = wheel_vy;
        of_vx_debug = 0.0f;
        of_vy_debug = 0.0f;
        vision_vx_debug = raw_vision_vx;
        vision_vy_debug = raw_vision_vy;

        chassis_vel_meas[0] = filt_vx;
        chassis_vel_meas[1] = filt_vy;
        fused_chassis_vx_output_port_ = filt_vx;
        fused_chassis_vy_output_port_ = filt_vy;

    } else if (velocity_source == 2) {
        // --- Fused: adaptive Kalman (wheel + optflow), per-axis params ---
        const auto of_vx = optflow_vx_input_port_.any();
        const auto of_vy = optflow_vy_input_port_.any();
        const float flow_vx = of_vx.has_value() ? *of_vx * 0.001f : 0.0f; // mm/s→m/s
        const float flow_vy = of_vy.has_value() ? *of_vy * 0.001f : 0.0f;

        wheel_vx_debug = wheel_vx;
        wheel_vy_debug = wheel_vy;
        of_vx_debug = flow_vx;
        of_vy_debug = flow_vy;

        // Tilt penalty (shared computation)
        const auto gx_opt = imu_gyro_x_input_port_.any();
        const auto gy_opt = imu_gyro_y_input_port_.any();
        const float gx = gx_opt.has_value() ? *gx_opt : 0.0f;
        const float gy = gy_opt.has_value() ? *gy_opt : 0.0f;
        const float tilt_rate = sqrtf(gx * gx + gy * gy);

        // ---- vx axis ----
        {
            const float alpha_x = std::clamp(fabsf(kf_vx_.v_est) / control_config::kFusionSpeedTransitionX, 0.0f, 1.0f);
            float R_w_x = control_config::kFusionKalmanRWheelMinX
                        + (control_config::kFusionKalmanRWheelMaxX - control_config::kFusionKalmanRWheelMinX)
                        * (1.0f - alpha_x);
            float R_of_x = control_config::kFusionKalmanROptflowMinX
                         + (control_config::kFusionKalmanROptflowMaxX - control_config::kFusionKalmanROptflowMinX)
                         * alpha_x;
            R_of_x *= (1.0f + control_config::kFusionTiltPenaltyGainX * tilt_rate);

            const float acc_abs_x = fabsf(acc_ref_m_s2_[0]);
            const float wheel_flow_residual_x = of_vx.has_value() ? fabsf(wheel_vx - flow_vx) : 0.0f;
            const float acc_penalty_x = accel_penalty_multiplier(
                acc_abs_x,
                control_config::kFusionWheelAccelThresholdX,
                control_config::kFusionWheelAccelPenaltyGainX);
            const float slip_penalty_x = of_vx.has_value()
                ? slip_residual_penalty_multiplier(
                    wheel_flow_residual_x,
                    acc_abs_x,
                    control_config::kFusionWheelOptflowResidualThresholdX,
                    control_config::kFusionWheelSlipAccelThresholdX,
                    control_config::kFusionWheelSlipResidualPenaltyGainX)
                : 1.0f;
            R_w_x *= acc_penalty_x * slip_penalty_x;

            fusion_r_wheel_x_debug = R_w_x;
            fusion_wheel_residual_x_debug = wheel_flow_residual_x;
            fusion_acc_penalty_x_debug = acc_penalty_x;

            kf_vx_.predict(control_config::kFusionKalmanQX);
            kf_vx_.update(wheel_vx, R_w_x);
            // kf_vx_.update(flow_vx, R_of_x);
        }

        // ---- vy axis ----
        {
            const float alpha_y = std::clamp(fabsf(kf_vy_.v_est) / control_config::kFusionSpeedTransitionY, 0.0f, 1.0f);
            float R_w_y = control_config::kFusionKalmanRWheelMinY
                        + (control_config::kFusionKalmanRWheelMaxY - control_config::kFusionKalmanRWheelMinY)
                        * (1.0f - alpha_y);
            float R_of_y = control_config::kFusionKalmanROptflowMinY
                         + (control_config::kFusionKalmanROptflowMaxY - control_config::kFusionKalmanROptflowMinY)
                         * alpha_y;
            R_of_y *= (1.0f + control_config::kFusionTiltPenaltyGainY * tilt_rate);

            const float acc_abs_y = fabsf(acc_ref_m_s2_[1]);
            const float wheel_flow_residual_y = of_vy.has_value() ? fabsf(wheel_vy - flow_vy) : 0.0f;
            const float acc_penalty_y = accel_penalty_multiplier(
                acc_abs_y,
                control_config::kFusionWheelAccelThresholdY,
                control_config::kFusionWheelAccelPenaltyGainY);
            const float slip_penalty_y = of_vy.has_value()
                ? slip_residual_penalty_multiplier(
                    wheel_flow_residual_y,
                    acc_abs_y,
                    control_config::kFusionWheelOptflowResidualThresholdY,
                    control_config::kFusionWheelSlipAccelThresholdY,
                    control_config::kFusionWheelSlipResidualPenaltyGainY)
                : 1.0f;
            R_w_y *= acc_penalty_y * slip_penalty_y;

            fusion_r_wheel_y_debug = R_w_y;
            fusion_wheel_residual_y_debug = wheel_flow_residual_y;
            fusion_acc_penalty_y_debug = acc_penalty_y;

            kf_vy_.predict(control_config::kFusionKalmanQY);
            kf_vy_.update(wheel_vy, R_w_y);
            // kf_vy_.update(flow_vy, R_of_y);
        }

        chassis_vel_meas[0] = kf_vx_.v_est;
        chassis_vel_meas[1] = kf_vy_.v_est;
        fused_chassis_vx_output_port_ = kf_vx_.v_est;
        fused_chassis_vy_output_port_ = kf_vy_.v_est;

    } else if (velocity_source == 1) {
        // Optical-flow-only velocity
        const auto of_vx = optflow_vx_input_port_.any();
        const auto of_vy = optflow_vy_input_port_.any();
        if (of_vx.has_value() && of_vy.has_value()) {
            chassis_vel_meas[0] = *of_vx * 0.001f;
            chassis_vel_meas[1] = *of_vy * 0.001f;
        }

    } else {
        // Wheel-only velocity (source==0, default)
        chassis_vel_meas[0] = wheel_vx;
        chassis_vel_meas[1] = wheel_vy;
    }

    const std::optional<float> yaw_deg = imu_yaw_input_port_.any();

    float yaw_rad = last_yaw_rad_;
    if (yaw_deg.has_value()) {
        const float current_raw_yaw_rad = *yaw_deg * kDegToRad;
        if (!has_last_raw_yaw_rad_) {
            has_last_raw_yaw_rad_ = true;
            last_raw_yaw_rad_ = current_raw_yaw_rad;
            accumulated_yaw_rad_ = current_raw_yaw_rad;
        } else {
            const float delta_yaw_rad = wrap_to_pi(current_raw_yaw_rad - last_raw_yaw_rad_);
            accumulated_yaw_rad_ += delta_yaw_rad;
            last_raw_yaw_rad_ = current_raw_yaw_rad;
        }
        yaw_rad = accumulated_yaw_rad_;
    }

    float omega_z_rad_s = last_omega_z_rad_s_;
    const std::optional<float> imu_omega_z_deg_s = imu_omega_z_input_port_.any();
    if (imu_omega_z_deg_s.has_value()) {
        omega_z_rad_s = *imu_omega_z_deg_s * kDegToRad;
    }

    last_yaw_rad_ = yaw_rad;
    last_omega_z_rad_s_ = omega_z_rad_s;

    // omega_z always from IMU (not wheel Jacobian)
    chassis_vel_meas[2] = omega_z_rad_s;

    // Debug outputs
    chassis_vx_debug = chassis_vel_meas[0];
    chassis_vy_debug = chassis_vel_meas[1];
    chassis_yaw_debug = yaw_rad;
    chassis_omega_z_debug = omega_z_rad_s;

    chassis_vx_output_port_ = chassis_vel_meas[0];
    chassis_vy_output_port_ = chassis_vel_meas[1];
    chassis_yaw_output_port_ = yaw_rad;
    chassis_omega_z_output_port_ = omega_z_rad_s;
}

void ChassisEstimator::reset() {
    for (int i = 0; i < 4; ++i) {
        last_wheel_vel_rpm_[i] = 0.0f;
    }
    has_last_raw_yaw_rad_ = false;
    last_raw_yaw_rad_ = 0.0f;
    accumulated_yaw_rad_ = 0.0f;
    last_yaw_rad_ = 0.0f;
    last_omega_z_rad_s_ = 0.0f;
    acc_ref_m_s2_[0] = 0.0f;
    acc_ref_m_s2_[1] = 0.0f;

    chassis_vx_output_port_ = 0.0f;
    chassis_vy_output_port_ = 0.0f;
    chassis_yaw_output_port_ = 0.0f;
    chassis_omega_z_output_port_ = 0.0f;
    wheel_chassis_vx_output_port_ = 0.0f;
    wheel_chassis_vy_output_port_ = 0.0f;
    fused_chassis_vx_output_port_ = 0.0f;
    fused_chassis_vy_output_port_ = 0.0f;
    kf_vx_ = {};
    kf_vy_ = {};
    kf_vision_vx_ = {};
    kf_vision_vy_ = {};
    vision_vx_filter_.reset(0.0f);
    vision_vy_filter_.reset(0.0f);
    last_vision_vx_ = 0.0f;
    last_vision_vy_ = 0.0f;
    wheel_chassis_limiter_initialized_ = false;
    limited_wheel_chassis_vx_ = 0.0f;
    limited_wheel_chassis_vy_ = 0.0f;
}

bool ChassisEstimator::inverse3x3(const float in[3][3], float out[3][3]) const {
    const float a = in[0][0], b = in[0][1], c = in[0][2];
    const float d = in[1][0], e = in[1][1], f = in[1][2];
    const float g = in[2][0], h = in[2][1], i = in[2][2];

    const float A = (e * i - f * h);
    const float B = -(d * i - f * g);
    const float C = (d * h - e * g);
    const float D = -(b * i - c * h);
    const float E = (a * i - c * g);
    const float F = -(a * h - b * g);
    const float G = (b * f - c * e);
    const float H = -(a * f - c * d);
    const float I = (a * e - b * d);

    const float det = a * A + b * B + c * C;
    if (std::fabs(det) < 1e-8f) {
        return false;
    }

    const float inv_det = 1.0f / det;
    out[0][0] = A * inv_det;
    out[0][1] = D * inv_det;
    out[0][2] = G * inv_det;
    out[1][0] = B * inv_det;
    out[1][1] = E * inv_det;
    out[1][2] = H * inv_det;
    out[2][0] = C * inv_det;
    out[2][1] = F * inv_det;
    out[2][2] = I * inv_det;
    return true;
}

void ChassisEstimator::precompute_mappings() {
    const float r = control_config::kWheelRadiusM;
    const float l = control_config::kWheelCenterDistanceM;
    const float alpha = control_config::kWheelAlphaRad;
    const float beta = control_config::kWheelBetaRad;

    const float ca = std::cos(alpha);
    const float sa = std::sin(alpha);
    const float cb = std::cos(beta);
    const float sb = std::sin(beta);
    const float inv_r = (r > 1e-9f) ? (1.0f / r) : 0.0f;

    const float j2[4][3] = {
        {inv_r * ca, inv_r * (-sa), inv_r * (-l)},
        {inv_r * (-ca), inv_r * (-sa), inv_r * (-l)},
        {inv_r * (-cb), inv_r * sb, inv_r * (-l)},
        {inv_r * cb, inv_r * sb, inv_r * (-l)},
    };

    float j2t_j2[3][3] = {{0.0f}};
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            for (int k = 0; k < 4; ++k) {
                j2t_j2[row][col] += j2[k][row] * j2[k][col];
            }
        }
    }

    float j2t_j2_inv[3][3] = {{0.0f}};
    if (inverse3x3(j2t_j2, j2t_j2_inv)) {
        for (int row = 0; row < 3; ++row) {
            for (int wheel = 0; wheel < 4; ++wheel) {
                float acc = 0.0f;
                for (int k = 0; k < 3; ++k) {
                    acc += j2t_j2_inv[row][k] * j2[wheel][k];
                }
                j2_pinv_[row][wheel] = acc;
            }
        }
    }
}
