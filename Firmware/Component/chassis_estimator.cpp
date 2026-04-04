#include "chassis_estimator.hpp"
#include "control_params.hpp"
#include <cmath>

// Debug
volatile float chassis_vx_debug = 0;
volatile float chassis_vy_debug = 0;
volatile float chassis_yaw_debug = 0;
volatile float chassis_omega_z_debug = 0;
volatile float wheel_vel_0_debug = 0;

namespace {

constexpr float kPi = 3.1415926535f;
constexpr float kDegToRad = kPi / 180.0f;
constexpr float kRpmToRadPerSec = 2.0f * kPi / 60.0f;

float wrap_to_pi(float angle_rad) {
    while (angle_rad > kPi) {
        angle_rad -= 2.0f * kPi;
    }
    while (angle_rad < -kPi) {
        angle_rad += 2.0f * kPi;
    }
    return angle_rad;
}

} // namespace

ChassisEstimator::ChassisEstimator() {
    yaw_pll_kp_ = 2.0f * control_config::kImuYawPllBandwidth;
    yaw_pll_ki_ = 0.25f * yaw_pll_kp_ * yaw_pll_kp_;
    if (control_config::kControlDtSec * yaw_pll_kp_ >= 1.0f && control_config::kControlDtSec > 0.0f) {
        yaw_pll_kp_ = 0.95f / control_config::kControlDtSec;
        yaw_pll_ki_ = 0.25f * yaw_pll_kp_ * yaw_pll_kp_;
    }
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
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 4; ++col) {
            chassis_vel_meas[row] += j2_pinv_[row][col] * wheel_vel_rad_s[col];
        }
    }

    const std::optional<float> yaw_deg = imu_yaw_input_port_.any();

    float yaw_rad = last_yaw_rad_;
    bool yaw_updated = false;
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
        yaw_updated = true;
    }

    if (!yaw_pll_initialized_) {
        if (yaw_updated) {
            // Align PLL state to the first valid yaw sample to avoid startup spikes.
            yaw_pll_pos_est_rad_ = yaw_rad;
            yaw_pll_vel_est_rad_s_ = 0.0f;
            yaw_omega_ramp_alpha_ = 0.0f;
            yaw_pll_initialized_ = true;
        }
    }

    if (yaw_pll_initialized_) {
        // Predict stage keeps velocity continuity when yaw updates are sparse.
        yaw_pll_pos_est_rad_ += dt_s * yaw_pll_vel_est_rad_s_;
        if (yaw_updated) {
            const float yaw_err = wrap_to_pi(yaw_rad - yaw_pll_pos_est_rad_);
            yaw_pll_pos_est_rad_ += dt_s * yaw_pll_kp_ * yaw_err;
            yaw_pll_vel_est_rad_s_ += dt_s * yaw_pll_ki_ * yaw_err;
        }

        if (std::fabs(yaw_pll_vel_est_rad_s_) < control_config::kImuYawPllZeroSnapEpsRadS) {
            yaw_pll_vel_est_rad_s_ = 0.0f;
        }
    } else {
        yaw_pll_vel_est_rad_s_ = 0.0f;
        yaw_omega_ramp_alpha_ = 0.0f;
    }

    if (yaw_pll_initialized_) {
        const float ramp_time_s = control_config::kImuYawPllOmegaRampTimeSec;
        if (ramp_time_s <= 1e-6f) {
            yaw_omega_ramp_alpha_ = 1.0f;
        } else {
            yaw_omega_ramp_alpha_ += dt_s / ramp_time_s;
            if (yaw_omega_ramp_alpha_ > 1.0f) {
                yaw_omega_ramp_alpha_ = 1.0f;
            }
        }
    }

    float omega_z_rad_s = yaw_pll_vel_est_rad_s_ * yaw_omega_ramp_alpha_;
    if (control_config::kChassisOmegaZSource == control_config::ChassisOmegaZSource::kImuOmegaDirect) {
        const std::optional<float> imu_omega_z_deg_s = imu_omega_z_input_port_.any();
        if (imu_omega_z_deg_s.has_value()) {
            omega_z_rad_s = *imu_omega_z_deg_s * kDegToRad;
        }
    }

    last_yaw_rad_ = yaw_rad;
    last_omega_z_rad_s_ = omega_z_rad_s;

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
    yaw_pll_pos_est_rad_ = 0.0f;
    yaw_pll_vel_est_rad_s_ = 0.0f;
    yaw_omega_ramp_alpha_ = 0.0f;
    yaw_pll_initialized_ = false;

    chassis_vx_output_port_ = 0.0f;
    chassis_vy_output_port_ = 0.0f;
    chassis_yaw_output_port_ = 0.0f;
    chassis_omega_z_output_port_ = 0.0f;
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
