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

constexpr float kDegToRad = 3.1415926535f / 180.0f;
constexpr float kRadToDeg = 180.0f / 3.1415926535f;
constexpr float kRpmToRadPerSec = 2.0f * 3.1415926535f / 60.0f;

} // namespace

ChassisEstimator::ChassisEstimator() {
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

    if (control_config::kChassisVelocitySource == 1) {
        // Optical-flow-based velocity
        const std::optional<float> optflow_vx = optflow_vx_input_port_.any();
        const std::optional<float> optflow_vy = optflow_vy_input_port_.any();
        if (optflow_vx.has_value() && optflow_vy.has_value()) {
            // Convert mm/s → m/s for chassis_vel_meas
            chassis_vel_meas[0] = *optflow_vx * 0.001f;
            chassis_vel_meas[1] = *optflow_vy * 0.001f;
        }
        // omega_z (chassis_vel_meas[2]) stays 0 from wheel fallback or from below
    }

    if (control_config::kChassisVelocitySource == 0 || chassis_vel_meas[0] == 0.0f) {
        // Wheel-based velocity (Jacobian pseudo-inverse from motor velocities)
        // Only vx (row 0) and vy (row 1) — omega_z comes from IMU
        for (int row = 0; row < 2; ++row) {
            for (int col = 0; col < 4; ++col) {
                chassis_vel_meas[row] += j2_pinv_[row][col] * wheel_vel_rad_s[col];
            }
        }
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
    chassis_yaw_debug = yaw_rad * kRadToDeg;
    chassis_omega_z_debug = omega_z_rad_s * kRadToDeg;

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
