#include "velocity_estimator.hpp"
#include "control_params.hpp"
#include <algorithm>
#include <cmath>
#include <optional>

namespace {

constexpr float kPi = 3.1415926535f;
constexpr float kDegToRad = kPi / 180.0f;
constexpr float kRpmToRadPerSec = 2.0f * kPi / 60.0f;

} // namespace

VelocityEstimator::VelocityEstimator() {
    precompute_mappings();
}

void VelocityEstimator::update(uint32_t) {
    step(control_config::kControlDtSec);
}

void VelocityEstimator::step(float dt_s) {
    if (dt_s <= 0.0f) {
        return;
    }

    float chassis_vel_meas[3] = {0.0f, 0.0f, 0.0f};
    compute_wheel_velocity_measurement(chassis_vel_meas);

    const std::optional<float> ax_opt = imu_acc_x_input_port_.any();
    const std::optional<float> ay_opt = imu_acc_y_input_port_.any();
    const std::optional<float> yaw_opt = imu_yaw_input_port_.any();
    const std::optional<float> omega_z_opt = imu_omega_z_input_port_.any();

    const float ax_raw = ax_opt.has_value() ? *ax_opt : last_acc_mps2_[0];
    const float ay_raw = ay_opt.has_value() ? *ay_opt : last_acc_mps2_[1];
    const float yaw_deg = yaw_opt.has_value() ? *yaw_opt : last_yaw_deg_;
    const float omega_z_deg_s = omega_z_opt.has_value() ? *omega_z_opt : last_omega_z_deg_s_;

    last_acc_mps2_[0] = ax_raw;
    last_acc_mps2_[1] = ay_raw;
    last_yaw_deg_ = yaw_deg;
    last_omega_z_deg_s_ = omega_z_deg_s;

    const float omega_z_rad_s = omega_z_deg_s * kDegToRad;

    if (stationary_detected(chassis_vel_meas[0], chassis_vel_meas[1], omega_z_rad_s, ax_raw, ay_raw)) {
        stationary_counter_++;
    } else {
        stationary_counter_ = 0;
    }

    if (stationary_counter_ >= control_config::kVelEstStationaryCycles) {
        acc_bias_[0] += control_config::kVelEstBiasLearnRate * (ax_raw - acc_bias_[0]);
        acc_bias_[1] += control_config::kVelEstBiasLearnRate * (ay_raw - acc_bias_[1]);
    }

    float ax = ax_raw - acc_bias_[0];
    float ay = ay_raw - acc_bias_[1];

    if (std::fabs(ax) < control_config::kVelEstAccDeadbandMps2) {
        ax = 0.0f;
    }
    if (std::fabs(ay) < control_config::kVelEstAccDeadbandMps2) {
        ay = 0.0f;
    }

    const float vx_pred = vel_est_[0] + dt_s * (ax + omega_z_rad_s * vel_est_[1]);
    const float vy_pred = vel_est_[1] + dt_s * (ay - omega_z_rad_s * vel_est_[0]);

    const float slip_vx = chassis_vel_meas[0] - vx_pred;
    const float slip_vy = chassis_vel_meas[1] - vy_pred;
    const float slip_norm = std::sqrt(slip_vx * slip_vx + slip_vy * slip_vy);

    float alpha = control_config::kVelEstWheelTrustNormal;
    if (slip_norm >= control_config::kVelEstSlipHighMps) {
        alpha = control_config::kVelEstWheelTrustSlip;
    } else if (slip_norm > control_config::kVelEstSlipLowMps) {
        const float ratio =
            (slip_norm - control_config::kVelEstSlipLowMps) /
            (control_config::kVelEstSlipHighMps - control_config::kVelEstSlipLowMps);
        alpha = control_config::kVelEstWheelTrustNormal +
                ratio * (control_config::kVelEstWheelTrustSlip - control_config::kVelEstWheelTrustNormal);
    }

    vel_est_[0] = alpha * chassis_vel_meas[0] + (1.0f - alpha) * vx_pred;
    vel_est_[1] = alpha * chassis_vel_meas[1] + (1.0f - alpha) * vy_pred;

    if (stationary_counter_ >= control_config::kVelEstStationaryCycles) {
        vel_est_[0] *= control_config::kVelEstZeroDamping;
        vel_est_[1] *= control_config::kVelEstZeroDamping;

        if (std::fabs(vel_est_[0]) < control_config::kVelEstZeroWheelSpeedMps) {
            vel_est_[0] = 0.0f;
        }
        if (std::fabs(vel_est_[1]) < control_config::kVelEstZeroWheelSpeedMps) {
            vel_est_[1] = 0.0f;
        }
    }

    vx_output_port_ = vel_est_[0];
    vy_output_port_ = vel_est_[1];
    omega_z_output_port_ = omega_z_rad_s;
    speed_output_port_ = std::sqrt(vel_est_[0] * vel_est_[0] + vel_est_[1] * vel_est_[1]);
}

void VelocityEstimator::reset() {
    vel_est_[0] = 0.0f;
    vel_est_[1] = 0.0f;
    acc_bias_[0] = 0.0f;
    acc_bias_[1] = 0.0f;
    stationary_counter_ = 0;

    for (int i = 0; i < 4; ++i) {
        last_wheel_vel_rpm_[i] = 0.0f;
    }

    last_acc_mps2_[0] = 0.0f;
    last_acc_mps2_[1] = 0.0f;
    last_yaw_deg_ = 0.0f;
    last_omega_z_deg_s_ = 0.0f;

    vx_output_port_ = 0.0f;
    vy_output_port_ = 0.0f;
    omega_z_output_port_ = 0.0f;
    speed_output_port_ = 0.0f;
}

void VelocityEstimator::reset_ports() {
    vx_output_port_.reset();
    vy_output_port_.reset();
    omega_z_output_port_.reset();
    speed_output_port_.reset();
}

bool VelocityEstimator::inverse3x3(const float in[3][3], float out[3][3]) const {
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

void VelocityEstimator::precompute_mappings() {
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

void VelocityEstimator::compute_wheel_velocity_measurement(float chassis_vel_meas[3]) {
    float wheel_vel_rad_s[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    for (int i = 0; i < 4; ++i) {
        const std::optional<float> wheel_vel_rpm = wheel_velocity_input_ports_[i].any();
        const float wheel_rpm = wheel_vel_rpm.has_value() ? *wheel_vel_rpm : last_wheel_vel_rpm_[i];
        last_wheel_vel_rpm_[i] = wheel_rpm;
        wheel_vel_rad_s[i] = wheel_rpm * kRpmToRadPerSec;
    }

    for (int row = 0; row < 3; ++row) {
        chassis_vel_meas[row] = 0.0f;
        for (int col = 0; col < 4; ++col) {
            chassis_vel_meas[row] += j2_pinv_[row][col] * wheel_vel_rad_s[col];
        }
    }
}

bool VelocityEstimator::stationary_detected(float vx_meas, float vy_meas, float omega_z_rad_s, float ax, float ay) const {
    return (std::fabs(vx_meas) < control_config::kVelEstZeroWheelSpeedMps) &&
           (std::fabs(vy_meas) < control_config::kVelEstZeroWheelSpeedMps) &&
           (std::fabs(omega_z_rad_s) < control_config::kVelEstZeroOmegaRadPerSec) &&
           (std::fabs(ax - acc_bias_[0]) < control_config::kVelEstZeroAccMps2) &&
           (std::fabs(ay - acc_bias_[1]) < control_config::kVelEstZeroAccMps2);
}
