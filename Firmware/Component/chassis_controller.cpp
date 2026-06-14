#include "chassis_controller.hpp"
#include "control_params.hpp"
#include <algorithm>
#include <cmath>

volatile float yaw_ref_input_debug = 0;
volatile float yaw_leso_z1_debug = 0;
volatile float yaw_leso_z2_debug = 0;
volatile float vx_ref_debug = 0;
volatile float vy_ref_debug = 0;
volatile float vx_pid_output_debug = 0;
volatile float vy_pid_output_debug = 0;
volatile float vx_pid_integ_debug = 0;
volatile float vy_pid_integ_debug = 0;
volatile float vx_pid_diff_debug = 0;
volatile float vy_pid_diff_debug = 0;
volatile float F_task_0_debug = 0;
volatile float F_task_1_debug = 0;
volatile float F_task_2_debug = 0;
volatile float torque_ff_debug_0 = 0;
volatile float torque_ff_debug_1 = 0;
volatile float u_psi_debug = 0;
volatile float err_psi_pos_debug = 0;
volatile float err_psi_vel_debug = 0;
volatile float err_wz_debug = 0;
volatile float yaw_rad_debug = 0;
volatile float wo_psi_eff_debug = 0;
volatile float omega_z_filt_debug = 0;
volatile float yaw_target_pos_debug = 0;
volatile float yaw_target_vel_debug = 0;

namespace {

const PID::Parameter_t kChassisVxPidParam = {
    .kp = control_config::kChassisVelPidKpX,
    .ki = control_config::kChassisVelPidKiX,
    .kd = control_config::kChassisVelPidKdX,
    .output_limit = control_config::kChassisVelPidOutputLimitX,
    .integ_limit = control_config::kChassisVelPidIntegLimitX,
    .dt = control_config::kControlDtSec,
    .back_calc_gain = control_config::kChassisVelPidBackCalcGainX,
    .diff_cutoff_hz = control_config::kChassisVelPidDiffCutoffHzX,
};

const PID::Parameter_t kChassisVyPidParam = {
    .kp = control_config::kChassisVelPidKpY,
    .ki = control_config::kChassisVelPidKiY,
    .kd = control_config::kChassisVelPidKdY,
    .output_limit = control_config::kChassisVelPidOutputLimitY,
    .integ_limit = control_config::kChassisVelPidIntegLimitY,
    .dt = control_config::kControlDtSec,
    .back_calc_gain = control_config::kChassisVelPidBackCalcGainY,
    .diff_cutoff_hz = control_config::kChassisVelPidDiffCutoffHzY,
};

} // namespace

ChassisController::ChassisController()
    : omega_z_filter_({control_config::kChassisOmegaZFilterCutoffHz,
                       control_config::kControlDtSec},
                      0.0f) {
    precompute_mappings();
}

void ChassisController::set_reference(const float vel_ref[3], const float acc_ref[3]) {
    for (int i = 0; i < 3; ++i) {
        vel_ref_[i] = vel_ref[i];
        acc_ref_[i] = acc_ref[i];
    }
}

void ChassisController::set_use_imu_yaw(bool enable) {
    if (enable != use_imu_) {
        yaw_leso_[0] = 0.0f;
        yaw_leso_[1] = 0.0f;
        yaw_target_pos_ = 0.0f;
        yaw_target_vel_ = 0.0f;
        use_imu_ = enable;
    }
}

void ChassisController::set_yaw_target(float target_pos, float target_vel) {
    yaw_target_pos_ = target_pos;
    yaw_target_vel_ = target_vel;

    yaw_target_pos_debug = target_pos;
    yaw_target_vel_debug = target_vel;
}

void ChassisController::step(float dt_s) {
    if (dt_s <= 0.0f) {
        return;
    }

    const std::optional<float> chassis_vx_meas = chassis_vx_input_port_.any();
    const std::optional<float> chassis_vy_meas = chassis_vy_input_port_.any();
    const std::optional<float> omega_z_meas = chassis_omega_z_input_port_.any();
    const std::optional<float> yaw_meas = chassis_yaw_input_port_.any();

    const float vx_m_s = chassis_vx_meas.has_value() ? *chassis_vx_meas : last_chassis_vx_m_s_;
    const float vy_m_s = chassis_vy_meas.has_value() ? *chassis_vy_meas : last_chassis_vy_m_s_;
    const float omega_z_rad_s_raw = omega_z_meas.has_value() ? *omega_z_meas : last_chassis_omega_z_rad_s_;
    const float omega_z_rad_s = omega_z_filter_.filter(omega_z_rad_s_raw);
    const float yaw_rad = yaw_meas.has_value() ? *yaw_meas : last_chassis_yaw_rad_;

    omega_z_filt_debug = omega_z_rad_s;
    yaw_rad_debug = yaw_rad;

    last_chassis_vx_m_s_ = vx_m_s;
    last_chassis_vy_m_s_ = vy_m_s;
    last_chassis_omega_z_rad_s_ = omega_z_rad_s_raw;
    last_chassis_yaw_rad_ = yaw_rad;

    float tau_sent[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    for (int i = 0; i < 4; ++i) {
        const std::optional<float> tau = wheel_sent_torque_input_ports_[i].any();
        tau_sent[i] = tau.has_value() ? *tau : 0.0f;
    }

    float u_body[3] = {0.0f, 0.0f, 0.0f};
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 4; ++col) {
            u_body[row] += j1_[row][col] * tau_sent[col];
        }
    }
    const float u_psi = u_body[2] / control_config::kRobotInertiaKgM2;
    u_psi_debug = u_psi;

    const float v_norm = sqrtf(vx_m_s * vx_m_s + vy_m_s * vy_m_s);
    const float alpha_v = std::clamp(v_norm / control_config::kLesoScheduleVelocityThreshold, 0.0f, 1.0f);
    const float alpha_omega = std::clamp(fabsf(omega_z_rad_s) / control_config::kLesoScheduleOmegaThreshold, 0.0f, 1.0f);
    const float alpha_psi = fmaxf(alpha_v, alpha_omega);
    const float wo_psi_eff = control_config::kLesoAngleBandwidthMin
                           + (control_config::kLesoAngleObserverBandwidth - control_config::kLesoAngleBandwidthMin) * alpha_psi;
    wo_psi_eff_debug = wo_psi_eff;

    const float L1_w = 2.0f * wo_psi_eff;
    const float L2_w = wo_psi_eff * wo_psi_eff;
    const float err_w = omega_z_rad_s - yaw_leso_[0];
    const float dz1_w = yaw_leso_[1] + L1_w * err_w + u_psi;
    const float dz2_w = L2_w * err_w;
    yaw_leso_[0] += dt_s * dz1_w;
    yaw_leso_[1] += dt_s * dz2_w;

    err_wz_debug = err_w;
    yaw_leso_z1_debug = yaw_leso_[0];
    yaw_leso_z2_debug = yaw_leso_[1];

    float fb_psi;
    float F_task_psi;
    if (use_imu_) {
        const float err_pos = wrap_to_pi(yaw_target_pos_ - yaw_rad);
        const float err_vel = yaw_target_vel_ - omega_z_rad_s;
        err_psi_pos_debug = err_pos;
        err_psi_vel_debug = err_vel;
        const float wc = control_config::kAngleControllerBandwidthMin
                       + (control_config::kAngleControllerBandwidth - control_config::kAngleControllerBandwidthMin) * alpha_psi;
        const float Kp = wc * wc;
        const float Kd = 2.0f * wc;
        fb_psi = Kp * err_pos + Kd * err_vel;
        yaw_ref_input_debug = fb_psi;
        F_task_psi = control_config::kRobotInertiaKgM2 * (fb_psi - yaw_leso_[1]
                     + control_config::kYawVyCoupling * vel_ref_[1]);
    } else {
        fb_psi = control_config::kVelFeedbackGainYaw * (vel_ref_[2] - yaw_leso_[0]);
        err_psi_pos_debug = 0.0f;
        err_psi_vel_debug = vel_ref_[2] - omega_z_rad_s;
        yaw_ref_input_debug = fb_psi;
        F_task_psi = control_config::kRobotInertiaKgM2 * (acc_ref_[2] + fb_psi - yaw_leso_[1]);
    }

    const float pid_vx = vx_pid_.calc(vel_ref_[0], vx_m_s, kChassisVxPidParam);
    const float pid_vy = vy_pid_.calc(vel_ref_[1], vy_m_s, kChassisVyPidParam);

    vx_ref_debug = vel_ref_[0];
    vy_ref_debug = vel_ref_[1];
    vx_pid_output_debug = pid_vx;
    vy_pid_output_debug = pid_vy;
    vx_pid_integ_debug = vx_pid_.get_integ();
    vy_pid_integ_debug = vy_pid_.get_integ();
    vx_pid_diff_debug = vx_pid_.get_diff();
    vy_pid_diff_debug = vy_pid_.get_diff();

    const float F_task[3] = {
        control_config::kRobotMassKg * (acc_ref_[0] + pid_vx),
        control_config::kRobotMassKg * (acc_ref_[1] + pid_vy),
        F_task_psi,
    };

    F_task_0_debug = F_task[0];
    F_task_1_debug = F_task[1];
    F_task_2_debug = F_task[2];

    for (int wheel = 0; wheel < 4; ++wheel) {
        float tau = 0.0f;
        for (int row = 0; row < 3; ++row) {
            tau += j1_pinv_[wheel][row] * F_task[row];
        }
        tau = std::clamp(tau, -control_config::kWheelTorqueFfLimitNm, control_config::kWheelTorqueFfLimitNm);
        wheel_torque_ff_output_ports_[wheel] = tau;
    }
    torque_ff_debug_0 = wheel_torque_ff_output_ports_[0].present().value_or(0.0f);
    torque_ff_debug_1 = wheel_torque_ff_output_ports_[1].present().value_or(0.0f);
}

void ChassisController::reset() {
    yaw_leso_[0] = 0.0f;
    yaw_leso_[1] = 0.0f;
    vx_pid_.reset();
    vy_pid_.reset();
    for (int i = 0; i < 4; ++i) {
        wheel_torque_ff_output_ports_[i] = 0.0f;
    }
    last_chassis_vx_m_s_ = 0.0f;
    last_chassis_vy_m_s_ = 0.0f;
    last_chassis_omega_z_rad_s_ = 0.0f;
    last_chassis_yaw_rad_ = 0.0f;
    omega_z_filter_.reset(0.0f);
}

bool ChassisController::inverse3x3(const float in[3][3], float out[3][3]) const {
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

void ChassisController::precompute_mappings() {
    const float r = control_config::kWheelRadiusM;
    const float l = control_config::kWheelCenterDistanceM;
    const float d = control_config::kCenterToComDistanceM;
    const float alpha = control_config::kWheelAlphaRad;
    const float beta = control_config::kWheelBetaRad;

    const float ca = std::cos(alpha);
    const float sa = std::sin(alpha);
    const float cb = std::cos(beta);
    const float sb = std::sin(beta);
    const float inv_r = (r > 1e-9f) ? (1.0f / r) : 0.0f;

    const float j1[3][4] = {
        {inv_r * ca, inv_r * (-ca), inv_r * (-cb), inv_r * cb},
        {inv_r * (-sa), inv_r * (-sa), inv_r * sb, inv_r * sb},
        {inv_r * (-l - d * sa), inv_r * (-l - d * sa), inv_r * (-l + d * sb), inv_r * (-l + d * sb)},
    };

    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 4; ++col) {
            j1_[row][col] = j1[row][col];
        }
    }

    float j1_j1t[3][3] = {{0.0f}};
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            for (int k = 0; k < 4; ++k) {
                j1_j1t[row][col] += j1[row][k] * j1[col][k];
            }
        }
    }

    float j1_j1t_inv[3][3] = {{0.0f}};
    if (inverse3x3(j1_j1t, j1_j1t_inv)) {
        for (int wheel = 0; wheel < 4; ++wheel) {
            for (int row = 0; row < 3; ++row) {
                float acc = 0.0f;
                for (int k = 0; k < 3; ++k) {
                    acc += j1[k][wheel] * j1_j1t_inv[k][row];
                }
                j1_pinv_[wheel][row] = acc;
            }
        }
    }
}
