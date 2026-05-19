#include "chassis_controller.hpp"
#include "control_params.hpp"
#include <algorithm>
#include <cmath>

volatile float yaw_ref_input_debug = 0;
volatile float vx_obs_z1_debug = 0;
volatile float vx_obs_z2_debug = 0;
volatile float yaw_obs_z1_debug = 0;
volatile float yaw_obs_z2_debug = 0;
volatile float f_task_0_debug = 0;
volatile float f_task_1_debug = 0;
volatile float f_task_2_debug = 0;
volatile float torque_ff_debug_0 = 0;
volatile float torque_ff_debug_1 = 0;

MixedLesoChassisController::MixedLesoChassisController() {
    precompute_mappings();
}

void MixedLesoChassisController::set_reference(const float vel_ref[3], const float acc_ref[3], float yaw_ref_rel_rad) {
    for (int i = 0; i < 3; ++i) {
        vel_ref_[i] = vel_ref[i];
        acc_ref_[i] = acc_ref[i];
    }
    (void)yaw_ref_rel_rad;
}

void MixedLesoChassisController::step(float dt_s) {
    if (dt_s <= 0.0f) {
        return;
    }

    const std::optional<float> chassis_vx_meas = chassis_vx_input_port_.any();
    const std::optional<float> chassis_vy_meas = chassis_vy_input_port_.any();
    const std::optional<float> omega_z_meas = chassis_omega_z_input_port_.any();

    const float vx_m_s = chassis_vx_meas.has_value() ? *chassis_vx_meas : last_chassis_vx_m_s_;
    const float vy_m_s = chassis_vy_meas.has_value() ? *chassis_vy_meas : last_chassis_vy_m_s_;
    const float omega_z_rad_s = omega_z_meas.has_value() ? *omega_z_meas : last_chassis_omega_z_rad_s_;

    last_chassis_vx_m_s_ = vx_m_s;
    last_chassis_vy_m_s_ = vy_m_s;
    last_chassis_omega_z_rad_s_ = omega_z_rad_s;

    // Read actual sent wheel torques from previous control cycle
    float tau_sent[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    for (int i = 0; i < 4; ++i) {
        const std::optional<float> tau = wheel_sent_torque_input_ports_[i].any();
        tau_sent[i] = tau.has_value() ? *tau : 0.0f;
    }

    // Map wheel torques → body wrench via forward Jacobian
    float u_body[3] = {0.0f, 0.0f, 0.0f};
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 4; ++col) {
            u_body[row] += j1_[row][col] * tau_sent[col];
        }
    }
    const float u_vx_actual = u_body[0] / control_config::kRobotMassKg;
    const float u_vy_actual = u_body[1] / control_config::kRobotMassKg;
    const float u_wz_actual = u_body[2] / control_config::kRobotInertiaKgM2;

    const float w_vel = control_config::kLesoVelObserverBandwidth;
    const float l1 = 2.0f * w_vel;
    const float l2 = w_vel * w_vel;

    const float w_omega = control_config::kLesoOmegaObserverBandwidth;
    const float l1_omega = 2.0f * w_omega;
    const float l2_omega = w_omega * w_omega;

    const float e_vx = vx_m_s - vel_obs_[0][0];
    const float z1_dot_vx = vel_obs_[0][1] + l1 * e_vx + u_vx_actual;
    const float z2_dot_vx = l2 * e_vx;
    vel_obs_[0][0] += dt_s * z1_dot_vx;
    vel_obs_[0][1] += dt_s * z2_dot_vx;

    vx_obs_z1_debug = vel_obs_[0][0];
    vx_obs_z2_debug = vel_obs_[0][1];

    const float e_vy = vy_m_s - vel_obs_[1][0];
    const float z1_dot_vy = vel_obs_[1][1] + l1 * e_vy + u_vy_actual;
    const float z2_dot_vy = l2 * e_vy;
    vel_obs_[1][0] += dt_s * z1_dot_vy;
    vel_obs_[1][1] += dt_s * z2_dot_vy;

    const float e_wz = omega_z_rad_s - vel_obs_[2][0];
    const float z1_dot_wz = vel_obs_[2][1] + l1_omega * e_wz + u_wz_actual;
    const float z2_dot_wz = l2_omega * e_wz;
    vel_obs_[2][0] += dt_s * z1_dot_wz;
    vel_obs_[2][1] += dt_s * z2_dot_wz;

    yaw_obs_z1_debug = vel_obs_[2][0];
    yaw_obs_z2_debug = vel_obs_[2][1];

    const float vx_feedback = control_config::kVelFeedbackGainX * (vel_ref_[0] - vel_obs_[0][0]);
    const float vy_feedback = control_config::kVelFeedbackGainY * (vel_ref_[1] - vel_obs_[1][0]);
    const float yaw_feedback = control_config::kVelFeedbackGainYaw * (vel_ref_[2] - vel_obs_[2][0]);
    yaw_ref_input_debug = yaw_feedback;

    const float f_task[3] = {
        control_config::kRobotMassKg *
            (acc_ref_[0] + vx_feedback - vel_obs_[0][1]),
        control_config::kRobotMassKg *
            (acc_ref_[1] + vy_feedback - vel_obs_[1][1]),
        control_config::kRobotInertiaKgM2 *
            (acc_ref_[2] + yaw_feedback - vel_obs_[2][1]),
    };

    f_task_0_debug = f_task[0];
    f_task_1_debug = f_task[1];
    f_task_2_debug = f_task[2];

    for (int wheel = 0; wheel < 4; ++wheel) {
        float tau = 0.0f;
        for (int row = 0; row < 3; ++row) {
            tau += j1_pinv_[wheel][row] * f_task[row];
        }
        tau = std::clamp(tau, -control_config::kWheelTorqueFfLimitNm, control_config::kWheelTorqueFfLimitNm);
        wheel_torque_ff_output_ports_[wheel] = tau;
    }
    torque_ff_debug_0 = wheel_torque_ff_output_ports_[0].present().value_or(0.0f);
    torque_ff_debug_1 = wheel_torque_ff_output_ports_[1].present().value_or(0.0f);
}

void MixedLesoChassisController::reset() {
    for (int axis = 0; axis < 3; ++axis) {
        vel_obs_[axis][0] = 0.0f;
        vel_obs_[axis][1] = 0.0f;
    }

    for (int i = 0; i < 4; ++i) {
        wheel_torque_ff_output_ports_[i] = 0.0f;
    }
    last_chassis_vx_m_s_ = 0.0f;
    last_chassis_vy_m_s_ = 0.0f;
    last_chassis_omega_z_rad_s_ = 0.0f;
}

bool MixedLesoChassisController::inverse3x3(const float in[3][3], float out[3][3]) const {
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

void MixedLesoChassisController::precompute_mappings() {
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
