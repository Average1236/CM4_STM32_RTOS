#include "chassis_controller.hpp"
#include "control_params.hpp"
#include <algorithm>
#include <cmath>

volatile float yaw_ref_input_debug = 0;
volatile float vx_leso_z1_debug = 0;
volatile float vx_leso_z2_debug = 0;
volatile float yaw_leso_z1_debug = 0;
volatile float yaw_leso_z2_debug = 0;
volatile float yaw_leso3_z3_debug = 0;
volatile float F_task_0_debug = 0;
volatile float F_task_1_debug = 0;
volatile float F_task_2_debug = 0;
volatile float torque_ff_debug_0 = 0;
volatile float torque_ff_debug_1 = 0;
volatile float u_psi_debug = 0;
volatile float err_psi_debug = 0;
volatile float yaw_rad_debug = 0;
volatile float yaw_target_td_debug = 0;
volatile float yaw_target_td_diff_debug = 0;

MixedLesoChassisController::MixedLesoChassisController()
    : yaw_td_({control_config::kYawTDR, control_config::kYawTDH, control_config::kControlDtSec,
               true, -kPi, kPi},
              0.0f) {
    precompute_mappings();
}

void MixedLesoChassisController::set_reference(const float vel_ref[3], const float acc_ref[3], float yaw_ref_rel_rad) {
    for (int i = 0; i < 3; ++i) {
        vel_ref_[i] = vel_ref[i];
        acc_ref_[i] = acc_ref[i];
    }
    (void)yaw_ref_rel_rad;
}

void MixedLesoChassisController::set_use_3rd_order_leso(bool enable) {
    if (enable != use_3rd_order_leso_) {
        leso2_[2][0] = 0.0f;
        leso2_[2][1] = 0.0f;
        leso3_psi_[0] = 0.0f;
        leso3_psi_[1] = 0.0f;
        leso3_psi_[2] = 0.0f;
        yaw_target_rad_ = 0.0f;
        yaw_ramp_alpha_ = 0.0f;
        use_3rd_order_leso_ = enable;
    }
}

void MixedLesoChassisController::set_yaw_target(float target_rad) {
    yaw_target_rad_ = target_rad;
}

void MixedLesoChassisController::step(float dt_s) {
    if (dt_s <= 0.0f) {
        return;
    }

    const std::optional<float> chassis_vx_meas = chassis_vx_input_port_.any();
    const std::optional<float> chassis_vy_meas = chassis_vy_input_port_.any();
    const std::optional<float> omega_z_meas = chassis_omega_z_input_port_.any();
    const std::optional<float> yaw_meas = chassis_yaw_input_port_.any();

    const float vx_m_s = chassis_vx_meas.has_value() ? *chassis_vx_meas : last_chassis_vx_m_s_;
    const float vy_m_s = chassis_vy_meas.has_value() ? *chassis_vy_meas : last_chassis_vy_m_s_;
    const float omega_z_rad_s = omega_z_meas.has_value() ? *omega_z_meas : last_chassis_omega_z_rad_s_;
    const float yaw_rad = yaw_meas.has_value() ? *yaw_meas : last_chassis_yaw_rad_;

    // Startup ramp: gradually scale yaw_rad from 0 → actual to avoid
    // a step discontinuity when the 3rd-order LESO initializes.
    float yaw_rad_effective = yaw_rad;
    if (use_3rd_order_leso_ && yaw_ramp_alpha_ < 1.0f) {
        const float ramp_t = control_config::kYawStartupRampTimeSec;
        if (ramp_t > 1e-6f) {
            yaw_ramp_alpha_ += dt_s / ramp_t;
            if (yaw_ramp_alpha_ > 1.0f) yaw_ramp_alpha_ = 1.0f;
        } else {
            yaw_ramp_alpha_ = 1.0f;
        }
        yaw_rad_effective *= yaw_ramp_alpha_;
    }

    yaw_rad_debug = yaw_rad_effective;

    last_chassis_vx_m_s_ = vx_m_s;
    last_chassis_vy_m_s_ = vy_m_s;
    last_chassis_omega_z_rad_s_ = omega_z_rad_s;
    last_chassis_yaw_rad_ = yaw_rad;

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
    const float u_vx = u_body[0] / control_config::kRobotMassKg;
    const float u_vy = u_body[1] / control_config::kRobotMassKg;
    const float u_psi = u_body[2] / control_config::kRobotInertiaKgM2;

    u_psi_debug = u_psi;

    // ---- 2nd-order LESO: vx axis ----
    const float wo_vel = control_config::kLesoVelObserverBandwidth;
    const float L1_vel = 2.0f * wo_vel;
    const float L2_vel = wo_vel * wo_vel;

    const float err_vx = vx_m_s - leso2_[0][0];
    const float dz1_vx = leso2_[0][1] + L1_vel * err_vx + u_vx;
    const float dz2_vx = L2_vel * err_vx;
    leso2_[0][0] += dt_s * dz1_vx;
    leso2_[0][1] += dt_s * dz2_vx;

    vx_leso_z1_debug = leso2_[0][0];
    vx_leso_z2_debug = leso2_[0][1];

    // ---- 2nd-order LESO: vy axis ----
    const float err_vy = vy_m_s - leso2_[1][0];
    const float dz1_vy = leso2_[1][1] + L1_vel * err_vy + u_vy;
    const float dz2_vy = L2_vel * err_vy;
    leso2_[1][0] += dt_s * dz1_vy;
    leso2_[1][1] += dt_s * dz2_vy;

    // ---- yaw axis: 2nd-order or 3rd-order LESO ----
    float fb_psi;
    float F_task_psi;

    if (use_3rd_order_leso_) {
        const float wo_psi = control_config::kLeso3rdOrderBandwidth;
        const float L1_psi = 3.0f * wo_psi;
        const float L2_psi = 3.0f * wo_psi * wo_psi;
        const float L3_psi = wo_psi * wo_psi * wo_psi;

        const float err_psi = wrap_to_pi(yaw_rad_effective - leso3_psi_[0]);
        const float dz1_psi = leso3_psi_[1] + L1_psi * err_psi;
        const float dz2_psi = leso3_psi_[2] + L2_psi * err_psi + u_psi;
        const float dz3_psi = L3_psi * err_psi;

        err_psi_debug = err_psi;

        // leso3_psi_[0] += dt_s * dz1_psi;
        leso3_psi_[0] = wrap_to_pi(leso3_psi_[0] + dt_s * dz1_psi);
        leso3_psi_[1] += dt_s * dz2_psi;
        leso3_psi_[2] += dt_s * dz3_psi;

        yaw_leso_z1_debug = leso3_psi_[0];
        yaw_leso_z2_debug = leso3_psi_[1];
        yaw_leso3_z3_debug = leso3_psi_[2];

        yaw_td_.calc(yaw_target_rad_);
        const float err_pos = wrap_to_pi(yaw_td_.get_data() - leso3_psi_[0]);
        // const float err_vel = yaw_td_.get_diff() - leso3_psi_[1];
        const float err_vel = yaw_td_.get_diff() - omega_z_rad_s;
        yaw_target_td_debug = yaw_td_.get_data();
        yaw_target_td_diff_debug = yaw_td_.get_diff();
        fb_psi = control_config::kPosFeedbackGainYaw * err_pos
               + control_config::kYawTDDiffGain * err_vel;
        yaw_ref_input_debug = fb_psi;
        F_task_psi = control_config::kRobotInertiaKgM2 * (fb_psi - leso3_psi_[2]);
    } else {
        const float wo_omega = control_config::kLesoOmegaObserverBandwidth;
        const float L1_omega = 2.0f * wo_omega;
        const float L2_omega = wo_omega * wo_omega;

        const float err_wz = omega_z_rad_s - leso2_[2][0];
        const float dz1_wz = leso2_[2][1] + L1_omega * err_wz + u_psi;
        const float dz2_wz = L2_omega * err_wz;
        leso2_[2][0] += dt_s * dz1_wz;
        leso2_[2][1] += dt_s * dz2_wz;

        yaw_leso_z1_debug = leso2_[2][0];
        yaw_leso_z2_debug = leso2_[2][1];
        yaw_leso3_z3_debug = 0.0f;

        fb_psi = control_config::kVelFeedbackGainYaw * (vel_ref_[2] - leso2_[2][0]);
        yaw_ref_input_debug = fb_psi;
        F_task_psi = control_config::kRobotInertiaKgM2 * (acc_ref_[2] + fb_psi - leso2_[2][1]);
    }

    const float fb_vx = control_config::kVelFeedbackGainX * (vel_ref_[0] - leso2_[0][0]);
    const float fb_vy = control_config::kVelFeedbackGainY * (vel_ref_[1] - leso2_[1][0]);

    const float F_task[3] = {
        control_config::kRobotMassKg * (acc_ref_[0] + fb_vx - leso2_[0][1]),
        control_config::kRobotMassKg * (acc_ref_[1] + fb_vy - leso2_[1][1]),
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

void MixedLesoChassisController::reset() {
    for (int axis = 0; axis < 3; ++axis) {
        leso2_[axis][0] = 0.0f;
        leso2_[axis][1] = 0.0f;
    }
    leso3_psi_[0] = 0.0f;
    leso3_psi_[1] = 0.0f;
    leso3_psi_[2] = 0.0f;

    for (int i = 0; i < 4; ++i) {
        wheel_torque_ff_output_ports_[i] = 0.0f;
    }
    last_chassis_vx_m_s_ = 0.0f;
    last_chassis_vy_m_s_ = 0.0f;
    last_chassis_omega_z_rad_s_ = 0.0f;
    last_chassis_yaw_rad_ = 0.0f;
    yaw_ramp_alpha_ = 0.0f;
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
