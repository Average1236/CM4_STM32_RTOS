#include "wheel_motor.hpp"
#include "control_params.hpp"
#include <cmath>
#include <cstring>

// Debug
volatile float torque_cmd_debug = 0;
volatile float torque_damp_debug = 0;
volatile float wheel_vel_raw_debug = 0;
volatile float wheel_integ_debug = 0;
volatile float wheel_diff_debug = 0;
volatile float torque_fb_debug = 0;
volatile float pos_fb_debug = 0;
volatile float pll_vel_est_debug = 0;
volatile float wheel_obs_vel_debug = 0;
volatile float wheel_obs_vel_filt_debug = 0;
volatile float wheel_obs_t_dist_debug = 0;

namespace {

uint16_t float_to_uint(float x, float x_min, float x_max, int bits) {
    const float span = x_max - x_min;
    if (span == 0.0f || bits <= 0 || bits > 16) {
        return 0;
    }

    const float clamped = (x < x_min) ? x_min : ((x > x_max) ? x_max : x);
    const uint32_t max_int = (1u << static_cast<uint32_t>(bits)) - 1u;
    const float scaled = (clamped - x_min) * static_cast<float>(max_int) / span;
    return static_cast<uint16_t>(scaled);
}

} // namespace

static const PID::Parameter_t kWheelSpeedPidParam = {
    .kp = control_config::kWheelSpeedPidKp,
    .ki = control_config::kWheelSpeedPidKi,
    .kd = control_config::kWheelSpeedPidKd,
    .output_limit = control_config::kWheelSpeedPidOutputLimitNm,
    .integ_limit = control_config::kWheelSpeedPidIntegLimitNm,
    .dt = control_config::kControlDtSec,
    .back_calc_gain = control_config::kWheelSpeedPidBackCalcGain,
    .diff_cutoff_hz = control_config::kWheelSpeedPidDiffCutoffHz,
};

static constexpr float kWheelSpeedPidKpRampTimeSec = control_config::kWheelSpeedPidKpRampTimeSec;
static constexpr float kWheelSpeedPidKpRampStep =
    (kWheelSpeedPidKpRampTimeSec > 1e-6f) ? (control_config::kControlDtSec / kWheelSpeedPidKpRampTimeSec) : 1.0f;
static constexpr float kWheelSpeedPllOmegaRampTimeSec = control_config::kWheelSpeedPllOmegaRampTimeSec;
static constexpr float kWheelSpeedPllOmegaRampStep =
    (kWheelSpeedPllOmegaRampTimeSec > 1e-6f) ? (control_config::kControlDtSec / kWheelSpeedPllOmegaRampTimeSec) : 1.0f;

// Luenberger observer gains — compile-time computation, zero runtime cost.
static constexpr float kObsDt = control_config::kControlDtSec;
static constexpr float kObsWo = control_config::kWheelObsVelocityBandwidth;
static constexpr float kObsL1 = 3.0f * kObsWo * kObsDt;
static constexpr float kObsL2 = 3.0f * kObsWo * kObsWo * kObsDt;
static constexpr float kObsL3 = control_config::kWheelInertiaKgM2 * kObsWo * kObsWo * kObsWo * kObsDt;
static constexpr float kObsInvJ = 1.0f / control_config::kWheelInertiaKgM2;
static constexpr float kObsBOverJ = control_config::kWheelViscousDampingNmPerRadPS / control_config::kWheelInertiaKgM2;

static const ButterworthLowPass2::Parameter_t kObsVelFilterParam = {
    .cutoff_hz = control_config::kWheelObsVelocityButterworthCutoffHz,
    .dt = control_config::kControlDtSec,
};

const WheelMotorBase::Info_t motor_info_DMH3510{
    .motor_current_limit = 3.2,
    .motor_torque_limit = 0.45,
    .broad_current_limit = 2,
    .raw_reduce_rate = 1,
};

WheelMotorBase::WheelMotorBase(Type type, const Config_t& config, const Info_t& info)
    : type_(type), info_(info), config_(config) {}

void WheelMotorBase::publish_feedback_ports() {
    angle_output_port_ = angle_;
    velocity_output_port_ = vel_;
    torque_output_port_ = torque_;
    current_output_port_ = current_;
}

void WheelMotorBase::reset_ports() {
    angle_output_port_.reset();
    velocity_output_port_.reset();
    torque_output_port_.reset();
    current_output_port_.reset();
    torque_cmd_output_port_.reset();
}

MotorDMH3510::MotorDMH3510(const Config_t& config)
        : WheelMotorBase(kTypeDMH3510, config, motor_info_DMH3510),
            wheel_speed_pid_(kWheelSpeedPidParam),
            obs_vel_filter_(kObsVelFilterParam, 0.0f) {
    update_wheel_speed_pll_gains();
}

void MotorDMH3510::pack_velocity_data(float velocity, uint8_t* tx_data) {
    writing_register_ = false;

    std::memcpy(tx_data, &velocity, sizeof(float));
}

void MotorDMH3510::pack_mit_data(float position, float velocity, float kp, float kd, float torque_ff, uint8_t* tx_data) {
    static constexpr float kMITKpMin = 0.0f;
    static constexpr float kMITKpMax = 500.0f;
    static constexpr float kMITKdMin = 0.0f;
    static constexpr float kMITKdMax = 5.0f;

    update_wheel_speed_pid_kp_ramp();

    // velocity uses rad/s in MIT command; convert feedback from rpm to rad/s.
    const float velocity_feedback = vel_ * kPi / 30.0f;
    const float torque_pid = wheel_speed_pid_.calc(velocity, velocity_feedback);
    if (config_.feedback_id == 1) {
        wheel_integ_debug = wheel_speed_pid_.get_integ();
        wheel_diff_debug = wheel_speed_pid_.get_diff();
    }

    // --- 3-state Luenberger observer ---
    const float dt = control_config::kControlDtSec;
    const float theta_meas_rad = angle_ * kPi / 180.0f;  // angle_ single-turn deg → rad

    if (!enabled_) {
        obs_initialized_ = false;
        obs_theta_rad_ = theta_meas_rad;
        obs_omega_rad_s_ = 0.0f;
        obs_t_dist_nm_ = 0.0f;
    } else {
        if (!obs_initialized_) {
            obs_theta_rad_ = theta_meas_rad;
            obs_omega_rad_s_ = 0.0f;
            obs_t_dist_nm_ = 0.0f;
            obs_initialized_ = true;
        }

        // Predict: torque feedforward + friction, semi-implicit Euler
        const float omega_pred = obs_omega_rad_s_
            + (last_torque_cmd_nm_ + obs_t_dist_nm_) * kObsInvJ * dt
            - kObsBOverJ * obs_omega_rad_s_ * dt;
        const float theta_pred = obs_theta_rad_ + omega_pred * dt;

        // Correct from single-turn position; wrap is correct because
        // per-step θ increment << π at 1kHz control rate.
        const float theta_err = wrap_pm_pi(theta_meas_rad - theta_pred);
        obs_theta_rad_ = theta_pred + kObsL1 * theta_err;
        obs_omega_rad_s_ = omega_pred + kObsL2 * theta_err;
        obs_t_dist_nm_ += kObsL3 * theta_err;

        if (obs_t_dist_nm_ > parameter_.tmax) {
            obs_t_dist_nm_ = parameter_.tmax;
        } else if (obs_t_dist_nm_ < -parameter_.tmax) {
            obs_t_dist_nm_ = -parameter_.tmax;
        }
    }

    // --- Virtual velocity damping (filter velocity, not output) ---
    const float obs_vel_filtered = obs_vel_filter_.filter(obs_omega_rad_s_);
    const float torque_damp = enabled_
        ? (control_config::kWheelVirtualDampingNmPerRadPS * (velocity - obs_vel_filtered))
        : 0.0f;

    if (config_.feedback_id == 1) {
        wheel_obs_vel_debug = obs_omega_rad_s_;
        wheel_obs_vel_filt_debug = obs_vel_filtered;
        wheel_obs_t_dist_debug = obs_t_dist_nm_;
    }

    float torque_cmd = torque_ff + torque_pid + torque_damp;
    if (torque_cmd > parameter_.tmax) {
        torque_cmd = parameter_.tmax;
    } else if (torque_cmd < -parameter_.tmax) {
        torque_cmd = -parameter_.tmax;
    }

    last_torque_cmd_nm_ = torque_cmd;
    torque_cmd_output_port_ = torque_cmd;

    if (config_.feedback_id == 1) {
        torque_cmd_debug = torque_cmd;
        torque_damp_debug = torque_damp;
    }

    const uint16_t pos_tmp = float_to_uint(position, -parameter_.pmax, parameter_.pmax, 16);
    const uint16_t vel_tmp = float_to_uint(velocity, -parameter_.vmax, parameter_.vmax, 12);
    const uint16_t kp_tmp = float_to_uint(kp, kMITKpMin, kMITKpMax, 12);
    const uint16_t kd_tmp = float_to_uint(kd, kMITKdMin, kMITKdMax, 12);
    const uint16_t tor_tmp = float_to_uint(torque_cmd, -parameter_.tmax, parameter_.tmax, 12);

    tx_data[0] = static_cast<uint8_t>((pos_tmp >> 8) & 0xFF);
    tx_data[1] = static_cast<uint8_t>(pos_tmp & 0xFF);
    tx_data[2] = static_cast<uint8_t>((vel_tmp >> 4) & 0xFF);
    tx_data[3] = static_cast<uint8_t>(((vel_tmp & 0x0F) << 4) | ((kp_tmp >> 8) & 0x0F));
    tx_data[4] = static_cast<uint8_t>(kp_tmp & 0xFF);
    tx_data[5] = static_cast<uint8_t>((kd_tmp >> 4) & 0xFF);
    tx_data[6] = static_cast<uint8_t>(((kd_tmp & 0x0F) << 4) | ((tor_tmp >> 8) & 0x0F));
    tx_data[7] = static_cast<uint8_t>(tor_tmp & 0xFF);

    writing_register_ = false;
}

void MotorDMH3510::update_wheel_speed_pid_kp_ramp() {
    if (!enabled_) {
        wheel_speed_pid_prev_enabled_ = false;
        wheel_speed_pid_kp_alpha_ = 0.0f;
    } else if (!wheel_speed_pid_prev_enabled_) {
        // Rising edge of enable: restart wheel-speed PID Kp from zero.
        wheel_speed_pid_prev_enabled_ = true;
        wheel_speed_pid_kp_alpha_ = 0.0f;
    }

    PID::Parameter_t param = kWheelSpeedPidParam;
    param.kp = kWheelSpeedPidParam.kp * wheel_speed_pid_kp_alpha_;
    wheel_speed_pid_.set_parameter(param);

    if (enabled_) {
        wheel_speed_pid_kp_alpha_ += kWheelSpeedPidKpRampStep;
        if (wheel_speed_pid_kp_alpha_ > 1.0f) {
            wheel_speed_pid_kp_alpha_ = 1.0f;
        }
    }
}

float MotorDMH3510::uint_to_float(int x_int, float x_min, float x_max, int bits) {
    const float span = x_max - x_min;
    if (span == 0.0f) {
        return x_min;
    }
    return static_cast<float>(x_int) * span / static_cast<float>((1 << bits) - 1) + x_min;
}

float MotorDMH3510::wrap_pm_pi(float angle_rad) {
    while (angle_rad > kPi) {
        angle_rad -= 2.0f * kPi;
    }
    while (angle_rad < -kPi) {
        angle_rad += 2.0f * kPi;
    }
    return angle_rad;
}

void MotorDMH3510::update_wheel_speed_pll_gains() {
    pll_kp_ = 2.0f * control_config::kWheelSpeedPllBandwidth;
    pll_ki_ = 0.25f * pll_kp_ * pll_kp_;

    const float dt = control_config::kControlDtSec;
    pll_gain_unstable_ = !(dt * pll_kp_ < 1.0f);
    if (pll_gain_unstable_ && dt > 0.0f) {
        pll_kp_ = 0.95f / dt;
        pll_ki_ = 0.25f * pll_kp_ * pll_kp_;
    }
}

void MotorDMH3510::reset_wheel_speed_pll(float measured_pos_rad, bool mark_prev_enabled) {
    pll_pos_est_rad_ = measured_pos_rad;
    pll_vel_est_rad_s_ = 0.0f;
    pll_vel_ramp_alpha_ = 0.0f;
    pll_initialized_ = false;
    pll_prev_enabled_ = mark_prev_enabled;
}

void MotorDMH3510::update_wheel_speed_pll_from_pos(float measured_pos_rad, bool enabled_now) {
    const float dt = control_config::kControlDtSec;
    if (dt <= 0.0f) {
        pll_vel_est_rad_s_ = 0.0f;
        return;
    }

    if (!enabled_now) {
        reset_wheel_speed_pll(measured_pos_rad, false);
        return;
    }

    if (!pll_initialized_ || !pll_prev_enabled_) {
        pll_pos_est_rad_ = measured_pos_rad;
        pll_vel_est_rad_s_ = 0.0f;
        pll_vel_ramp_alpha_ = 0.0f;
        pll_initialized_ = true;
    }

    const float pos_pred = pll_pos_est_rad_ + dt * pll_vel_est_rad_s_;
    const float pos_err = wrap_pm_pi(measured_pos_rad - pos_pred);

    pll_pos_est_rad_ = pos_pred + dt * pll_kp_ * pos_err;
    pll_vel_est_rad_s_ += dt * pll_ki_ * pos_err;

    const float zero_snap_eps_rad_s = control_config::kWheelSpeedPllZeroSnapEpsRpm * kPi / 30.0f;
    if (std::fabs(pll_vel_est_rad_s_) < zero_snap_eps_rad_s) {
        pll_vel_est_rad_s_ = 0.0f;
    }

    if (config_.feedback_id == 1) {
        pll_vel_est_debug = pll_vel_est_rad_s_;
    }

    pll_vel_ramp_alpha_ += kWheelSpeedPllOmegaRampStep;
    if (pll_vel_ramp_alpha_ > 1.0f) {
        pll_vel_ramp_alpha_ = 1.0f;
    }

    pll_prev_enabled_ = true;
}

void MotorDMH3510::parse_feedback_data(const uint8_t rx_data[8]) {

    state_ = static_cast<State>((rx_data[0] & 0xF0) >> 4);

    const uint16_t pos_uint = (rx_data[1] << 8) | rx_data[2];
    const uint16_t vel_uint = (rx_data[3] << 4) | (rx_data[4] >> 4);
    const uint16_t tor_uint = ((rx_data[4] & 0x0F) << 8) | rx_data[5];

    const float pos = uint_to_float(pos_uint, -parameter_.pmax, parameter_.pmax, 16);
    const float vel = uint_to_float(vel_uint, -parameter_.vmax, parameter_.vmax, 12);
    torque_ = uint_to_float(tor_uint, -parameter_.tmax, parameter_.tmax, 12) * config_.direction;

    const float measured_pos_rad = config_.direction * pos;
    angle_ = measured_pos_rad * 180.0f / kPi;
    if (config_.feedback_id == 1) {
        wheel_vel_raw_debug = config_.direction * vel;
        torque_fb_debug = torque_;
        pos_fb_debug = measured_pos_rad;
    }

    enabled_ = (state_ == kStateMotorEnable);
    // last_error_ = (state_ != kStateMotorEnable) ? state_ : last_error_;
    if (!enabled_) {
        last_error_ = state_;
    }
    update_wheel_speed_pll_from_pos(measured_pos_rad, enabled_);
    vel_ = enabled_ ? (pll_vel_est_rad_s_ * 30.0f / kPi * pll_vel_ramp_alpha_) : 0.0f;
    publish_feedback_ports();
}

void MotorDMH3510::build_set_mode_msg(Mode mode, can_Message_t& msg) {
    if (mode < kModeMITControl || mode > kModeMixedControl) {
        return;
    }
    if (mode != kModeMITControl) {
        reset_wheel_speed_pid();
    }
    reset_wheel_speed_pll(0.0f, false);
    mode_ = static_cast<Mode>(mode);
    build_write_register_msg(0x0A, static_cast<uint32_t>(mode), msg);
}

void MotorDMH3510::build_enable_msg(can_Message_t& msg) {
    writing_register_ = false;

    msg.id = config_.control_id;
    msg.isExt = false;
    msg.rtr = false;
    msg.len = 8;
    msg.buf[0] = 0xFF;
    msg.buf[1] = 0xFF;
    msg.buf[2] = 0xFF;
    msg.buf[3] = 0xFF;
    msg.buf[4] = 0xFF;
    msg.buf[5] = 0xFF;
    msg.buf[6] = 0xFF;
    msg.buf[7] = 0xFC;
}

void MotorDMH3510::build_disable_msg(can_Message_t& msg) {
    writing_register_ = false;
    reset_wheel_speed_pid();
    reset_wheel_speed_pll(0.0f, false);

    msg.id = config_.control_id;
    msg.isExt = false;
    msg.rtr = false;
    msg.len = 8;
    msg.buf[0] = 0xFF;
    msg.buf[1] = 0xFF;
    msg.buf[2] = 0xFF;
    msg.buf[3] = 0xFF;
    msg.buf[4] = 0xFF;
    msg.buf[5] = 0xFF;
    msg.buf[6] = 0xFF;
    msg.buf[7] = 0xFD;
}

void MotorDMH3510::reset_wheel_speed_pid() {
    wheel_speed_pid_.reset();
    wheel_speed_pid_kp_alpha_ = 0.0f;
    wheel_speed_pid_prev_enabled_ = false;
    obs_initialized_ = false;
    obs_theta_rad_ = 0.0f;
    obs_omega_rad_s_ = 0.0f;
    obs_t_dist_nm_ = 0.0f;
    last_torque_cmd_nm_ = 0.0f;
    obs_vel_filter_.reset(0.0f);
}

void MotorDMH3510::build_clear_error_msg(can_Message_t& msg) {
    writing_register_ = false;
    reset_wheel_speed_pll(0.0f, false);

    msg.id = config_.control_id;
    msg.isExt = false;
    msg.rtr = false;
    msg.len = 8;
    msg.buf[0] = 0xFF;
    msg.buf[1] = 0xFF;
    msg.buf[2] = 0xFF;
    msg.buf[3] = 0xFF;
    msg.buf[4] = 0xFF;
    msg.buf[5] = 0xFF;
    msg.buf[6] = 0xFF;
    msg.buf[7] = 0xFB;
}

void MotorDMH3510::build_save_zero_msg(can_Message_t& msg) {
    writing_register_ = false;

    msg.id = config_.control_id;
    msg.isExt = false;
    msg.rtr = false;
    msg.len = 8;
    msg.buf[0] = 0xFF;
    msg.buf[1] = 0xFF;
    msg.buf[2] = 0xFF;
    msg.buf[3] = 0xFF;
    msg.buf[4] = 0xFF;
    msg.buf[5] = 0xFF;
    msg.buf[6] = 0xFF;
    msg.buf[7] = 0xFE;
}

void MotorDMH3510::build_set_acc_msg(float acc, can_Message_t& msg) {
    uint32_t data = 0;
    std::memcpy(&data, &acc, sizeof(float));
    const_cast<MotorDMH3510*>(this)->build_write_register_msg(kRegACC, data, msg);
}

void MotorDMH3510::build_set_dec_msg(float dec, can_Message_t& msg) {
    uint32_t data = 0;
    std::memcpy(&data, &dec, sizeof(float));
    const_cast<MotorDMH3510*>(this)->build_write_register_msg(kRegDEC, data, msg);
}

void MotorDMH3510::build_set_velocity_kp_msg(float kp, can_Message_t& msg) {
    uint32_t data = 0;
    std::memcpy(&data, &kp, sizeof(float));
    const_cast<MotorDMH3510*>(this)->build_write_register_msg(kRegKP_ASR, data, msg);
}

void MotorDMH3510::build_set_velocity_ki_msg(float ki, can_Message_t& msg) {
    uint32_t data = 0;
    std::memcpy(&data, &ki, sizeof(float));
    const_cast<MotorDMH3510*>(this)->build_write_register_msg(kRegKI_ASR, data, msg);
}

void MotorDMH3510::build_set_pmax_msg(float pmax, can_Message_t& msg) {
    uint32_t data = 0;
    std::memcpy(&data, &pmax, sizeof(float));
    const_cast<MotorDMH3510*>(this)->build_write_register_msg(kRegPMAX, data, msg);
}

void MotorDMH3510::build_set_vmax_msg(float vmax, can_Message_t& msg) {
    uint32_t data = 0;
    std::memcpy(&data, &vmax, sizeof(float));
    const_cast<MotorDMH3510*>(this)->build_write_register_msg(kRegVMAX, data, msg);
}

void MotorDMH3510::build_set_tmax_msg(float tmax, can_Message_t& msg) {
    uint32_t data = 0;
    std::memcpy(&data, &tmax, sizeof(float));
    const_cast<MotorDMH3510*>(this)->build_write_register_msg(kRegTMAX, data, msg);
}

void MotorDMH3510::build_write_register_msg(uint8_t rid, uint32_t data, can_Message_t& msg) {
    const uint16_t esc_id = static_cast<uint16_t>(config_.control_id);
    msg.id = 0x7FF;
    msg.isExt = false;
    msg.rtr = false;
    msg.len = 8;
    msg.buf[0] = static_cast<uint8_t>(esc_id & 0xFF);
    msg.buf[1] = static_cast<uint8_t>((esc_id >> 8) & 0xFF);
    msg.buf[2] = 0x55;
    msg.buf[3] = rid;
    msg.buf[4] = static_cast<uint8_t>(data & 0xFF);
    msg.buf[5] = static_cast<uint8_t>((data >> 8) & 0xFF);
    msg.buf[6] = static_cast<uint8_t>((data >> 16) & 0xFF);
    msg.buf[7] = static_cast<uint8_t>((data >> 24) & 0xFF);
    writing_register_ = true;
}

uint32_t MotorDMH3510::command_can_id() const {
    switch (mode_)
    {
    case kModeMITControl:
        return static_cast<uint32_t>(config_.control_id);
        break;

    case kModePositionVelocityControl:
        return static_cast<uint32_t>(config_.control_id) + 0x100;
        break;

    case kModeVelocityControl:
        return static_cast<uint32_t>(config_.control_id) + 0x200;
        break;

    case kModeMixedControl:
        return static_cast<uint32_t>(config_.control_id) + 0x300;
        break;
    
    default:
        break;
    }

    return static_cast<uint32_t>(config_.control_id);
}


