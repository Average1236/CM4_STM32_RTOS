#include "wheel_motor.hpp"
#include "control_params.hpp"
#include <cmath>
#include <algorithm>
#include <cstring>

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

MotorDMH3510::MotorDMH3510(const Config_t& config) : config_(config) {
    update_wheel_speed_pll_gains();
}

void MotorDMH3510::pack_torque_data(float torque, uint8_t* tx_data) {
    const float torque_cmd = std::clamp(torque, -parameter_.tmax, parameter_.tmax);
    const uint16_t pos_tmp = float_to_uint(0, -parameter_.pmax, parameter_.pmax, 16);
    const uint16_t vel_tmp = float_to_uint(0, -parameter_.vmax, parameter_.vmax, 12);
    const uint16_t kp_tmp = 0;
    const uint16_t kd_tmp = 0;
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

float MotorDMH3510::uint_to_float(int x_int, float x_min, float x_max, int bits) {
    const float span = x_max - x_min;
    if (span == 0.0f) {
        return x_min;
    }
    return static_cast<float>(x_int) * span / static_cast<float>((1 << bits) - 1) + x_min;
}

float MotorDMH3510::wrap_pm_pi(float angle_rad) {
    while (angle_rad > control_config::kPi) {
        angle_rad -= 2.0f * control_config::kPi;
    }
    while (angle_rad < -control_config::kPi) {
        angle_rad += 2.0f * control_config::kPi;
    }
    return angle_rad;
}

void MotorDMH3510::update_wheel_speed_pll_gains() {
    pll_kp_ = 2.0f * control_config::kWheelSpeedPllBandwidth;
    pll_ki_ = 0.25f * pll_kp_ * pll_kp_;

    const float dt = control_config::kControlDtSec;
    if (dt > 0.f && dt * pll_kp_ >= 1.f) {
        pll_kp_ = 0.95f / dt;
        pll_ki_ = 0.25f * pll_kp_ * pll_kp_;
    }
}

void MotorDMH3510::reset_wheel_speed_pll(float measured_pos_rad, bool mark_prev_enabled) {
    pll_pos_est_rad_ = measured_pos_rad;
    pll_vel_est_rad_s_ = 0.0f;
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
        pll_initialized_ = true;
    }

    const float pos_pred = pll_pos_est_rad_ + dt * pll_vel_est_rad_s_;
    const float pos_err = wrap_pm_pi(measured_pos_rad - pos_pred);

    pll_pos_est_rad_ = pos_pred + dt * pll_kp_ * pos_err;
    pll_vel_est_rad_s_ += dt * pll_ki_ * pos_err;

    const float zero_snap_eps_rad_s = control_config::kWheelSpeedPllZeroSnapEpsRpm * control_config::kPi / 30.0f;
    if (std::fabs(pll_vel_est_rad_s_) < zero_snap_eps_rad_s) {
        pll_vel_est_rad_s_ = 0.0f;
    }

    pll_prev_enabled_ = true;
}

void MotorDMH3510::parse_feedback_data(const uint8_t rx_data[8]) {
    const uint8_t state = (rx_data[0] & 0xf0) >> 4;
    const uint16_t position = (rx_data[1] << 8) | rx_data[2];
    const float measured_pos_rad = config_.direction * uint_to_float(position, -parameter_.pmax, parameter_.pmax, 16);
    enabled_ = state == 1;
    update_wheel_speed_pll_from_pos(measured_pos_rad, enabled_);
    velocity_rpm_ = enabled_ ? pll_vel_est_rad_s_ * 30.f / control_config::kPi : 0.f;
}

void MotorDMH3510::build_set_torque_mode_msg(can_Message_t& msg) {
    reset_wheel_speed_pll(0.f, false);
    torque_mode_configured_ = true;
    build_write_register_msg(0x0a, 1, msg);
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

void MotorDMH3510::build_set_pmax_msg(float pmax, can_Message_t& msg) {
    uint32_t data = 0;
    std::memcpy(&data, &pmax, sizeof(float));
    const_cast<MotorDMH3510*>(this)->build_write_register_msg(0x15, data, msg);
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
    pending_write_register_rid_ = rid;
    pending_write_register_data_ = data;
    writing_register_ = true;
}

MotorDMH3510::RegisterWriteCheckResult MotorDMH3510::check_write_register_feedback(const can_Message_t& msg) {
    if (!writing_register_ || msg.len != 8) {
        return kRegisterWriteNotAck;
    }

    const uint16_t esc_id = static_cast<uint16_t>(msg.buf[0]) |
                            (static_cast<uint16_t>(msg.buf[1]) << 8);
    if (esc_id != static_cast<uint16_t>(config_.control_id) || msg.buf[2] != 0x55) {
        return kRegisterWriteNotAck;
    }

    last_write_register_rid_ = msg.buf[3];
    last_write_register_data_ = static_cast<uint32_t>(msg.buf[4]) |
                                (static_cast<uint32_t>(msg.buf[5]) << 8) |
                                (static_cast<uint32_t>(msg.buf[6]) << 16) |
                                (static_cast<uint32_t>(msg.buf[7]) << 24);
    writing_register_ = false;

    if (last_write_register_rid_ == pending_write_register_rid_ &&
        last_write_register_data_ == pending_write_register_data_) {
        return kRegisterWriteOk;
    }
    return kRegisterWriteMismatch;
}
