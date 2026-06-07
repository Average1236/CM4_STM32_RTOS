#include "dribbler_zfoc.hpp"
#include "Task/z_main.h"
#include <cstring>

// Debug
volatile float infra_voltage_raw_debug = 0;
volatile float infra_voltage_filt_debug = 0;
volatile int current_state_debug = 0;
volatile int has_error_debug = 0;

void DribblerZfoc::parse_heartbeat(const can_Message_t& msg) {
    // Bytes [0:3]: axis.error_
    axis_error = static_cast<uint32_t>(msg.buf[0]) | (static_cast<uint32_t>(msg.buf[1]) << 8) | (static_cast<uint32_t>(msg.buf[2]) << 16) | (static_cast<uint32_t>(msg.buf[3]) << 24);

    // Byte 4: current_state
    current_state = msg.buf[4];
    current_state_debug = current_state;

    // Byte 5: error flags (bit0=motor, bit1=encoder, bit2=controller)
    error_flags = msg.buf[5];
    has_error = (axis_error != 0) || (error_flags & 0x07) != 0;
    has_error_debug = has_error ? 1 : 0;

    // Bytes [6:7]: infra_adc_raw (uint16, little-endian)
    uint16_t infra_adc_raw = static_cast<uint16_t>(msg.buf[6]) | (static_cast<uint16_t>(msg.buf[7]) << 8);
    infra_voltage_raw = (static_cast<float>(infra_adc_raw) / kAdcMax) * kAdcRefVoltage;
    infra_voltage_raw_debug = infra_voltage_raw;
}

void DribblerZfoc::filter_voltage() {
    if (infra_voltage_raw > infra_voltage_filt) {
        // Rising edge: instant response
        infra_voltage_filt = infra_voltage_raw;
    } else {
        // Falling edge: low-pass filter
        infra_voltage_filt += kFilterAlpha * (infra_voltage_raw - infra_voltage_filt);
    }
    infra_voltage_filt_debug = infra_voltage_filt;
}

void DribblerZfoc::process_state_machine(bool torque_mode) {
    pending_count = 0;

    // Idle confirmation gate: wait until heartbeat has reported Idle (buf[4]==1)
    // for at least kIdleConfirmDelayMs continuously before sending any CAN commands.
    if (!idle_confirmed_) {
        if (!has_error && current_state == kAxisStateIdle) {
            if (idle_first_tick_ == 0) {
                idle_first_tick_ = HAL_GetTick();
            } else if (HAL_GetTick() - idle_first_tick_ >= kIdleConfirmDelayMs) {
                idle_confirmed_ = true;
            }
        } else {
            idle_first_tick_ = 0;
        }
        return;
    }

    if (!has_error && current_state == kAxisStateIdle) {
        build_set_requested_state_msg(pending_msgs[pending_count++], kAxisStateClosedLoopControl);
        const int32_t ctrl_mode = torque_mode ? kControlModeTorque : kControlModeVelocity;
        build_set_controller_modes_msg(pending_msgs[pending_count++], ctrl_mode, kInputModePassthrough);
    }
}

void DribblerZfoc::build_set_requested_state_msg(can_Message_t& msg, int32_t state) const {
    msg.id = kCanIdSetRequestedState;
    msg.isExt = false;
    msg.rtr = false;
    msg.len = 8;
    memset(msg.buf, 0, sizeof(msg.buf));
    memcpy(msg.buf, &state, sizeof(state));
}

void DribblerZfoc::build_set_controller_modes_msg(can_Message_t& msg, int32_t ctrl_mode, int32_t input_mode) const {
    msg.id = kCanIdSetControllerModes;
    msg.isExt = false;
    msg.rtr = false;
    msg.len = 8;
    memset(msg.buf, 0, sizeof(msg.buf));
    memcpy(&msg.buf[0], &ctrl_mode, sizeof(ctrl_mode));
    memcpy(&msg.buf[4], &input_mode, sizeof(input_mode));
}

void DribblerZfoc::build_torque_msg(can_Message_t& msg, float torque, float velocity) const {
    // buf[0..3]: torque [Nm], buf[4..7]: velocity [turns/s or m/s], both little-endian float32
    msg.id = kCanIdSetInputTorque;
    msg.isExt = false;
    msg.rtr = false;
    msg.len = 8;
    memcpy(&msg.buf[0], &torque, sizeof(torque));
    memcpy(&msg.buf[4], &velocity, sizeof(velocity));
}

void DribblerZfoc::build_velocity_msg(can_Message_t& msg, float velocity, float torque_ff) const {
    // buf[0..3]: velocity [turns/s], buf[4..7]: torque feedforward [Nm], both little-endian float32
    msg.id = kCanIdSetInputVelocity;
    msg.isExt = false;
    msg.rtr = false;
    msg.len = 8;
    memcpy(&msg.buf[0], &velocity, sizeof(velocity));
    memcpy(&msg.buf[4], &torque_ff, sizeof(torque_ff));
}
