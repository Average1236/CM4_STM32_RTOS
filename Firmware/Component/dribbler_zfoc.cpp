#include "dribbler_zfoc.hpp"
#include "Task/z_main.h"
#include <cstring>

void DribblerZfoc::parse_heartbeat(const can_Message_t& msg) {
    // Byte 4: current_state
    current_state = msg.buf[4];

    // Bytes [6:7]: infra_adc_raw (uint16, little-endian)
    uint16_t infra_adc_raw = static_cast<uint16_t>(msg.buf[6]) | (static_cast<uint16_t>(msg.buf[7]) << 8);
    infra_voltage_raw = (static_cast<float>(infra_adc_raw) / kAdcMax) * kAdcRefVoltage;
}

void DribblerZfoc::filter_voltage() {
    if (infra_voltage_raw > infra_voltage_filt) {
        // Rising edge: instant response
        infra_voltage_filt = infra_voltage_raw;
    } else {
        // Falling edge: low-pass filter
        infra_voltage_filt += kFilterAlpha * (infra_voltage_raw - infra_voltage_filt);
    }
}

void DribblerZfoc::process_state_machine() {
    // Detect calibration completion: FULL_CALIBRATION -> IDLE
    if (last_current_state_ == kAxisStateFullCalibration && current_state == kAxisStateIdle) {
        if (!closed_loop_requested_) {
            send_requested_state(kAxisStateClosedLoopControl);
            send_controller_mode(kControlModeTorque, kInputModePassthrough);
            closed_loop_requested_ = true;
        }
    }

    // Trigger calibration if we see IDLE before ever requesting closed-loop
    if (current_state == kAxisStateIdle && !calibration_requested_ && !closed_loop_requested_) {
        send_requested_state(kAxisStateFullCalibration);
        calibration_requested_ = true;
    }

    last_current_state_ = current_state;
}

void DribblerZfoc::send_requested_state(int32_t state) {
    can_Message_t msg;
    msg.id = kCanIdSetRequestedState;
    msg.isExt = false;
    msg.rtr = false;
    msg.len = 8;
    memset(msg.buf, 0, sizeof(msg.buf));
    memcpy(msg.buf, &state, sizeof(state));
    can1_bus.send_message(msg);
}

void DribblerZfoc::send_controller_mode(int32_t ctrl_mode, int32_t input_mode) {
    can_Message_t msg;
    msg.id = kCanIdSetControllerModes;
    msg.isExt = false;
    msg.rtr = false;
    msg.len = 8;
    memset(msg.buf, 0, sizeof(msg.buf));
    memcpy(&msg.buf[0], &ctrl_mode, sizeof(ctrl_mode));
    memcpy(&msg.buf[4], &input_mode, sizeof(input_mode));
    can1_bus.send_message(msg);
}

void DribblerZfoc::send_torque(float torque) {
    can_Message_t msg;
    msg.id = kCanIdSetInputTorque;
    msg.isExt = false;
    msg.rtr = false;
    msg.len = 8;
    memset(msg.buf, 0, sizeof(msg.buf));
    memcpy(msg.buf, &torque, sizeof(torque));
    can1_bus.send_message(msg);
}
