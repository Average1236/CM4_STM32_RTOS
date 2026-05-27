#ifndef DRIBBLER_ZFOC_HPP
#define DRIBBLER_ZFOC_HPP

#include "Communication/can/can_helpers.hpp"
#include <cstdint>

class DribblerZfoc {
public:
    // ZFOC CAN protocol constants
    static constexpr uint8_t kNodeId = 5;

    // CAN command IDs (cmd_id in low 5 bits)
    static constexpr uint8_t kCmdHeartbeat = 0x001;
    static constexpr uint8_t kCmdSetRequestedState = 0x006;
    static constexpr uint8_t kCmdSetControllerModes = 0x00A;
    static constexpr uint8_t kCmdSetInputTorque = 0x00D;

    // Full CAN IDs = (node_id << 5) | cmd_id
    static constexpr uint32_t kCanIdHeartbeat = (kNodeId << 5) | kCmdHeartbeat;
    static constexpr uint32_t kCanIdSetRequestedState = (kNodeId << 5) | kCmdSetRequestedState;
    static constexpr uint32_t kCanIdSetControllerModes = (kNodeId << 5) | kCmdSetControllerModes;
    static constexpr uint32_t kCanIdSetInputTorque = (kNodeId << 5) | kCmdSetInputTorque;

    // Axis states
    static constexpr uint8_t kAxisStateUndefined = 0;
    static constexpr uint8_t kAxisStateIdle = 1;
    static constexpr uint8_t kAxisStateFullCalibration = 3;
    static constexpr uint8_t kAxisStateClosedLoopControl = 8;

    // Control / input modes
    static constexpr int32_t kControlModeTorque = 1;
    static constexpr int32_t kInputModePassthrough = 1;

    // Conversion
    static constexpr float kAdcMax = 4095.0f;
    static constexpr float kAdcRefVoltage = 3.3f;

    // Filter
    static constexpr float kFilterAlpha = 0.1f;

    // ISR-safe: parse heartbeat message, extract infra voltage and current state
    void parse_heartbeat(const can_Message_t& msg);

    // Called in ctrl_task: asymmetric low-pass filter on falling edge only
    void filter_voltage();

    // Called in ctrl_task: if no error and idle, queue state transition messages
    void process_state_machine();

    // Build CAN messages (does NOT send — ctrl_task sends under semaphore)
    void build_torque_msg(can_Message_t& msg, float torque) const;

    // Pending messages populated by process_state_machine, sent by ctrl_task
    uint8_t pending_count = 0;
    can_Message_t pending_msgs[2];

    // State (ISR writes raw, ctrl_task reads/filters)
    float infra_voltage_raw = 0.0f;
    float infra_voltage_filt = 0.0f;
    uint8_t current_state = kAxisStateUndefined;
    bool has_error = false;
    uint8_t error_flags = 0;
    uint32_t axis_error = 0;

private:
    void build_set_requested_state_msg(can_Message_t& msg, int32_t state) const;
    void build_set_controller_modes_msg(can_Message_t& msg, int32_t ctrl_mode, int32_t input_mode) const;
};

#endif // DRIBBLER_ZFOC_HPP
