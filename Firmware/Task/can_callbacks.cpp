#include "can_callbacks.h"

#include "Communication/can/canbus.hpp"
#include "Component/opt_flow.hpp"
#include "Component/wheel_motor.hpp"
#include "Component/dribble_motor.hpp"
#include "Component/dribbler_zfoc.hpp"
#include "freertos_vars.h"
#include <cstring>
#include "stm32f4xx_hal.h"

namespace {

DualOptFlowSnapshot_t g_optflow_snapshot = {};

}

// Callback for optical flow sensor data (CAN ID 0x300 left, 0x301 right)
void on_optflow_rx(void* ctx, const can_Message_t& msg) {
    (void)ctx;

    if (msg.len != 8) {
        return;
    }

    if (msg.id != kOptFlowCanIdLeft && msg.id != kOptFlowCanIdRight) {
        return;
    }

    float x = 0.0f;
    float y = 0.0f;
    memcpy(&x, &msg.buf[0], sizeof(float));
    memcpy(&y, &msg.buf[4], sizeof(float));

    DualOptFlowSnapshot_t snapshot = g_optflow_snapshot;
    if (msg.id == kOptFlowCanIdLeft) {
        snapshot.left_x = x;
        snapshot.left_y = -y;
        snapshot.valid_mask |= 0x01;
        snapshot.left_tick_ms = HAL_GetTick();
    } else {
        snapshot.right_x = x;
        snapshot.right_y = -y;
        snapshot.valid_mask |= 0x02;
        snapshot.right_tick_ms = HAL_GetTick();
    }
    snapshot.tick_ms = HAL_GetTick();
    g_optflow_snapshot = snapshot;

    if (q_optflow_dataHandle != nullptr) {
        // Best-effort queueing: producer keeps the latest snapshot and does not wait for paired frames.
        osMessageQueuePut(q_optflow_dataHandle, &snapshot, 0, 0);
    }
}

// Callback for dribbler ZFOC heartbeat messages (CAN ID 0x0A1)
void on_dribbler_heartbeat_rx(void* ctx, const can_Message_t& msg) {
    DribblerZfoc* dribbler = static_cast<DribblerZfoc*>(ctx);
    if (dribbler) {
        dribbler->parse_heartbeat(msg);
    }
}

// Callback for motor feedback messages (CAN IDs 0x201-0x205)
void on_motor_fb_rx(void* ctx, const can_Message_t& msg) {

    can_Message_t fb_msg = msg; // Make a copy

    // Send to queue (non-blocking from ISR context)
    osMessageQueuePut(q_motor_fbHandle, &fb_msg, 0, 0);
}
