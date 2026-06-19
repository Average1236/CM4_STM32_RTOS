#include "can_callbacks.h"

#include "Communication/can/canbus.hpp"
#include "Component/opt_flow.hpp"
#include "Component/wheel_motor.hpp"
#include "Component/dribble_motor.hpp"
#include "Component/dribbler_zfoc.hpp"
#include "freertos_vars.h"
#include <cstring>
#include "stm32f4xx_hal.h"

volatile uint32_t optflow_can_counter[2] = {0, 0}; // For debugging: count received frames for left (0) and right (1) optical flow sensors
volatile uint32_t motor_fb_bad_len_count = 0;
volatile uint32_t motor_fb_unmatched_count = 0;
volatile uint32_t motor_fb_queue_drop_count = 0;

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
        optflow_can_counter[0]++;
    } else if (msg.id == kOptFlowCanIdRight) {
        snapshot.right_x = x;
        snapshot.right_y = -y;
        snapshot.valid_mask |= 0x02;
        snapshot.right_tick_ms = HAL_GetTick();
        optflow_can_counter[1]++;
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

// Callback for motor feedback messages (exact wheel feedback IDs on CAN2)
void on_motor_fb_rx(void* ctx, const can_Message_t& msg) {
    WheelMotorBase* motor = static_cast<WheelMotorBase*>(ctx);
    if (msg.len != 8) {
        motor_fb_bad_len_count++;
        return;
    }
    if (motor == nullptr || msg.id != motor->feedback_can_id()) {
        motor_fb_unmatched_count++;
        return;
    }
    if (q_motor_fbHandle == nullptr) {
        motor_fb_queue_drop_count++;
        return;
    }

    can_Message_t fb_msg = msg;
    if (osMessageQueuePut(q_motor_fbHandle, &fb_msg, 0, 0) != osOK) {
        motor_fb_queue_drop_count++;
    }
}
