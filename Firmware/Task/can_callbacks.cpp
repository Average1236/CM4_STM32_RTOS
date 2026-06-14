#include "can_callbacks.h"

#include "Communication/can/canbus.hpp"
#include "Component/opt_flow.hpp"
#include "Component/wheel_motor.hpp"
#include "Component/dribble_motor.hpp"
#include "Component/dribbler_zfoc.hpp"
#include "freertos_vars.h"
#include "Task/z_main.h"
#include <cstring>
#include "stm32f4xx_hal.h"

volatile uint32_t optflow_can_counter[2] = {0, 0}; // For debugging: count received frames for left (0) and right (1) optical flow sensors
volatile uint32_t ball_sensor_can_counter = 0;
volatile BallSensorSnapshot_t g_ball_sensor_snapshot = {
    0xFFFFu, 0xFFFFu, 0xFFu, 0xFFu, 0u, 0u, 0u
};

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
    } else {
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

// Callback for infrared matrix ball sensor data (CAN1, ID 0x123)
void on_ball_sensor_rx(void* ctx, const can_Message_t& msg) {
    (void)ctx;

    if (msg.id != kBallSensorCanId || msg.len < 4) {
        return;
    }

    BallSensorSnapshot_t snapshot = {};
    snapshot.left_mm = static_cast<uint16_t>(msg.buf[0]) |
                       (static_cast<uint16_t>(msg.buf[1]) << 8);

    if (msg.len >= 8) {
        snapshot.right_mm = static_cast<uint16_t>(msg.buf[2]) |
                            (static_cast<uint16_t>(msg.buf[3]) << 8);
        snapshot.depth_mm = msg.buf[4];
        snapshot.depth_level = msg.buf[5];
        snapshot.valid = msg.buf[6];
        snapshot.seq = msg.buf[7];
    } else {
        snapshot.right_mm = 0xFFFFu;
        snapshot.depth_mm = msg.buf[2];
        snapshot.depth_level = 0xFFu;
        snapshot.valid = (snapshot.left_mm != 0xFFFFu && snapshot.depth_mm != 0xFFu) ? 1u : 0u;
        snapshot.seq = msg.buf[3];
    }

    snapshot.tick_ms = HAL_GetTick();
    g_ball_sensor_snapshot.left_mm = snapshot.left_mm;
    g_ball_sensor_snapshot.right_mm = snapshot.right_mm;
    g_ball_sensor_snapshot.depth_mm = snapshot.depth_mm;
    g_ball_sensor_snapshot.depth_level = snapshot.depth_level;
    g_ball_sensor_snapshot.valid = snapshot.valid;
    g_ball_sensor_snapshot.seq = snapshot.seq;
    g_ball_sensor_snapshot.tick_ms = snapshot.tick_ms;
    ball_sensor_can_counter++;

    robot.ball_sensor_left_mm = snapshot.left_mm;
    robot.ball_sensor_right_mm = snapshot.right_mm;
    robot.ball_sensor_depth_mm = snapshot.depth_mm;
    robot.ball_sensor_depth_level = snapshot.depth_level;
    robot.ball_sensor_valid = snapshot.valid;
    robot.ball_sensor_seq = snapshot.seq;
    robot.ball_sensor_tick_ms = snapshot.tick_ms;
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
