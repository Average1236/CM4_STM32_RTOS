#ifndef __CAN_CALLBACKS_H
#define __CAN_CALLBACKS_H

#include "Communication/can/can_helpers.hpp"
#include <stdint.h>

enum {
    kOptFlowCanIdLeft  = 0x300,
    kOptFlowCanIdRight = 0x301,
    kBallSensorCanId   = 0x123,
};

struct DualOptFlowSnapshot_t {
    float left_x;
    float left_y;
    float right_x;
    float right_y;
    uint32_t valid_mask;
    uint32_t tick_ms;
    uint32_t left_tick_ms;
    uint32_t right_tick_ms;
};

static_assert(sizeof(DualOptFlowSnapshot_t) == 32, "DualOptFlowSnapshot_t size must match RTOS queue item size");

struct BallSensorSnapshot_t {
    uint16_t left_mm;
    uint16_t right_mm;
    uint8_t depth_mm;
    uint8_t depth_level;
    uint8_t valid;
    uint8_t seq;
    uint32_t tick_ms;
};

static_assert(sizeof(BallSensorSnapshot_t) == 12, "BallSensorSnapshot_t size changed unexpectedly");

extern volatile uint32_t ball_sensor_can_counter;
extern volatile BallSensorSnapshot_t g_ball_sensor_snapshot;

// CAN message callbacks for ZCAN subscriptions
void on_optflow_rx(void* ctx, const can_Message_t& msg);
void on_ball_sensor_rx(void* ctx, const can_Message_t& msg);
void on_motor_fb_rx(void* ctx, const can_Message_t& msg);
void on_dribbler_heartbeat_rx(void* ctx, const can_Message_t& msg);

#endif // __CAN_CALLBACKS_H
