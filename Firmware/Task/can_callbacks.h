#ifndef __CAN_CALLBACKS_H
#define __CAN_CALLBACKS_H

#include "Communication/can/can_helpers.hpp"
#include <stdint.h>

enum {
    kOptFlowCanIdLeft  = 0x300,
    kOptFlowCanIdRight = 0x301,
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

// CAN message callbacks for ZCAN subscriptions
void on_optflow_rx(void* ctx, const can_Message_t& msg);
void on_motor_fb_rx(void* ctx, const can_Message_t& msg);

#endif // __CAN_CALLBACKS_H
