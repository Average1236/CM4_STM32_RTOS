#ifndef __CAN_CALLBACKS_H
#define __CAN_CALLBACKS_H

#include "Communication/can/can_helpers.hpp"

// CAN message callbacks for ZCAN subscriptions
void on_optflow_rx(void* ctx, const can_Message_t& msg);
void on_motor_fb_rx(void* ctx, const can_Message_t& msg);
void on_dribbler_heartbeat_rx(void* ctx, const can_Message_t& msg);

#endif // __CAN_CALLBACKS_H
