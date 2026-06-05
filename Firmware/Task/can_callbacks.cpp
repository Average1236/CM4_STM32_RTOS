#include "can_callbacks.h"

#include "Communication/can/canbus.hpp"
#include "Component/opt_flow.hpp"
#include "Component/wheel_motor.hpp"
#include "Component/dribble_motor.hpp"
#include "Component/dribbler_zfoc.hpp"
#include "freertos_vars.h"
#include <cstring>


// Callback for optical flow sensor data (CAN ID 0x300)
void on_optflow_rx(void* ctx, const can_Message_t& msg) {
    if (msg.len != 8) return;
    
    OptFlow::Data_t data;
    memcpy(&data.x, &msg.buf[0], sizeof(float));
    memcpy(&data.y, &msg.buf[4], sizeof(float));
    
    // Send to queue (non-blocking from ISR context)
    osMessageQueuePut(q_optflow_dataHandle, &data, 0, 0);
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
