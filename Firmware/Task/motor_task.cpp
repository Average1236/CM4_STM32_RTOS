#include "cmsis_os.h"
#include "freertos_vars.h"
#include "can_callbacks.h"
#include "z_main.h"
#include <cstring>

// Debug variables
volatile uint8_t id_debug = 0;
volatile uint32_t motor_fb_count[4] = {0, 0, 0, 0};
volatile uint32_t motor_fb_unmatched_task_count = 0;

extern "C" {

// MotorRxTask - Process motor feedback messages
void StartMotorRxTask(void *argument) {
    osDelay(100);  // Wait for initialization
    
    can_Message_t fb_msg;
    
    for(;;) {
        // Block waiting for motor feedback from queue
        if (osMessageQueueGet(q_motor_fbHandle, &fb_msg, NULL, osWaitForever) == osOK) {
            // Match wheel motor feedback by configured feedback CAN ID instead of hardcoded IDs.
            int matched_wheel_idx = -1;
            for (int i = 0; i < 4; ++i) {
                if (robot.wheel_motors[i] != nullptr &&
                    fb_msg.id == robot.wheel_motors[i]->feedback_can_id()) {
                    matched_wheel_idx = i;
                    break;
                }
            }

            if (matched_wheel_idx >= 0) {
                motor_fb_count[matched_wheel_idx]++;
                if (!robot.wheel_motors[matched_wheel_idx]->is_writing_register()) {
                    robot.wheel_motors[matched_wheel_idx]->parse_feedback_data(fb_msg.buf);
                }
            } else {
                motor_fb_unmatched_task_count++;
            }

        }
    }
}

} // extern "C"
