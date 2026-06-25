#include "cmsis_os.h"
#include "freertos_vars.h"
#include "can_callbacks.h"
#include "z_main.h"
#include <cstring>

// Debug variables
volatile uint8_t id_debug = 0;
volatile uint32_t motor_fb_count[4] = {0, 0, 0, 0};
volatile uint32_t motor_fb_unmatched_task_count = 0;
volatile uint32_t motor_write_reg_ok_count[4] = {0, 0, 0, 0};
volatile uint32_t motor_write_reg_mismatch_count[4] = {0, 0, 0, 0};
volatile uint32_t motor_write_reg_non_ack_count[4] = {0, 0, 0, 0};
volatile uint8_t motor_write_reg_last_index = 0;
volatile uint8_t motor_write_reg_last_result = 0;
volatile uint8_t motor_write_reg_last_expected_rid = 0;
volatile uint8_t motor_write_reg_last_actual_rid = 0;
volatile uint32_t motor_write_reg_last_expected_data = 0;
volatile uint32_t motor_write_reg_last_actual_data = 0;

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
                MotorDMH3510* motor = robot.wheel_motors[matched_wheel_idx];
                if (motor->is_writing_register()) {
                    const WheelMotorBase::RegisterWriteCheckResult result =
                        motor->check_write_register_feedback(fb_msg);
                    if (result == WheelMotorBase::kRegisterWriteOk) {
                        motor_write_reg_ok_count[matched_wheel_idx]++;
                    } else if (result == WheelMotorBase::kRegisterWriteMismatch) {
                        motor_write_reg_mismatch_count[matched_wheel_idx]++;
                    } else {
                        motor_write_reg_non_ack_count[matched_wheel_idx]++;
                    }

                    motor_write_reg_last_index = static_cast<uint8_t>(matched_wheel_idx);
                    motor_write_reg_last_result = static_cast<uint8_t>(result);
                    motor_write_reg_last_expected_rid = motor->pending_write_register_rid();
                    motor_write_reg_last_actual_rid = motor->last_write_register_rid();
                    motor_write_reg_last_expected_data = motor->pending_write_register_data();
                    motor_write_reg_last_actual_data = motor->last_write_register_data();
                } else {
                    motor->parse_feedback_data(fb_msg.buf);
                }
            } else {
                motor_fb_unmatched_task_count++;
            }

        }
    }
}

} // extern "C"
