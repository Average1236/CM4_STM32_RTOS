#include "cmsis_os.h"
#include "freertos_vars.h"
#include "can_callbacks.h"
#include "z_main.h"
#include "Component/opt_flow.hpp"
#include <cstring>

float raw_X, raw_Y;
float last_X, last_Y;
float delta_X, delta_Y;
float global_X, global_Y;
float body_vx, body_vy, omega_z;
float raw_vx, raw_vy, raw_omega;
uint32_t mouse_time_ms, last_mouse_time_ms;
float dt_s;
float delta_yaw, angle;
float e, f;
float all;
float imu_omega_z;
float imu_angle_z;

volatile bool g_optflow_available = false;
volatile uint32_t g_optflow_last_update_ms = 0;

namespace {
static constexpr uint32_t kOptFlowQueueWaitMs = 20;
static constexpr uint32_t kOptFlowOfflineTimeoutMs = 200;
}


extern "C" {

// OptFlowRxTask - Process optical flow sensor data
void StartOptFlowRxTask(void *argument) {
    osDelay(100);  // Wait for initialization
    
    OptFlow::Data_t data;
    
    for(;;) {
        // Poll queue with timeout so that we can maintain an explicit offline state.
        if (osMessageQueueGet(q_optflow_dataHandle, &data, NULL, kOptFlowQueueWaitMs) == osOK) {
            
            // // Acquire robot state mutex
            // if (osMutexAcquire(mtx_robot_stateHandle, 10) == osOK) {
                
            // Get IMU data
            imu_omega_z = imu.get_data(IMU::kOmegaZ);
            imu_angle_z = imu.get_data(IMU::kAngleZ);
            
            // Process optical flow data
            opt_flow.process(data, imu_omega_z, imu_angle_z);
            
            // Update external variables for backward compatibility
            const OptFlow::State_t& state = opt_flow.get_state();
            raw_X = state.raw_x;
            raw_Y = state.raw_y;
            last_X = state.last_x;
            last_Y = state.last_y;
            delta_X = state.delta_x;
            delta_Y = state.delta_y;
            global_X = state.global_x;
            global_Y = state.global_y;
            raw_vx = state.raw_vx;
            raw_vy = state.raw_vy;
            raw_omega = state.raw_omega;
            mouse_time_ms = state.time_ms;
            last_mouse_time_ms = state.last_time_ms;
            dt_s = state.dt_s;
            delta_yaw = state.delta_yaw;
            angle = state.angle;
            e = state.e;
            f = state.f;
            all = state.all_distance;

            g_optflow_last_update_ms = HAL_GetTick();
            g_optflow_available = true;
                
            //     osMutexRelease(mtx_robot_stateHandle);
            // }
        } else {
            const uint32_t now_ms = HAL_GetTick();
            if ((now_ms - g_optflow_last_update_ms) > kOptFlowOfflineTimeoutMs) {
                g_optflow_available = false;
            }
        }
    }
}

} // extern "C"

