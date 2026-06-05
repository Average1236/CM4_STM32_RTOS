#include "cmsis_os.h"
#include "freertos_vars.h"
#include "can_callbacks.h"
#include "z_main.h"
#include "Component/opt_flow.hpp"
#include <cstring>

// IMU init flag defined in board.cpp (set after imu.init() succeeds)
extern volatile uint8_t g_imu_init_ok;

// OptFlow health flags consumed by telemetry_task
volatile bool g_optflow_available = false;
volatile uint32_t g_optflow_last_update_ms = 0;

namespace {
static constexpr uint32_t kOptFlowQueueWaitMs = 20;
static constexpr uint32_t kOptFlowOfflineTimeoutMs = 200;
}

// Debug / telemetry globals (snapshot + per-sensor velocities)
float dual_flow_left_x;
float dual_flow_left_y;
float dual_flow_right_x;
float dual_flow_right_y;
float dual_flow_left_vx;
float dual_flow_left_vy;
float dual_flow_right_vx;
float dual_flow_right_vy;
float raw_vx, raw_vy;
float body_vx, body_vy, omega_z;
float robot_pos_x_mm, robot_pos_y_mm;
float flow_px, flow_py, flow_yaw;
unsigned int mouse_time_ms;
unsigned int optflow_valid_mask;
unsigned int mouse_left_tick_ms;
unsigned int mouse_right_tick_ms;

extern "C" {

// OptFlowRxTask - Process dual optical flow sensor data fused with IMU
void StartOptFlowRxTask(void *argument) {
    osDelay(100);  // Wait for initialization

    DualOptFlowSnapshot_t snapshot;

    for(;;) {
        // Poll with timeout so we can maintain explicit offline state.
        if (osMessageQueueGet(q_optflow_dataHandle, &snapshot, NULL, kOptFlowQueueWaitMs) == osOK) {

            // --- Mirror raw snapshot to global vars (debug) ---
            dual_flow_left_x   = snapshot.left_x;
            dual_flow_left_y   = snapshot.left_y;
            dual_flow_right_x  = snapshot.right_x;
            dual_flow_right_y  = snapshot.right_y;
            optflow_valid_mask = snapshot.valid_mask;
            mouse_time_ms      = snapshot.tick_ms;
            mouse_left_tick_ms  = snapshot.left_tick_ms;
            mouse_right_tick_ms = snapshot.right_tick_ms;

            // --- Build OptFlow::Data_t (flow + IMU) ---
            OptFlow::Data_t data;
            data.left_x        = snapshot.left_x;
            data.left_y        = snapshot.left_y;
            data.right_x       = snapshot.right_x;
            data.right_y       = snapshot.right_y;
            data.tick_ms       = snapshot.tick_ms;
            data.left_tick_ms  = snapshot.left_tick_ms;
            data.right_tick_ms = snapshot.right_tick_ms;
            data.valid_mask    = snapshot.valid_mask;

            // Fetch the latest IMU frame and feed acc x/y (m/s²) + gyro z (rad/s).
            float imu_frame[9];
            imu.get_data(imu_frame);
            data.imu_acc_x   = imu_frame[0];
            data.imu_acc_y   = imu_frame[1];
            data.imu_omega_z = imu_frame[5] * (3.14159f / 180.0f); // DPS -> rad/s
            data.imu_valid   = (g_imu_init_ok != 0);

            opt_flow.process(data);

            // --- Pull fused state back into globals ---
            const OptFlow::State_t& s = opt_flow.get_state();

            dual_flow_left_vx  = s.left_vx;
            dual_flow_left_vy  = s.left_vy;
            dual_flow_right_vx = s.right_vx;
            dual_flow_right_vy = s.right_vy;

            // body_vx/vy 现在采用卡尔曼融合结果；omega_z 用原始光流值
            body_vx = s.kf_vx;
            body_vy = s.kf_vy;
            omega_z = s.omega_z;
            robot_pos_x_mm = s.flow_px;
            robot_pos_y_mm = s.flow_py;
            flow_px = s.flow_px;
            flow_py = s.flow_py;
            flow_yaw = s.flow_yaw;

            // raw 保留原始光流推导，方便调试对比
            raw_vx = s.body_vx;
            raw_vy = s.body_vy;

            g_optflow_last_update_ms = HAL_GetTick();
            g_optflow_available = true;
        } else {
            const uint32_t now_ms = HAL_GetTick();
            if ((now_ms - g_optflow_last_update_ms) > kOptFlowOfflineTimeoutMs) {
                g_optflow_available = false;
            }
        }
    }
}

} // extern "C"
