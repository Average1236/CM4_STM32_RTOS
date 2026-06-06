#include "cmsis_os.h"
#include "freertos_vars.h"
#include "can_callbacks.h"
#include "z_main.h"
#include "Component/opt_flow.hpp"
#include <cstring>

// OptFlow health flags consumed by telemetry_task
volatile bool g_optflow_available = false;
volatile uint32_t g_optflow_last_update_ms = 0;

// IMU roll/pitch anchor flag + gyro bias (written by SixAxisImuBias, read by imu_task)
volatile bool  g_imu_trust_accel = false;
volatile float g_imu_bias_gx_dps = 0.0f;
volatile float g_imu_bias_gy_dps = 0.0f;

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

            // Fetch the latest IMU frame — all 6 axes (3 acc + 3 gyro)
            float imu_frame[9];
            imu.get_data(imu_frame);
            data.imu_acc_x   = imu_frame[0];   // ax  m/s²
            data.imu_acc_y   = imu_frame[1];   // ay  m/s²
            data.imu_acc_z   = imu_frame[2];   // az  m/s²
            data.imu_gyro_x  = imu_frame[3];   // gx  deg/s
            data.imu_gyro_y  = imu_frame[4];   // gy  deg/s
            data.imu_gyro_z  = imu_frame[5];   // gz  deg/s (will be converted to rad/s in process)
            data.imu_valid   = true;

            opt_flow.process(data);

            // --- Pull fused state back into globals ---
            const OptFlow::State_t& s = opt_flow.get_state();

            dual_flow_left_vx  = s.pll_l_vx;
            dual_flow_left_vy  = s.pll_l_vy;
            dual_flow_right_vx = s.pll_r_vx;
            dual_flow_right_vy = s.pll_r_vy;

            // body_vx/vy 采用卡尔曼融合结果
            body_vx = s.kf_vx;
            body_vy = s.kf_vy;
            omega_z = s.omega_z;
            robot_pos_x_mm = s.flow_px;
            robot_pos_y_mm = s.flow_py;
            flow_px = s.flow_px;
            flow_py = s.flow_py;
            flow_yaw = s.flow_yaw;

            // raw 保留 PLL 融合后的 body 速度，方便调试对比
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
