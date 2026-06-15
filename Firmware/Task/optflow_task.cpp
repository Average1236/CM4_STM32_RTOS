#include "cmsis_os.h"
#include "freertos_vars.h"
#include "can_callbacks.h"
#include "z_main.h"
#include "Component/opt_flow.hpp"
#include <cstring>

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
    bool have_snapshot = false;

    for(;;) {
        // Poll with timeout so we can maintain explicit offline state.
        if (osMessageQueueGet(q_optflow_dataHandle, &snapshot, NULL, kOptFlowQueueWaitMs) == osOK) {
            have_snapshot = true;

            // --- Mirror raw snapshot to global vars (debug) ---
            dual_flow_left_x   = snapshot.left_x;
            dual_flow_left_y   = snapshot.left_y;
            dual_flow_right_x  = snapshot.right_x;
            dual_flow_right_y  = snapshot.right_y;
            optflow_valid_mask = snapshot.valid_mask;
            mouse_time_ms      = snapshot.tick_ms;
            mouse_left_tick_ms  = snapshot.left_tick_ms;
            mouse_right_tick_ms = snapshot.right_tick_ms;

            // --- Build OptFlow::Data_t (flow sensor only; IMU via Ports) ---
            OptFlow::Data_t data;
            data.left_x        = snapshot.left_x;
            data.left_y        = snapshot.left_y;
            data.right_x       = snapshot.right_x;
            data.right_y       = snapshot.right_y;
            data.tick_ms       = snapshot.tick_ms;
            data.left_tick_ms  = snapshot.left_tick_ms;
            data.right_tick_ms = snapshot.right_tick_ms;
            data.valid_mask    = snapshot.valid_mask;

            opt_flow.process(data);

            // --- Pull fused state from OutputPorts ---
            const OptFlow::State_t& s = opt_flow.get_state();

            dual_flow_left_vx  = s.pll_l_vx;
            dual_flow_left_vy  = s.pll_l_vy;
            dual_flow_right_vx = s.pll_r_vx;
            dual_flow_right_vy = s.pll_r_vy;

            body_vx = s.kf_vx * 0.001f; // mm/s → m/s
            body_vy = s.kf_vy * 0.001f; // mm/s → m/s
            omega_z = s.omega_z;
            robot_pos_x_mm = s.flow_px;
            robot_pos_y_mm = s.flow_py;
            flow_px = s.flow_px;
            flow_py = s.flow_py;
            flow_yaw = s.flow_yaw;

            raw_vx = s.body_vx;
            raw_vy = s.body_vy;

            g_optflow_last_update_ms = HAL_GetTick();
            g_optflow_available = true;
        } else {
            const uint32_t now_ms = HAL_GetTick();
            if ((now_ms - g_optflow_last_update_ms) > kOptFlowOfflineTimeoutMs) {
                g_optflow_available = false;
            }
            if (have_snapshot) {
                OptFlow::Data_t data;
                data.left_x        = snapshot.left_x;
                data.left_y        = snapshot.left_y;
                data.right_x       = snapshot.right_x;
                data.right_y       = snapshot.right_y;
                data.tick_ms       = now_ms;
                data.left_tick_ms  = snapshot.left_tick_ms;
                data.right_tick_ms = snapshot.right_tick_ms;
                data.valid_mask    = 0u;
                opt_flow.process(data);
            }
        }
    }
}

} // extern "C"
