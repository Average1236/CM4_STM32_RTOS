#include "cmsis_os.h"
#include "freertos_vars.h"
#include "z_main.h"
// #include "SEGGER_RTT.h"
#include <cstdio>

// Timestamp monitoring structure
struct SensorTimestamp_t {
    uint32_t optflow_last;
    uint32_t motor_last;
    uint32_t imu_last;
    uint32_t ctrl_last;
    
    uint32_t optflow_delay;
    uint32_t motor_delay;
    uint32_t imu_delay;
    uint32_t ctrl_jitter;
    
    uint32_t optflow_count;
    uint32_t motor_count;
    uint32_t imu_count;
    uint32_t ctrl_count;
};

static SensorTimestamp_t ts = {0};

extern float body_vx, body_vy, omega_z;
extern float raw_vx, raw_vy;

extern "C" {

// TelemetryTask - Debug output and timestamp monitoring
void StartTelemetryTask(void *argument) {
    osDelay(200);  // Wait for system startup
    
    char rtt_buf[256];
    uint32_t last_report_time = 0;
    
    for(;;) {
        uint32_t current_time = HAL_GetTick();
        
        // Update timestamp statistics
        if (osMutexAcquire(mtx_robot_stateHandle, 10) == osOK) {
            // Monitor optical flow timing with explicit online/offline status.
            if (g_optflow_available) {
                ts.optflow_delay = current_time - g_optflow_last_update_ms;
                if (g_optflow_last_update_ms != ts.optflow_last) {
                    ts.optflow_last = g_optflow_last_update_ms;
                    ts.optflow_count++;
                }
            } else {
                ts.optflow_delay = 0;
            }
            
            osMutexRelease(mtx_robot_stateHandle);
        }
        
        // Report statistics every second
        if (current_time - last_report_time >= 1000) {
            
            // Format telemetry data as CSV
            // Time, OptFlow_Hz, OptFlow_Delay, Motor_Hz, IMU_Hz, Ctrl_Hz
            snprintf(rtt_buf, sizeof(rtt_buf), 
                     "%lu,%lu,%lu,%lu,%lu,%lu\n",
                     current_time,
                     ts.optflow_count,
                     ts.optflow_delay,
                     ts.motor_count,
                     ts.imu_count,
                     ts.ctrl_count);
            
            // SEGGER_RTT_WriteString(0, rtt_buf);
            
            // // Output velocity data for debugging
            // snprintf(rtt_buf, sizeof(rtt_buf),
            //          "VEL: raw_vx=%.1f, vx=%.1f, raw_vy=%.1f, vy=%.1f, omega=%.2f\n",
            //          raw_vx, body_vx, raw_vy, body_vy, omega_z);
            // SEGGER_RTT_WriteString(0, rtt_buf);
            
            // Reset counters
            ts.optflow_count = 0;
            ts.motor_count = 0;
            ts.imu_count = 0;
            ts.ctrl_count = 0;
            
            last_report_time = current_time;
        }
        
        // Run at 10Hz
        osDelay(100);
    }
}

} // extern "C"
