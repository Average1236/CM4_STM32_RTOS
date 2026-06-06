#include "cmsis_os.h"
#include "freertos_vars.h"
#include "z_main.h"
#include "Component/control_params.hpp"
#include <cstring>

float imu_last_time = 0;
float imu_dt = 0;

// IMU data structure
struct ImuData {
    float data[9];  // ax, ay, az, gx, gy, gz, roll, pitch, yaw
};

extern "C" {

// ImuRxTask - Process IMU data via UART
void StartImuRxTask(void *argument) {
    osDelay(100);  // Wait for initialization
    
    for(;;) {
        // ICM42688: TIM7 ISR releases sem_imu_readyHandle at 800Hz.
        const osStatus_t imu_wait_status = osSemaphoreAcquire(sem_imu_readyHandle, osWaitForever);
        if (imu_wait_status == osOK) {
            float current_time = TIM13->CNT / 1000000.0f;  // Current time in seconds
            imu_dt = (current_time > imu_last_time) ? (current_time - imu_last_time) : (current_time + (0.065536f - imu_last_time));
            imu_last_time = current_time;

            imu.process_once();
            imu.update_integrated_yaw(imu_dt);
            // Roll/pitch with gyro bias correction from SixAxisImuBias
            imu.update_roll_pitch(imu_dt,
                g_imu_trust_accel,
                g_imu_bias_gx_dps,
                g_imu_bias_gy_dps);
        }
    }
}

} // extern "C"
