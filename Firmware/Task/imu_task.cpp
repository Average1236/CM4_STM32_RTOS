#include "cmsis_os.h"
#include "freertos_vars.h"
#include "z_main.h"
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
        // Wake source is model-dependent:
        // - JY931: UART RX callback releases sem_imu_readyHandle.
        // - ICM42688: TIM7 ISR releases sem_imu_readyHandle at 800Hz.
        const osStatus_t imu_wait_status = osSemaphoreAcquire(sem_imu_readyHandle, osWaitForever);
        if (imu_wait_status == osOK) {
            float current_time = TIM13->CNT / 1000000.0f;  // Current time in seconds
            imu_dt = (current_time > imu_last_time) ? (current_time - imu_last_time) : (current_time + (0.065536f - imu_last_time));
            imu_last_time = current_time;
            // Acquire robot state mutex
            if (osMutexAcquire(mtx_robot_stateHandle, 10) == osOK) {
                imu.process_once();
                
                // // Signal IMU data ready
                // osSemaphoreRelease(sem_imu_readyHandle);
                
                osMutexRelease(mtx_robot_stateHandle);
            }
            
        }
    }
}

} // extern "C"
