#include "cmsis_os.h"
#include "freertos_vars.h"
#include "z_main.h"
#include "Component/control_params.hpp"
#include "motion/types.hpp"
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
            imu.compute_stationary_and_bias();
            imu.update_integrated_yaw(imu_dt);
            imu.update_roll_pitch(imu_dt);

            // Feed the new estimator from raw, freshly decoded values only.
            // A failed burst read is not published (consume_raw_sample false).
            float accel[3];
            float gyro[3];
            std::uint32_t sequence = 0;
            if (imu.consume_raw_sample(accel, gyro, &sequence)) {
                motion::RawImuSample raw;
                raw.sample_time_us = motion_now_us();
                raw.accel_raw_m_s2 = motion::Vec3(accel[0], accel[1], accel[2]);
                raw.gyro_raw_rad_s = motion::Vec3(gyro[0], gyro[1], gyro[2]);
                raw.sequence = sequence;
                raw.calibration_version = 1;
                raw.valid = true;
                robot.queue_raw_imu(raw);
            }
        }
    }
}

} // extern "C"
