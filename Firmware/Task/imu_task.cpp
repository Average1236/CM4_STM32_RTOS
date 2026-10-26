#include "cmsis_os.h"
#include "freertos_vars.h"
#include "z_main.h"
#include "Component/control_params.hpp"
#include "motion/types.hpp"
#include <cstring>

extern "C" {

void StartImuRxTask(void *argument) {
    osDelay(100);  // Wait for initialization

    for(;;) {
        // ICM42688: TIM7 ISR releases sem_imu_readyHandle at 800Hz.
        const osStatus_t imu_wait_status = osSemaphoreAcquire(sem_imu_readyHandle, osWaitForever);
        if (imu_wait_status == osOK) {
            imu.process_once();
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
