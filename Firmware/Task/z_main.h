#ifndef __Z_MAIN_H
#define __Z_MAIN_H

// Hardware configuration
#include "board.h"

#include "main.h"

#include "Task/can_callbacks.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "cmsis_os.h"
#include "stm32f4xx_hal.h"
#include "freertos_vars.h"

#define PI_UART_TX_DATA_LENGTH 3
#define PI_UART_RX_DATA_LENGTH 7
#define SPI_LENGTH 256

#define INFRARED_THRESHOLD (0.5f)

const uint8_t piRxFrameHeader = 0xbb;
inline const float bat_k = (95.3f+5.6f)/5.6f;
inline const float cap_k = 113.73;

#ifdef __cplusplus
}

#include <cstdint>

// Monotonic microsecond timebase shared by the IMU, SPI and control tasks.
// short interrupt mask so 800 Hz sensor samples keep distinct timestamps.
inline std::uint64_t motion_now_us() {
    static std::uint32_t last_count = 0;
    static std::uint64_t base = 0;
    const std::uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const std::uint32_t count = TIM13->CNT;
    if (count < last_count) base += 65536u;
    last_count = count;
    const std::uint64_t now = base + count;
    __set_PRIMASK(primask);
    return now;
}


// Component includes
#include "Component/wheel_motor.hpp"
#include "Component/imu.hpp"
#include "Component/opt_flow.hpp"
#include "Component/robot.hpp"

// Task includes
#include "Communication/can/z_can.hpp"

// Forward declarations to ensure types are known before externs
class Robot;

extern Robot robot;

extern ZCAN can1_bus;
extern ZCAN can2_bus;
extern IMU imu;
extern OptFlow opt_flow;


#endif // __cplusplus

#endif // __Z_MAIN_H
