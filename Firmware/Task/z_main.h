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
#define SPI_LENGTH 96

#define INFRARED_THRESHOLD (0.5f)

const uint8_t piRxFrameHeader = 0xbb;
const float bat_k = 106.04f / 6.04f;
const float cap_k = 113.73;

#ifdef __cplusplus
}

#include "interfaces.hpp"

// Component includes
#include "Component/wheel_motor.hpp"
#include "Component/dribble_motor.hpp"
#include "Component/imu.hpp"
#include "Component/opt_flow.hpp"
#include "Component/robot.hpp"

// Task includes
#include "Task/utils.hpp"
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
