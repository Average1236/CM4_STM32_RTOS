#ifndef CONTROL_PARAMS_HPP
#define CONTROL_PARAMS_HPP
#include "Task/z_main.h"
namespace control_config {
inline constexpr uint8_t kDribblerModeTorque = 1;
inline constexpr uint8_t kDribblerModeSpeed = 2;
inline constexpr uint8_t kDribblerModeHybrid = 3;
inline constexpr float kDribblerHybridTorqueNm = -0.05f;
inline constexpr float kDribblerHybridSpeedRps = -120.0f;
inline constexpr float kDribblerHybridTorqueLimitNm = 0.15f;
inline constexpr uint32_t kDribblerHybridBallHoldFrames = 20;
inline constexpr uint32_t kDribblerHybridBallLostFrames = 10;
inline constexpr uint32_t kDribblerHybridDebounceStep = 2;
inline constexpr float kControlDtSec = static_cast<float>(TIM2_PERIOD_CLOCKS) / 1000000.0f;
inline constexpr float kPi           = 3.1415926535f;
inline constexpr float kRobotMassKg           = 3.3f;
inline constexpr float kRobotInertiaKgM2      = 8e-3f;
inline constexpr float kWheelRadiusM          = 0.033f;
inline constexpr float kWheelCenterDistanceM  = 0.0785f;
inline constexpr float kWheelSpeedPllBandwidth      = 75.0f;
inline constexpr float kWheelSpeedPllZeroSnapEpsRpm = 0.1f;
inline constexpr float kWheelInertiaKgM2              = 5e-5f;
inline constexpr float kWheelViscousDampingNmPerRadPS = 6.5e-5f;
inline constexpr float kMotorRecoverDelayMs           = 1000.0f;
inline constexpr float kMotorClearErrorToEnableDelayMs = 10.0f;
}
#endif
