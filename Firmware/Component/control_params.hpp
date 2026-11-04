#ifndef CONTROL_PARAMS_HPP
#define CONTROL_PARAMS_HPP
#include "Task/z_main.h"
#include "motion/common.hpp"
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
inline constexpr float kRobotMassKg           = motion::RobotModel{}.mass_kg;
inline constexpr float kRobotInertiaKgM2      = motion::RobotModel{}.inertia_kg_m2;
inline constexpr float kWheelRadiusM          = motion::RobotModel{}.wheel_radius_m;
inline constexpr float kWheelCenterDistanceM  = motion::RobotModel{}.wheel_center_distance_m;
inline constexpr float kWheelSpeedPllBandwidth      = 75.0f;
inline constexpr float kWheelSpeedPllZeroSnapEpsRpm = 0.1f;
inline constexpr float kWheelInertiaKgM2              = motion::RobotModel{}.wheel_inertia_kg_m2;
inline constexpr float kWheelViscousDampingNmPerRadPS = motion::RobotModel{}.wheel_viscous_nm_per_rad_s;
inline constexpr float kMotorRecoverDelayMs           = 1000.0f;
inline constexpr float kMotorClearErrorToEnableDelayMs = 10.0f;
}
#endif
