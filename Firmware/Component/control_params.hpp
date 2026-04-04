#ifndef __CONTROL_PARAMS_HPP
#define __CONTROL_PARAMS_HPP

#include "Task/z_main.h"

namespace control_config {

enum class ChassisOmegaZSource : uint8_t {
	kYawPll = 0,
	kImuOmegaDirect = 1,
};

inline constexpr float kControlDtSec = static_cast<float>(TIM2_PERIOD_CLOCKS) / 1000000.0f;
inline constexpr float kPi = 3.1415926535f;

inline constexpr float kAccThresholdX = 6.0f;
inline constexpr float kAccThresholdY = 6.0f;
inline constexpr float kAccThresholdYaw = 40.0f;

inline constexpr float kJerkLimitX = 50.0f;
inline constexpr float kJerkLimitY = 50.0f;
inline constexpr float kJerkLimitYaw = 600.0f;

inline constexpr float kRobotMassKg = 2.19692f;
inline constexpr float kRobotInertiaKgM2 = 2e-2f;
inline constexpr float kWheelRadiusM = 0.033f;
inline constexpr float kWheelCenterDistanceM = 0.07956f;
inline constexpr float kCenterToComDistanceM = 0.0f;
inline constexpr float kWheelAlphaRad = 30.0f / 180.0f * kPi;
inline constexpr float kWheelBetaRad = 45.0f / 180.0f * kPi;

inline constexpr float kLesoVelObserverBandwidth = 50.0f;
inline constexpr float kLesoYawObserverBandwidth = 30.0f;
inline constexpr float kImuYawPllBandwidth = 30.0f;
inline constexpr float kImuYawPllZeroSnapEpsRadS = 0.1f;
inline constexpr float kImuYawPllOmegaRampTimeSec = 0.8f;
inline constexpr ChassisOmegaZSource kChassisOmegaZSource = ChassisOmegaZSource::kYawPll;

inline constexpr float kVelFeedbackGainX = 10.0f;
inline constexpr float kVelFeedbackGainY = 10.0f;
inline constexpr float kVelFeedbackGainYaw = 100.0f;
inline constexpr float kVelFeedbackDGainYaw = 10.0f;

inline constexpr float kWheelTorqueFfLimitNm = 0.60f;

inline constexpr float kWheelSpeedPidKp = 0.4f;
inline constexpr float kWheelSpeedPidKi = 0.0f;
inline constexpr float kWheelSpeedPidKd = 0.0f;
inline constexpr float kWheelSpeedPidBackCalcGain = 0.2f;
inline constexpr float kWheelSpeedPidOutputLimitNm = 0.0f;
inline constexpr float kWheelSpeedPidIntegLimitNm = 0.0f;
inline constexpr float kWheelSpeedPidKpRampTimeSec = 1.5f;
inline constexpr float kWheelSpeedPllBandwidth = 75.0f;
inline constexpr float kWheelSpeedPllZeroSnapEpsRpm = 0.1f;
inline constexpr float kWheelSpeedPllOmegaRampTimeSec = 0.2f;

inline constexpr float kMITRunKp = 0.0f;
// inline constexpr float kMITRunKd = 0.07f;
inline constexpr float kMITRunKd = 0.04f;
inline constexpr float kMITRunTorqueFf = 0.0f;
inline constexpr float kMITKdRampTimeSec = 1.0f;

inline constexpr float kMITSafeKp = 0.0f;
inline constexpr float kMITSafeKd = 0.0f;
inline constexpr float kMITSafeTorqueFf = 0.0f;

inline constexpr float kMotorRecoverDelayMs = 1000.0f;
inline constexpr float kMotorClearErrorToEnableDelayMs = 10.0f;

} // namespace control_config

#endif // __CONTROL_PARAMS_HPP