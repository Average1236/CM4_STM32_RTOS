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

inline constexpr float kAccThresholdX = 6.5f;
inline constexpr float kAccThresholdY = 6.5f;
inline constexpr float kAccThresholdYaw = 40.0f;

inline constexpr float kJerkLimitX = 175.0f;
inline constexpr float kJerkLimitY = 175.0f;
inline constexpr float kJerkLimitYaw = 600.0f;

inline constexpr float kRobotMassKg = 1.99692f;
inline constexpr float kRobotInertiaKgM2 = 3e-3f;
inline constexpr float kWheelRadiusM = 0.033f;
inline constexpr float kWheelCenterDistanceM = 0.07956f;
inline constexpr float kCenterToComDistanceM = 0.0f;
inline constexpr float kWheelAlphaRad = 30.0f / 180.0f * kPi;
inline constexpr float kWheelBetaRad = 45.0f / 180.0f * kPi;

inline constexpr float kLesoVelObserverBandwidth = 100.0f;
inline constexpr float kLesoOmegaObserverBandwidth = 300.0f;
inline constexpr ChassisOmegaZSource kChassisOmegaZSource = ChassisOmegaZSource::kImuOmegaDirect;
inline constexpr float kImuYawPllBandwidth = 200.0f;
inline constexpr float kImuYawPllZeroSnapEpsRadS = 0.1f;
inline constexpr float kImuYawPllOmegaRampTimeSec = 0.8f;
inline constexpr float kImuOmegaButterworthCutoffHz = 400.0f;

inline constexpr float kVelFeedbackGainX = 20.0f;
inline constexpr float kVelFeedbackGainY = 20.0f;
inline constexpr float kVelFeedbackGainYaw = 450.0f;

inline constexpr float kWheelTorqueFfLimitNm = 0.0f;

inline constexpr float kWheelSpeedPidKp = 0.025f;
inline constexpr float kWheelSpeedPidKi = 0.0f;
inline constexpr float kWheelSpeedPidKd = 0.01f;
inline constexpr float kWheelSpeedPidBackCalcGain = 0.2f;
inline constexpr float kWheelSpeedPidDiffCutoffHz = 200.0f;
inline constexpr float kWheelSpeedPidOutputLimitNm = 0.2f;
inline constexpr float kWheelSpeedPidIntegLimitNm = 0.0f;
inline constexpr float kWheelSpeedPidKpRampTimeSec = 1.5f;
inline constexpr float kWheelSpeedPllBandwidth = 200.0f;
inline constexpr float kWheelSpeedPllZeroSnapEpsRpm = 0.0f;
inline constexpr float kWheelSpeedPllOmegaRampTimeSec = 0.2f;

inline constexpr float kMITRunKp = 0.0f;
// inline constexpr float kMITRunKd = 0.07f;
inline constexpr float kMITRunKd = 0.0f;
inline constexpr float kMITRunTorqueFf = 0.0f;
inline constexpr float kMITKdRampTimeSec = 1.0f;

inline constexpr float kMITSafeKp = 0.0f;
inline constexpr float kMITSafeKd = 0.0f;
inline constexpr float kMITSafeTorqueFf = 0.0f;

inline constexpr float kMotorRecoverDelayMs = 1000.0f;
inline constexpr float kMotorClearErrorToEnableDelayMs = 10.0f;

} // namespace control_config

#endif // __CONTROL_PARAMS_HPP