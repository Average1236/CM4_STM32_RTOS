#ifndef __CONTROL_PARAMS_HPP
#define __CONTROL_PARAMS_HPP

#include "Task/z_main.h"

namespace control_config {

inline constexpr float kControlDtSec = static_cast<float>(TIM2_PERIOD_CLOCKS) / 1000000.0f;
inline constexpr float kPi = 3.1415926535f;

inline constexpr float kAccThresholdX = 6.5f;
inline constexpr float kAccThresholdY = 5.0f;
inline constexpr float kAccThresholdYaw = 40.0f;

inline constexpr float kJerkLimitX = 200.0f;
inline constexpr float kJerkLimitY = 90.0f;
inline constexpr float kJerkLimitYaw = 600.0f;

inline constexpr float kRobotMassKg = 1.99692f;
inline constexpr float kRobotInertiaKgM2 = 2e-2f;
inline constexpr float kWheelRadiusM = 0.033f;
inline constexpr float kWheelCenterDistanceM = 0.07956f;
inline constexpr float kCenterToComDistanceM = 0.0f;
inline constexpr float kWheelAlphaRad = 30.0f / 180.0f * kPi;
inline constexpr float kWheelBetaRad = 45.0f / 180.0f * kPi;

inline constexpr float kLesoVelObserverBandwidth = 60.0f;
inline constexpr float kLesoOmegaObserverBandwidth = 150.0f;
inline constexpr float kLeso3rdOrderBandwidth = 150.0f;
inline constexpr float kImuOmegaButterworthCutoffHz = 400.0f;
inline constexpr float kImuOmegaBiasZ = 1.0f;

inline constexpr float kVelFeedbackGainX = 0.0f;
inline constexpr float kVelFeedbackGainY = 0.0f;
inline constexpr float kVelFeedbackGainYaw = 450.0f;
inline constexpr float kPosFeedbackGainYaw = 450.0f;
inline constexpr float kYawDesiredOmegaGain = 10.0f;
inline constexpr float kYawTDR = 100.0f;
inline constexpr float kYawTDH = 0.02f;
inline constexpr float kYawTDDiffGain = 0.0f;
inline constexpr float kYawStartupRampTimeSec = 1.5f;

inline constexpr float kWheelTorqueFfLimitNm = 0.1f;

inline constexpr float kWheelSpeedPidKp = 0.03f;
inline constexpr float kWheelSpeedPidKi = 0.6f;
inline constexpr float kWheelSpeedPidKd = 0.0f;
inline constexpr float kWheelSpeedPidBackCalcGain = 0.3f;
inline constexpr float kWheelSpeedPidDiffCutoffHz = 50.0f;
inline constexpr float kWheelSpeedPidOutputLimitNm = 0.0f;
inline constexpr float kWheelSpeedPidIntegLimitNm = 0.0f;
inline constexpr float kWheelSpeedPidKpRampTimeSec = 1.5f;
inline constexpr float kWheelSpeedPllBandwidth = 75.0f;
inline constexpr float kWheelSpeedPllZeroSnapEpsRpm = 0.1f;
inline constexpr float kWheelSpeedPllOmegaRampTimeSec = 0.2f;

// Wheels mechanical model
inline constexpr float kWheelInertiaKgM2 = 5e-5f;
inline constexpr float kWheelViscousDampingNmPerRadPS = 6.5e-5f;

// 3-state Luenberger observer bandwidth (rad/s, triple pole).
// L1 = 3·ω_o·dt, must be < 1.0 for stable Euler discretization.
// dt=0.002s (500Hz): ω_o=100 → L1=0.6 (safe).
inline constexpr float kWheelObsVelocityBandwidth = 100.0f;

// Virtual damping gain (Nm per rad/s).
inline constexpr float kWheelVirtualDampingNmPerRadPS = 0.027f;

// Butterworth on observer velocity for damping (Hz).
// Heavy filtering emulates dmiao driver's sluggish velocity loop,
// decoupling the fast inner damping from the slower chassis control.
inline constexpr float kWheelObsVelocityButterworthCutoffHz = 100.0f;

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