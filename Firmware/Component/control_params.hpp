#ifndef __CONTROL_PARAMS_HPP
#define __CONTROL_PARAMS_HPP

#include "Task/z_main.h"

namespace control_config {

inline constexpr float kControlDtSec = static_cast<float>(TIM2_PERIOD_CLOCKS) / 1000000.0f;
inline constexpr float kImuDtSec = static_cast<float>(TIM7_PERIOD_CLOCKS) / 1000000.0f;
inline constexpr float kPi = 3.1415926535f;

inline constexpr float kAccThresholdX = 15.0f;
inline constexpr float kAccThresholdY = 9.0f;
inline constexpr float kAccThresholdYaw = 40.0f;

inline constexpr float kJerkLimitX = 300.0f;
inline constexpr float kJerkLimitY = 100.0f;
inline constexpr float kJerkLimitYaw = 600.0f;

inline constexpr float kRobotMassKg = 2.25f;
inline constexpr float kRobotInertiaKgM2 = 8e-3f;
inline constexpr float kWheelRadiusM = 0.033f;
inline constexpr float kWheelCenterDistanceM = 0.07956f;
inline constexpr float kCenterToComDistanceM = -0.01f;
inline constexpr float kWheelAlphaRad = 30.0f / 180.0f * kPi;
inline constexpr float kWheelBetaRad = 45.0f / 180.0f * kPi;

inline constexpr float kLesoVelObserverBandwidth = 60.0f;
inline constexpr float kLesoAngleObserverBandwidth = 150.0f;

// Velocity-scheduled LESO bandwidth limits — high when moving (ground),
// low when near-zero speed (airborne oscillation suppression).
inline constexpr float kLesoVelBandwidthMin = 0.0f;
inline constexpr float kLesoAngleBandwidthMin = 0.0f;
inline constexpr float kLesoScheduleVelocityThreshold = 0.01f;  // m/s
inline constexpr float kLesoScheduleOmegaThreshold = 0.01f;    // rad/s
inline constexpr float kImuOmegaButterworthCutoffHz = 350.0f;
inline constexpr float kChassisOmegaZFilterCutoffHz = 0.0f;
inline constexpr float kImuOmegaBiasZ = 1.0f;

// Velocity feedback gains — scheduled with LESO bandwidth via alpha_v.
// Kv = ω_c for 1st-order velocity loop pole placement.
inline constexpr float kVelFeedbackGainX = 8.0f;
inline constexpr float kVelFeedbackGainY = 2.0f;
inline constexpr float kVelFeedbackGainMinX = 1.0f;
inline constexpr float kVelFeedbackGainMinY = 1.0f;
inline constexpr float kVelFeedbackGainYaw = 0.0f;
// PD controller bandwidth — scheduled together with LESO bandwidth.
// max when moving (stiff angle tracking), min when stationary (stable).
inline constexpr float kAngleControllerBandwidth = 50.0f;
inline constexpr float kAngleControllerBandwidthMin = 0.0f;
inline constexpr float kYawDesiredOmegaGain = 10.0f;

// S-curve yaw target planner
inline constexpr float kYawSCurveVmax = 40.0f;   // rad/s
inline constexpr float kYawSCurveAmax = 40.0f;  // rad/s²
inline constexpr float kYawSCurveJmax = 1000.0f; // rad/s³
inline constexpr float kYawVyCoupling = 0.0f;

inline constexpr float kWheelTorqueFfLimitNm = 0.7f;

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

// Virtual damping — high near zero speed (airborne oscillation suppression),
// low at high speed (natural friction provides damping).
inline constexpr float kWheelVirtualDampingNmPerRadPS = 0.027f;
inline constexpr float kWheelVirtualDampingMin = 0.01f;
inline constexpr float kWheelDampScheduleSpeedRadPS = 10.0f;
inline constexpr float kWheelVirtualDampingLimitNm = 0.4f;

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