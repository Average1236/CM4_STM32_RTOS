#ifndef __CONTROL_PARAMS_HPP
#define __CONTROL_PARAMS_HPP

#include "Task/z_main.h"

namespace control_config {

// Dribbler control mode: true = torque control (CAN ID 0xAD), false = velocity control (CAN ID 0xAC)
inline constexpr bool kDribblerTorqueMode = true;

inline constexpr float kControlDtSec = static_cast<float>(TIM2_PERIOD_CLOCKS) / 1000000.0f;
inline constexpr float kImuDtSec = static_cast<float>(TIM7_PERIOD_CLOCKS) / 1000000.0f;
inline constexpr float kPi = 3.1415926535f;

// DEPRECATED — no longer used by the incremental yaw reference tracker.
// Kept for reference only.
inline constexpr float kYawTargetReplanDeadbandRad = 1e-2f;

// Incremental yaw reference tracker
inline constexpr float kYawTargetStopBandRad = 1e-3f;
inline constexpr float kYawTargetVelZeroEpsRadS = 0.02f;

inline constexpr float kRobotMassKg = 3.0f;
inline constexpr float kRobotInertiaKgM2 = 8e-3f;
inline constexpr float kWheelRadiusM = 0.03f;
inline constexpr float kWheelCenterDistanceM = 0.0785f;
inline constexpr float kCenterToComDistanceM = 0.0f;
inline constexpr float kWheelAlphaRad = 30.0f / 180.0f * kPi;
inline constexpr float kWheelBetaRad = 45.0f / 180.0f * kPi;

inline constexpr float kLesoAngleObserverBandwidth = 200.0f;

// Velocity-scheduled LESO bandwidth limits — high when moving (ground),
// low when near-zero speed (airborne oscillation suppression).
inline constexpr float kLesoAngleBandwidthMin = 100.0f;
inline constexpr float kLesoScheduleVelocityThreshold = 0.01f;  // m/s
inline constexpr float kLesoScheduleOmegaThreshold = 0.01f;    // rad/s
inline constexpr float kImuOmegaButterworthCutoffHz = 0.0f;
inline constexpr float kChassisOmegaZFilterCutoffHz = 0.0f;
inline constexpr float kImuOmegaBiasZ = 1.0f;

// Velocity feedback gains — scheduled with LESO bandwidth via alpha_v.
// Kv = ω_c for 1st-order velocity loop pole placement.
inline constexpr float kChassisVelPidKpX = 30.0f;
inline constexpr float kChassisVelPidKiX = 300.0f;
inline constexpr float kChassisVelPidKdX = 0.0f;
inline constexpr float kChassisVelPidOutputLimitX = 10.0f;  // m/s^2
inline constexpr float kChassisVelPidIntegLimitX = 5.0f;   // m/s^2 contribution
inline constexpr float kChassisVelPidBackCalcGainX = 0.3f;
inline constexpr float kChassisVelPidDiffCutoffHzX = 0.0f;
inline constexpr float kChassisVelPidKpY = 30.0f;
inline constexpr float kChassisVelPidKiY = 300.0f;
inline constexpr float kChassisVelPidKdY = 0.0f;
inline constexpr float kChassisVelPidOutputLimitY = 10.0f;  // m/s^2
inline constexpr float kChassisVelPidIntegLimitY = 5.0f;   // m/s^2 contribution
inline constexpr float kChassisVelPidBackCalcGainY = 0.3f;
inline constexpr float kChassisVelPidDiffCutoffHzY = 0.0f;
inline constexpr float kVelFeedbackGainYaw = 0.0f;
// PD controller bandwidth — scheduled together with LESO bandwidth.
// max when moving (stiff angle tracking), min when stationary (stable).
inline constexpr float kAngleControllerBandwidth = 50.0f;
inline constexpr float kAngleControllerBandwidthMin = 50.0f;

// S-curve yaw target planner
inline constexpr float kYawAngleLinearFallbackMinTimeSec = 0.05f;
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
inline constexpr float kMITRunKd = 0.0f;
inline constexpr float kMITRunTorqueFf = 0.0f;
inline constexpr float kMITKdRampTimeSec = 1.0f;

inline constexpr float kMITSafeKp = 0.0f;
inline constexpr float kMITSafeKd = 0.0f;
inline constexpr float kMITSafeTorqueFf = 0.0f;

inline constexpr float kMotorRecoverDelayMs = 1000.0f;
inline constexpr float kMotorClearErrorToEnableDelayMs = 10.0f;

// ---- Optical Flow PLL ----
inline constexpr float kOptFlowPllBandwidthHz = 300.0f;
inline constexpr float kOptFlowPllZeroSnapMmPerS = 1.0f;
inline constexpr float kOptFlowPllMaxJumpMm = 100.0f;
inline constexpr uint8_t kOptFlowPllJumpConfirmFrames = 3;

// ---- Six-Axis Stationary Detection ----
inline constexpr float kStationaryAccelVarThreshold = 0.15f;
inline constexpr float kStationaryGyroThresholdDegPerS = 1.5f;
inline constexpr float kStationaryFlowThresholdMmPerS = 10.0f;
inline constexpr uint8_t kStationaryConfirmFrames = 10;
inline constexpr uint8_t kStationaryWindowFrames = 50;
inline constexpr float kImuBiasAlpha = 0.02f;
inline constexpr float kImuBiasAccelNormTolerance = 0.45f;       // m/s^2
inline constexpr float kImuBiasAccelVarThreshold = 0.08f;        // m/s^2 per sample
inline constexpr float kImuBiasAccelStdMax = 0.10f;              // m/s^2 over window
inline constexpr float kImuBiasGyroStillThresholdDegPerS = 1.5f; // deg/s
inline constexpr float kImuBiasGyroStdMaxDegPerS = 0.20f;        // deg/s over window
inline constexpr float kImuBiasMaxAbsDegPerS = 3.0f;             // candidate reject
inline constexpr float kImuBiasMaxUpdateStepDegPerS = 0.05f;     // per accepted window
inline constexpr uint16_t kImuBiasConfirmFrames = 200;           // 0.25s at 800Hz
inline constexpr uint16_t kImuBiasWindowFrames = 400;            // 0.50s at 800Hz
inline constexpr uint16_t kImuBiasStartupIgnoreFrames = 2400;     // 3.0s at 800Hz
inline constexpr uint8_t kImuBiasValidWindows = 2;                // require stable verified windows before applying bias

// ---- Simplified Kalman ----
inline constexpr float kOptFlowKfQVel = 20.0f;
inline constexpr float kOptFlowKfRVelFixed = 300.0f;
inline constexpr float kOptFlowVelocityLimitMS = 5.0f;
inline constexpr float kOptFlowAccelLimitMS2 = 10.0f;
inline constexpr uint32_t kOptFlowSensorStaleTimeoutMs = 100;

// ---- Chassis Velocity Source ----
// 0: wheel-based (Jacobian pseudo-inverse from motor velocities)
// 1: optical-flow-based (kf_vx / kf_vy from OptFlow)
// 2: fused (1-state Kalman, adaptive R per speed + tilt)
// 3: optflow + raw vision velocity fused by 1-state Kalman
// 4: wheel + raw vision velocity fused by 1-state Kalman
inline constexpr uint8_t kChassisVelocitySource = 2;

// ---- Fusion Kalman (source==2) ----
// Process noise
inline constexpr float kFusionKalmanQX = 0.01f;
inline constexpr float kFusionKalmanQY = 0.01f;

// Wheel measurement noise: high at low speed, low at high speed
inline constexpr float kFusionKalmanRWheelMinX = 0.01f;
inline constexpr float kFusionKalmanRWheelMaxX = 0.05f;
inline constexpr float kFusionKalmanRWheelMinY = 0.05f;
inline constexpr float kFusionKalmanRWheelMaxY = 0.2f;

// Optflow measurement noise: low at low speed, high at high speed
inline constexpr float kFusionKalmanROptflowMinX = 0.01f;
inline constexpr float kFusionKalmanROptflowMaxX = 0.01f;
inline constexpr float kFusionKalmanROptflowMinY = 0.01f;
inline constexpr float kFusionKalmanROptflowMaxY = 0.01f;

// Speed transition point (m/s)
inline constexpr float kFusionSpeedTransitionX = 6.0f;
inline constexpr float kFusionSpeedTransitionY = 6.0f;

// Tilt penalty gain (per deg/s)
inline constexpr float kFusionTiltPenaltyGainX = 10.0f;
inline constexpr float kFusionTiltPenaltyGainY = 10.0f;

// Wheel slip suppression: increase wheel velocity measurement noise when the
// planner asks for large acceleration or wheel/optflow residuals diverge.
inline constexpr float kFusionWheelAccelThresholdX = 1.5f; // m/s^2
inline constexpr float kFusionWheelAccelThresholdY = 1.5f; // m/s^2
inline constexpr float kFusionWheelAccelPenaltyGainX = 0.8f;
inline constexpr float kFusionWheelAccelPenaltyGainY = 0.8f;
inline constexpr float kFusionWheelSlipAccelThresholdX = 3.0f; // m/s^2
inline constexpr float kFusionWheelSlipAccelThresholdY = 3.0f; // m/s^2
inline constexpr float kFusionWheelOptflowResidualThresholdX = 1.0f; // m/s
inline constexpr float kFusionWheelOptflowResidualThresholdY = 1.0f; // m/s
inline constexpr float kFusionWheelSlipResidualPenaltyGainX = 0.0f;
inline constexpr float kFusionWheelSlipResidualPenaltyGainY = 0.0f;
inline constexpr float kWheelChassisVxScale = 0.8f;   // wheel-based vx scaling
inline constexpr float kWheelChassisVyScale = 0.5f;   // wheel-based vy scaling
inline constexpr float kWheelChassisVelocityLimitMS = 5.0f;
inline constexpr float kWheelChassisAccelLimitMS2 = 10.0f;

// ---- Optflow + Vision Velocity Fusion (runtime vision_source == 3) ----
inline constexpr float kVisionOptflowFusionQX = 0.02f;
inline constexpr float kVisionOptflowFusionQY = 0.02f;
inline constexpr float kVisionOptflowFusionROptflowX = 0.05f;
inline constexpr float kVisionOptflowFusionROptflowY = 0.05f;
inline constexpr float kVisionOptflowFusionRVisionX = 0.005f;
inline constexpr float kVisionOptflowFusionRVisionY = 0.005f;

// ---- Wheel + Vision Velocity Fusion (runtime vision_source == 4) ----
inline constexpr float kVisionWheelFusionQX = 0.02f;
inline constexpr float kVisionWheelFusionQY = 0.02f;
inline constexpr float kVisionWheelFusionRWheelX = 0.02f;
inline constexpr float kVisionWheelFusionRWheelY = 0.02f;
inline constexpr float kVisionWheelFusionRVisionX = 0.02f;
inline constexpr float kVisionWheelFusionRVisionY = 0.02f;

// ---- Test Kick ----
// true:  bypass infrared sensor, rising-edge trigger on kick_discharge_time
// false: require infrared ball detection (INFRARED_THRESHOLD)
inline constexpr bool kTestKick = false;

// ---- IMU Roll/Pitch Complementary Filter ----
inline constexpr float kImuRollPitchAlpha = 0.98f;

} // namespace control_config

#endif // __CONTROL_PARAMS_HPP
