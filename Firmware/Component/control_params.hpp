#ifndef __CONTROL_PARAMS_HPP
#define __CONTROL_PARAMS_HPP

#include "Task/z_main.h"

namespace control_config {

// Dribbler control mode flags (from CM4 drib_mode byte over SPI)
// 1 = torque control, 2 = velocity control, 3 = hybrid
inline constexpr uint8_t kDribblerModeTorque = 1;
inline constexpr uint8_t kDribblerModeSpeed = 2;
inline constexpr uint8_t kDribblerModeHybrid = 3;

inline constexpr float kDribblerHybridTorqueNm = -0.05f;
inline constexpr float kDribblerHybridSpeedRps = -120.0f;
inline constexpr float kDribblerHybridTorqueLimitNm = 0.15f;
inline constexpr uint32_t kDribblerHybridBallHoldFrames = 20;
inline constexpr uint32_t kDribblerHybridBallLostFrames = 10;
inline constexpr uint32_t kDribblerHybridDebounceStep = 2;

inline constexpr float kDribblerSpeedBaseRps = -50.0f;
inline constexpr float kDribblerSpeedCompensateGain = 23.0f;
inline constexpr float kDribblerSpeedSlipMargin = 1.3f;
inline constexpr float kDribblerSpeedDeadZone = -0.05f;
inline constexpr float kDribblerSpeedSafetyClamp = -120.0f;

// ==========================================================================
//  1. System Constants
// --------------------------------------------------------------------------
//  Derived from hardware timers.  Do not hand-edit.
// ==========================================================================

inline constexpr float kControlDtSec = static_cast<float>(TIM2_PERIOD_CLOCKS) / 1000000.0f;
inline constexpr float kImuDtSec     = static_cast<float>(TIM7_PERIOD_CLOCKS) / 1000000.0f;
inline constexpr float kPi           = 3.1415926535f;


// ==========================================================================
//  2. Robot Physical Model
// --------------------------------------------------------------------------
//  Mass, inertia, wheel geometry.  Only change when hardware is modified.
// ==========================================================================

inline constexpr float kRobotMassKg           = 3.3f;
inline constexpr float kRobotInertiaKgM2      = 8e-3f;
inline constexpr float kWheelRadiusM          = 0.033f;
inline constexpr float kWheelCenterDistanceM  = 0.0785f;
inline constexpr float kCenterToComDistanceM  = 0.0f;
inline constexpr float kWheelAlphaRad         = 30.0f / 180.0f * kPi;
inline constexpr float kWheelBetaRad          = 45.0f / 180.0f * kPi;


// ==========================================================================
//  3. Yaw Control  —  Angle PID → Rate LADRC  (ACTIVE PATH)
// --------------------------------------------------------------------------
//  Replaces the old YawSCurve-based planner.
//
//  Data flow:
//    wrapped target → CircularLPF → Angle PID (in ChassisController) → ω_ref
//                   → Inner Rate LADRC (ChassisController LESO) → F_task_ψ
//
//  Tuning order:  inner rate → outer angle PID → LPF cutoff
// ==========================================================================

// ---- 3a. Target low-pass filter ----
// Smooths the wrapped yaw target from vision / host.
// ↑ = faster response to target changes   ↓ = less jitter
inline constexpr float kYawTargetLowPassCutoffHz = 8.0f;

// ---- 3b. Outer Angle PID (in ChassisController) ----
// PID on wrapped angle error → ω_ref, clamped to yaw_max_vel.
// D-term uses actual angular velocity (derivative-on-measurement, no kick).
//
// Kp: proportional gain (rad/s per rad error)
//   e.g. 0.1 rad error × Kp=100 → 10 rad/s ω_ref
//   ↑ = stiffer angle tracking   ↓ = softer, less oscillation
//
// Ki: integral gain (rad/s per rad·s)
//   Eliminates steady-state error from unmodeled friction / bias.
//   ↑ = faster bias rejection    ↓ = less windup risk
//
// Kd: derivative gain on ω_z (dimensionless damping)
//   D = -Kd * ω_z_meas  (derivative-on-measurement)
//   ↑ = more damping, less overshoot   ↓ = livelier response
inline constexpr float kYawAnglePidKp = 25.0f;
inline constexpr float kYawAnglePidKi = 1.5f;
inline constexpr float kYawAnglePidKd = 0.0f;

// ---- 3c. Wheel-speed fallback outer angle PID (in Robot::prepare_yaw_control) ----
// Separate from the torque-control yaw PID so wheel-speed fallback can be tuned
// without changing the active torque/LADRC yaw path.
inline constexpr float kYawFallbackAnglePidKp = 10.0f;
inline constexpr float kYawFallbackAnglePidKi = 2.0f;
inline constexpr float kYawFallbackAnglePidKd = 0.3f;
inline constexpr float kYawFallbackOmegaZFilterCutoffHz = 50.0f;

// ---- 3d. Vy coupling feedforward ----
// Feeds forward vy reference into yaw torque (Coriolis-like coupling).
// 0.0 = disabled.  Non-zero when lateral motion affects yaw noticeably.
inline constexpr float kYawVyCoupling = 0.0f;


// ==========================================================================
//  4. Yaw  —  Inner Omega LESO & Rate Controller (ChassisController)
// --------------------------------------------------------------------------
//  2nd-order Linear Extended State Observer for angular velocity.
//  Bandwidth is velocity-scheduled between min (airborne) and max (ground).
//  P + disturbance-rejection control law on ω_z.
// ==========================================================================

// ---- 4a. LESO bandwidth scheduling ----
inline constexpr float kLesoAngleObserverBandwidth  = 250.0f;   // max bandwidth (rad/s)
inline constexpr float kLesoAngleBandwidthMin       = 250.0f;   // min bandwidth (rad/s)
inline constexpr float kLesoScheduleVelocityThreshold = 0.01f;  // scheduling knee (m/s)
inline constexpr float kLesoScheduleOmegaThreshold    = 0.01f;  // scheduling knee (rad/s)

// ---- 4b. Rate P controller ----
// P-gain from ω_z error → yaw torque (rad/s).
// Control law: F_task_ψ = I · (wc_rate · (ω_ref − ω_z_est) − disturbance + coupling)
//   ↑ = tighter velocity tracking        ↓ = less motor saturation
//   Rule of thumb: wc_rate ≤ inner_wo / 3
inline constexpr float kYawRateControllerBandwidth = 75.0f;

// ---- 4c. Omega-z / gyro pre-filtering (Butterworth 2nd-order) ----
// 0.0 = passthrough.  Non-zero adds delay, use only if gyro is noisy.
inline constexpr float kChassisOmegaZFilterCutoffHz  = 0.0f;
inline constexpr float kImuOmegaButterworthCutoffHz  = 0.0f;

// Gyro-Z bias correction factor (1.0 = raw, < 1.0 = attenuate)
inline constexpr float kImuOmegaBiasZ = 1.0f;

// Non-IMU fallback: velocity feedback gain on ω_z
// 0.0 = pure feedforward (no feedback correction)
inline constexpr float kVelFeedbackGainYaw = 0.0f;


// ==========================================================================
//  5. Chassis Velocity Control  —  vx / vy PID
// --------------------------------------------------------------------------
//  Path: SPI velocity target → Butterworth LPF → PID plus host acceleration feedforward.
//  No TD-based acceleration planner. Feedforward uses CmdVel acceleration_x/y.
//  PID output (m/s²) is directly scaled to force by mass.
// ==========================================================================

// ---- vx ----
inline constexpr float kChassisVxRefButterworthCutoffHz = 5.0f;   // target LPF (Hz)
inline constexpr float kChassisVelPidKpX            = 30.0f;
inline constexpr float kChassisVelPidKiX            = 300.0f;
inline constexpr float kChassisVelPidKdX            = 0.0f;
inline constexpr float kChassisVelPidOutputLimitX   = 20.0f;   // m/s²
inline constexpr float kChassisVelPidIntegLimitX    = 20.0f;    // m/s² contribution
inline constexpr float kChassisVelPidBackCalcGainX  = 0.3f;
inline constexpr float kChassisVelPidDiffCutoffHzX  = 30.0f;

// ---- vy ----
inline constexpr float kChassisVyRefButterworthCutoffHz = 5.0f;   // target LPF (Hz)
inline constexpr float kChassisVelPidKpY            = 30.0f;
inline constexpr float kChassisVelPidKiY            = 300.0f;
inline constexpr float kChassisVelPidKdY            = 0.0f;
inline constexpr float kChassisVelPidOutputLimitY   = 20.0f;   // m/s²
inline constexpr float kChassisVelPidIntegLimitY    = 20.0f;    // m/s² contribution
inline constexpr float kChassisVelPidBackCalcGainY  = 0.3f;
inline constexpr float kChassisVelPidDiffCutoffHzY  = 30.0f;

// Feedforward acceleration Butterworth LPF cutoff (Hz).
// Smooths host acceleration feedforward; set to 0 to disable.
inline constexpr float kChassisAccFfButterworthCutoffHz = 20.0f;

// Wheel-speed PID fallback disables chassis torque feedforward and closes
// yaw angle through wheel velocity commands instead.
inline constexpr bool kUseWheelSpeedPidFallback = false;

// Yaw torque feedforward limit per wheel (Nm)
inline constexpr float kWheelTorqueFfLimitNm = kUseWheelSpeedPidFallback ? 0.0f : 0.7f;


// ==========================================================================
//  6. Wheel Motor Control
// --------------------------------------------------------------------------
//  Per-motor speed PID, velocity observer (PLL), virtual damping, MIT mode.
// ==========================================================================

// ---- 6a. Speed PID ----
inline constexpr float kWheelSpeedPidKp             = 0.06f;
inline constexpr float kWheelSpeedPidKi             = 0.6f;
inline constexpr float kWheelSpeedPidKd             = 0.0f;
inline constexpr float kWheelSpeedPidBackCalcGain   = 0.3f;
inline constexpr float kWheelSpeedPidDiffCutoffHz   = 50.0f;
inline constexpr float kWheelSpeedPidOutputLimitNm  = kUseWheelSpeedPidFallback ? 0.5f : 0.0f;
inline constexpr float kWheelSpeedPidIntegLimitNm   = kUseWheelSpeedPidFallback ? 0.5f : 0.0f;
inline constexpr float kWheelSpeedPidKpRampTimeSec  = 1.5f;    // Kp ramp-up time

// ---- 6b. Velocity PLL (motor speed observer) ----
inline constexpr float kWheelSpeedPllBandwidth      = 75.0f;   // rad/s
inline constexpr float kWheelSpeedPllZeroSnapEpsRpm = 0.1f;    // snap-to-zero threshold
inline constexpr float kWheelSpeedPllOmegaRampTimeSec = 0.2f;

// ---- 6c. Wheel Mechanical Model ----
inline constexpr float kWheelInertiaKgM2              = 5e-5f;
inline constexpr float kWheelViscousDampingNmPerRadPS = 6.5e-5f;

// ---- 6d. 3-State Luenberger Observer (motor velocity) ----
// L1 = 3·ω_o·dt, must be < 1.0 for stable Euler discretization.
// dt=0.002s (500Hz): ω_o=100 → L1=0.6 (safe).
inline constexpr float kWheelObsVelocityBandwidth           = 100.0f;   // rad/s
inline constexpr float kWheelObsVelocityButterworthCutoffHz = 100.0f;   // LPF on observer output

// ---- 6e. Virtual Damping ----
// High near zero speed (airborne oscillation suppression),
// low at high speed (natural friction provides damping).
inline constexpr float kWheelVirtualDampingNmPerRadPS = 0.027f;
inline constexpr float kWheelVirtualDampingMin        = 0.005f;
inline constexpr float kWheelDampScheduleSpeedRadPS   = 10.0f;    // scheduling knee
inline constexpr float kWheelVirtualDampingLimitNm    = 0.4f;

// ---- 6f. MIT Control Mode (CAN) ----
// Run state: velocity damping + torque FF (kp=0 disables position loop)
inline constexpr float kMITRunKp      = 0.0f;
inline constexpr float kMITRunKd      = 0.0f;
inline constexpr float kMITRunTorqueFf = 0.0f;
inline constexpr float kMITKdRampTimeSec = 1.0f;    // Kd ramp-up when engaging

// Safe state: all gains zeroed (motor freewheels)
inline constexpr float kMITSafeKp      = 0.0f;
inline constexpr float kMITSafeKd      = 0.0f;
inline constexpr float kMITSafeTorqueFf = 0.0f;

// ---- 6g. Motor Protection ----
inline constexpr float kMotorRecoverDelayMs           = 1000.0f;
inline constexpr float kMotorClearErrorToEnableDelayMs = 10.0f;

// ---- 6h. Stationary Hold ----
// When chassis velocity command (sqrt(vx²+vy²)) and angular velocity command
// are both near zero, the wheel-speed PID output limit ramps from 0 to
// kStationaryHoldPidOutputLimitNm, adding holding torque on top of chassis
// controller torque_ff.
inline constexpr float kStationaryHoldSpeedThreshold = 0.05f;   // m/s
inline constexpr float kStationaryHoldOmegaThreshold = 0.1f;    // rad/s (~30 deg/s)
inline constexpr float kStationaryHoldPidOutputLimitNm = 0.3f;  // Nm per wheel


// ==========================================================================
//  7. Velocity Estimation & Fusion
// ==========================================================================

// ---- 7a. Chassis Velocity Source ----
// 0: wheel-based (Jacobian pseudo-inverse from motor velocities)
// 1: optical-flow-based (kf_vx / kf_vy from OptFlow)
// 2: fused — wheel + optflow 1-state Kalman (adaptive R, tilt-aware)
// 3: optflow + raw vision velocity fused by 1-state Kalman
// 4: pure vision velocity with Butterworth low-pass filter (no Kalman)
inline constexpr uint8_t kChassisVelocitySource = 2;

// ---- 7b. Fusion Kalman (source == 2) ----
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

// Speed transition point between low-speed / high-speed noise models
inline constexpr float kFusionSpeedTransitionX = 6.0f;   // m/s
inline constexpr float kFusionSpeedTransitionY = 6.0f;

// Tilt penalty: increase wheel R when IMU detects angular velocity
inline constexpr float kFusionTiltPenaltyGainX = 10.0f;  // per deg/s
inline constexpr float kFusionTiltPenaltyGainY = 10.0f;

// Wheel slip suppression: increase wheel R when planner demands
// large acceleration or wheel/optflow residuals diverge.
inline constexpr float kFusionWheelAccelThresholdX          = 1.5f;  // m/s²
inline constexpr float kFusionWheelAccelThresholdY          = 1.5f;
inline constexpr float kFusionWheelAccelPenaltyGainX        = 0.8f;
inline constexpr float kFusionWheelAccelPenaltyGainY        = 0.8f;
inline constexpr float kFusionWheelSlipAccelThresholdX      = 3.0f;  // m/s²
inline constexpr float kFusionWheelSlipAccelThresholdY      = 3.0f;
inline constexpr float kFusionWheelOptflowResidualThresholdX = 1.0f;  // m/s
inline constexpr float kFusionWheelOptflowResidualThresholdY = 1.0f;
inline constexpr float kFusionWheelSlipResidualPenaltyGainX  = 0.0f;
inline constexpr float kFusionWheelSlipResidualPenaltyGainY  = 0.0f;

// Wheel-based chassis velocity scaling & limits
inline constexpr float kWheelChassisVxScale        = 0.8f;
inline constexpr float kWheelChassisVyScale        = 0.5f;
inline constexpr float kWheelChassisVelocityLimitMS = 5.0f;   // m/s
inline constexpr float kWheelChassisAccelLimitMS2   = 10.0f;  // m/s²

// ---- 7c. Optflow + Vision Velocity Fusion (source == 3) ----
inline constexpr float kVisionOptflowFusionQX       = 0.02f;
inline constexpr float kVisionOptflowFusionQY       = 0.02f;
inline constexpr float kVisionOptflowFusionROptflowX = 0.05f;
inline constexpr float kVisionOptflowFusionROptflowY = 0.05f;
inline constexpr float kVisionOptflowFusionRVisionX  = 0.005f;
inline constexpr float kVisionOptflowFusionRVisionY  = 0.005f;

// ---- 7d. Pure Vision Velocity (source == 4) ----
	// Vision velocity only, no Kalman fusion; smoothed by Butterworth LPF.
inline constexpr float kVisionVelocityButterworthCutoffHz = 20.0f;


// ==========================================================================
//  8. Optical Flow
// ==========================================================================

// ---- PLL for flow sensor ----
inline constexpr float    kOptFlowPllBandwidthHz     = 300.0f;
inline constexpr float    kOptFlowPllZeroSnapMmPerS  = 1.0f;
inline constexpr float    kOptFlowPllMaxJumpMm       = 100.0f;
inline constexpr uint8_t  kOptFlowPllJumpConfirmFrames = 3;

// ---- Simplified Kalman for flow velocity ----
inline constexpr float    kOptFlowKfQVel               = 20.0f;
inline constexpr float    kOptFlowKfRVelFixed          = 300.0f;
inline constexpr float    kOptFlowVelocityLimitMS      = 5.0f;    // m/s
inline constexpr float    kOptFlowAccelLimitMS2        = 10.0f;   // m/s²
inline constexpr uint32_t kOptFlowSensorStaleTimeoutMs = 100;


// ==========================================================================
//  9. IMU  —  Stationary Detection  &  Bias Estimation
// ==========================================================================

// ---- 9a. Six-Axis Stationary Detection ----
inline constexpr float    kStationaryAccelVarThreshold   = 0.15f;
inline constexpr float    kStationaryGyroThresholdDegPerS = 1.5f;
inline constexpr float    kStationaryFlowThresholdMmPerS  = 10.0f;
inline constexpr uint8_t  kStationaryConfirmFrames = 10;
inline constexpr uint8_t  kStationaryWindowFrames  = 50;

// ---- 9b. Gyro Bias Estimation ----
// Bias alpha: blend rate for EMA of stationary gyro readings
inline constexpr float    kImuBiasAlpha                  = 0.02f;
inline constexpr float    kImuBiasAccelNormTolerance     = 0.45f;   // m/s²
inline constexpr float    kImuBiasAccelVarThreshold      = 0.08f;   // m/s² per sample
inline constexpr float    kImuBiasAccelStdMax            = 0.10f;   // m/s² over window
inline constexpr float    kImuBiasGyroStillThresholdDegPerS = 1.5f;  // deg/s
inline constexpr float    kImuBiasGyroStdMaxDegPerS      = 0.20f;   // deg/s over window
inline constexpr float    kImuBiasMaxAbsDegPerS          = 3.0f;    // candidate reject
inline constexpr float    kImuBiasMaxUpdateStepDegPerS   = 0.05f;   // per accepted window
inline constexpr uint16_t kImuBiasConfirmFrames          = 200;     // 0.25s at 800Hz
inline constexpr uint16_t kImuBiasWindowFrames           = 400;     // 0.50s at 800Hz
inline constexpr uint16_t kImuBiasStartupIgnoreFrames    = 2400;    // 3.0s at 800Hz
inline constexpr uint8_t  kImuBiasValidWindows           = 2;       // stable windows required

// ---- 9c. Roll / Pitch Complementary Filter ----
// Alpha for accel-gyro complementary filter (closer to 1 = trust gyro more)
inline constexpr float kImuRollPitchAlpha = 0.98f;


// ==========================================================================
// 10. Dribbler & Kick
// ==========================================================================

// Dribbler control mode
// true  = torque control (CAN ID 0xAD)
// false = velocity control (CAN ID 0xAC)
inline constexpr bool kDribblerTorqueMode = true;

// Test kick: bypass infrared sensor, rising-edge trigger on kick_discharge_time
inline constexpr bool kTestKick = false;


// ==========================================================================
//  A. Deprecated Parameters
// --------------------------------------------------------------------------
//  Kept for reference / potential fallback.  Not on the active control path.
// ==========================================================================

// Was used by YawSCurve incremental reference tracker (replaced by §3).
inline constexpr float kYawTargetReplanDeadbandRad = 1e-2f;
inline constexpr float kYawTargetStopBandRad       = 1e-3f;
inline constexpr float kYawTargetVelZeroEpsRadS    = 0.02f;

// Was used by YawSCurve linear fallback timer.
inline constexpr float kYawAngleLinearFallbackMinTimeSec = 0.05f;

// Was used by ChassisController IMU-branch PD position control
// (replaced by §3 inner-rate P control + §4 LESO).
inline constexpr float kAngleControllerBandwidth    = 50.0f;
inline constexpr float kAngleControllerBandwidthMin = 50.0f;

} // namespace control_config

#endif // __CONTROL_PARAMS_HPP
