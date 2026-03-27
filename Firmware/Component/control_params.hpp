#ifndef __CONTROL_PARAMS_HPP
#define __CONTROL_PARAMS_HPP

#include <cstdint>

namespace control_config {

inline constexpr float kControlDtSec = 1.0f / 1000.0f;
inline constexpr float kPi = 3.1415926535f;

inline constexpr float kAccThresholdX = 8.0f;
inline constexpr float kAccThresholdY = 8.0f;
inline constexpr float kAccThresholdYaw = 40.0f;

inline constexpr float kVelErrGainX = 12.0f;
inline constexpr float kVelErrGainY = 12.0f;
inline constexpr float kVelErrGainYaw = 20.0f;

inline constexpr float kJerkLimitX = 200.0f;
inline constexpr float kJerkLimitY = 200.0f;
inline constexpr float kJerkLimitYaw = 800.0f;

inline constexpr float kRobotMassKg = 2.19692f;
inline constexpr float kRobotInertiaKgM2 = 2.29587357e-6f;
inline constexpr float kWheelRadiusM = 0.0285f;
inline constexpr float kWheelCenterDistanceM = 0.073f;
inline constexpr float kCenterToComDistanceM = 0.0f;
inline constexpr float kWheelAlphaRad = 30.0f / 180.0f * kPi;
inline constexpr float kWheelBetaRad = 45.0f / 180.0f * kPi;

inline constexpr float kLesoVelObserverBandwidth = 65.0f;
inline constexpr float kLesoYawObserverBandwidth = 80.0f;

inline constexpr float kVelFeedbackGainX = 12.0f;
inline constexpr float kVelFeedbackGainY = 12.0f;
inline constexpr float kVelFeedbackGainYaw = 20.0f;

inline constexpr float kWheelTorqueFfLimitNm = 0.9f;

inline constexpr float kMITRunKp = 0.0f;
inline constexpr float kMITRunKd = 0.5f;
inline constexpr float kMITRunTorqueFf = 0.0f;

inline constexpr float kMITSafeKp = 2.0f;
inline constexpr float kMITSafeKd = 0.1f;
inline constexpr float kMITSafeTorqueFf = 0.0f;

inline constexpr float kVelEstAccDeadbandMps2 = 0.10f;
inline constexpr float kVelEstWheelTrustNormal = 0.92f;
inline constexpr float kVelEstWheelTrustSlip = 0.35f;
inline constexpr float kVelEstSlipLowMps = 0.15f;
inline constexpr float kVelEstSlipHighMps = 0.40f;
inline constexpr float kVelEstZeroWheelSpeedMps = 0.03f;
inline constexpr float kVelEstZeroOmegaRadPerSec = 0.15f;
inline constexpr float kVelEstZeroAccMps2 = 0.20f;
inline constexpr float kVelEstZeroDamping = 0.85f;
inline constexpr float kVelEstBiasLearnRate = 0.01f;
inline constexpr uint16_t kVelEstStationaryCycles = 20;

} // namespace control_config

#endif // __CONTROL_PARAMS_HPP
