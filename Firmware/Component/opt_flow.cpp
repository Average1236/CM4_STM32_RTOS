// ============================================================
// 双光流 + PLL + 简化静止检测 + 简化卡尔曼融合
// ============================================================

#include "opt_flow.hpp"
#include "control_params.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include "stm32f4xx_hal.h"

volatile float acc_var_debug = 0.0f;
volatile uint32_t stationary_confirm_count_debug = 0;

// ============================================================
// PositionPLL 实现
// ============================================================

PositionPLL::PositionPLL(float bandwidth_hz, float zero_snap_threshold,
                         float max_jump_mm, uint8_t jump_confirm_frames)
    : pos_est_mm_(0.0f), vel_est_mm_s_(0.0f),
      kp_(0.0f), ki_(0.0f),
      zero_snap_threshold_(zero_snap_threshold),
      max_jump_mm_(max_jump_mm),
      jump_confirm_frames_(jump_confirm_frames),
      jump_count_(0),
      initialized_(false)
{
    kp_ = 2.0f * bandwidth_hz;
    ki_ = 0.25f * kp_ * kp_;
}

void PositionPLL::reset(float measured_pos_mm) {
    pos_est_mm_   = measured_pos_mm;
    vel_est_mm_s_ = 0.0f;
    jump_count_   = 0;
    initialized_  = true;
}

void PositionPLL::update(float measured_pos_mm, float dt_s) {
    if (dt_s <= 0.0f) return;

    if (!initialized_) {
        reset(measured_pos_mm);
        return;
    }

    const float pos_pred = pos_est_mm_ + dt_s * vel_est_mm_s_;
    const float pos_err  = measured_pos_mm - pos_pred;

    if (fabsf(pos_err) > max_jump_mm_) {
        jump_count_++;
        if (jump_count_ >= jump_confirm_frames_) {
            reset(measured_pos_mm);
            jump_count_ = 0;
        } else {
            pos_est_mm_ = pos_pred;
        }
        return;
    }
    jump_count_ = 0;

    pos_est_mm_   = pos_pred + dt_s * kp_ * pos_err;
    vel_est_mm_s_ += dt_s * ki_ * pos_err;

    if (fabsf(vel_est_mm_s_) < zero_snap_threshold_) {
        vel_est_mm_s_ = 0.0f;
    }
}

// ============================================================
// FlowStationaryDetector 实现
//   — 光流+IMU联合静止检测 + accel偏置
//   — 陀螺偏置已迁移到 IMU
// ============================================================

namespace {
constexpr float kGravityMS2 = 9.81f;
constexpr float kDegToRad   = 3.1415926535f / 180.0f;

uint32_t elapsed_ms(uint32_t now, uint32_t then) {
    return (now >= then) ? (now - then) : (now + (0xFFFFFFFFu - then) + 1u);
}

bool tick_is_fresh(uint32_t now, uint32_t tick, uint32_t timeout_ms) {
    return tick != 0u && elapsed_ms(now, tick) <= timeout_ms;
}

float limit_velocity_axis(float target, float last, float dt_s,
                          float max_velocity, float max_accel) {
    target = std::clamp(target, -max_velocity, max_velocity);
    const float max_delta = max_accel * dt_s;
    return last + std::clamp(target - last, -max_delta, max_delta);
}
}

FlowStationaryDetector::FlowStationaryDetector()
    : phase_(Phase::kNormal), confirm_count_(0), collect_count_(0),
      stationary_confirmed_(false),
      sum_ax_(0.0f), sum_ay_(0.0f),
      prev_acc_norm_(kGravityMS2),
      bias_ax_(0.0f), bias_ay_(0.0f)
{}

void FlowStationaryDetector::update(float ax, float ay, float az,
                                     float gx, float gy, float gz,
                                     float flow_speed_norm)
{
    const float acc_norm = sqrtf(ax*ax + ay*ay + az*az);
    const float acc_var  = fabsf(acc_norm - prev_acc_norm_);
    acc_var_debug = acc_var;
    prev_acc_norm_ = acc_norm;

    const bool accel_stable = (acc_var < control_config::kStationaryAccelVarThreshold);
    const bool gyro_still   = (fabsf(gx) < control_config::kStationaryGyroThresholdDegPerS)
                           && (fabsf(gy) < control_config::kStationaryGyroThresholdDegPerS)
                           && (fabsf(gz) < control_config::kStationaryGyroThresholdDegPerS);
    const bool flow_still   = (flow_speed_norm < control_config::kStationaryFlowThresholdMmPerS);
    const bool all_still    = accel_stable && gyro_still && flow_still;

    switch (phase_) {
    case Phase::kNormal:
        if (all_still) {
            confirm_count_++;
            if (confirm_count_ >= control_config::kStationaryConfirmFrames) {
                phase_ = Phase::kCollecting;
                collect_count_ = 0;
                sum_ax_ = 0.0f; sum_ay_ = 0.0f;
            }
        } else {
            confirm_count_ = 0;
        }
        stationary_confirmed_ = false;
        break;

    case Phase::kCollecting:
        if (!all_still) {
            phase_ = Phase::kNormal;
            confirm_count_ = 0;
            stationary_confirmed_ = false;
            break;
        }
        sum_ax_ += ax; sum_ay_ += ay;
        collect_count_++;
        if (collect_count_ >= control_config::kStationaryWindowFrames) {
            phase_ = Phase::kVerifying;
        }
        stationary_confirmed_ = true;
        break;

    case Phase::kVerifying:
        if (all_still) {
            const float n = static_cast<float>(collect_count_);
            const float mean_ax = sum_ax_ / n;
            const float mean_ay = sum_ay_ / n;

            const float alpha = control_config::kImuBiasAlpha;
            bias_ax_ = alpha * mean_ax + (1.0f - alpha) * bias_ax_;
            bias_ay_ = alpha * mean_ay + (1.0f - alpha) * bias_ay_;

            phase_ = Phase::kCollecting;
            collect_count_ = 0;
            sum_ax_ = 0.0f; sum_ay_ = 0.0f;
            stationary_confirmed_ = true;
        } else {
            phase_ = Phase::kNormal;
            confirm_count_ = 0;
            stationary_confirmed_ = false;
        }
        break;
    }

    if (!all_still && phase_ == Phase::kCollecting) {
        phase_ = Phase::kNormal;
        confirm_count_ = 0;
        stationary_confirmed_ = false;
    }
    stationary_confirm_count_debug = confirm_count_;
}

// ============================================================
// Kalman2DPosVel 实现
// ============================================================

Kalman2DPosVel::Kalman2DPosVel() {
    setNoise(0.1f, 5.0f, 0.01f, 0.01f, 300.0f, 1e6f);
    init(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 500.0f);
}

void Kalman2DPosVel::setNoise(float q_pos, float q_vel, float q_bias_ax, float q_bias_ay, float r_vel, float r_pos) {
    Q_[0] = q_pos; Q_[1] = q_pos;
    Q_[2] = q_vel; Q_[3] = q_vel;
    Q_[4] = q_bias_ax; Q_[5] = q_bias_ay;
    Rv_[0] = r_vel; Rv_[1] = r_vel;
    Rp_[0] = r_pos; Rp_[1] = r_pos;
}

void Kalman2DPosVel::init(float px0, float py0, float vx0, float vy0, float bax0, float bay0, float p0) {
    x_[0] = px0; x_[1] = py0;
    x_[2] = vx0; x_[3] = vy0;
    x_[4] = bax0; x_[5] = bay0;
    for (int i = 0; i < 36; ++i) P_[i] = 0.0f;
    P_[0] = p0; P_[7] = p0; P_[14] = p0; P_[21] = p0;
    P_[28] = p0; P_[35] = p0;
}

void Kalman2DPosVel::predict(float ax, float ay, float dt) {
    if (dt <= 0.0f) return;
    const float hdt2 = 0.5f * dt * dt;

    const float ax_eff = ax - x_[4];
    const float ay_eff = ay - x_[5];

    x_[0] += dt * x_[2] + hdt2 * ax_eff;
    x_[1] += dt * x_[3] + hdt2 * ay_eff;
    x_[2] += dt * ax_eff;
    x_[3] += dt * ay_eff;

    float AP[36];
    for (int j = 0; j < 6; ++j) {
        AP[0*6+j] = P_[0*6+j] + dt * P_[2*6+j] - hdt2 * P_[4*6+j];
        AP[1*6+j] = P_[1*6+j] + dt * P_[3*6+j] - hdt2 * P_[5*6+j];
        AP[2*6+j] = P_[2*6+j] - dt * P_[4*6+j];
        AP[3*6+j] = P_[3*6+j] - dt * P_[5*6+j];
        AP[4*6+j] = P_[4*6+j];
        AP[5*6+j] = P_[5*6+j];
    }

    float Pn[36];
    for (int i = 0; i < 6; ++i) {
        Pn[i*6+0] = AP[i*6+0];
        Pn[i*6+1] = AP[i*6+1];
        Pn[i*6+2] = AP[i*6+0] * dt + AP[i*6+2];
        Pn[i*6+3] = AP[i*6+1] * dt + AP[i*6+3];
        Pn[i*6+4] = -AP[i*6+0] * hdt2 - AP[i*6+2] * dt + AP[i*6+4];
        Pn[i*6+5] = -AP[i*6+1] * hdt2 - AP[i*6+3] * dt + AP[i*6+5];
    }

    Pn[0]  += Q_[0];   Pn[7]  += Q_[1];
    Pn[14] += Q_[2];   Pn[21] += Q_[3];
    Pn[28] += Q_[4];   Pn[35] += Q_[5];

    for (int i = 0; i < 36; ++i) P_[i] = Pn[i];
}

void Kalman2DPosVel::updateVel(float vx_meas, float vy_meas) {
    const float y0 = vx_meas - x_[2];
    const float y1 = vy_meas - x_[3];

    const float S00 = P_[2*6+2] + Rv_[0];
    const float S01 = P_[2*6+3];
    const float S10 = P_[3*6+2];
    const float S11 = P_[3*6+3] + Rv_[1];

    const float det = S00 * S11 - S01 * S10;
    if (det == 0.0f) return;
    const float invS00 =  S11 / det, invS01 = -S01 / det;
    const float invS10 = -S10 / det, invS11 =  S00 / det;

    float PHt[12];
    for (int i = 0; i < 6; ++i) {
        PHt[i*2+0] = P_[i*6+2];
        PHt[i*2+1] = P_[i*6+3];
    }

    float K[12];
    for (int i = 0; i < 6; ++i) {
        K[i*2+0] = PHt[i*2+0] * invS00 + PHt[i*2+1] * invS10;
        K[i*2+1] = PHt[i*2+0] * invS01 + PHt[i*2+1] * invS11;
    }

    for (int i = 0; i < 6; ++i) {
        x_[i] += K[i*2+0] * y0 + K[i*2+1] * y1;
    }

    float Pn[36];
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            Pn[i*6+j] = P_[i*6+j] - (K[i*2+0] * P_[2*6+j] + K[i*2+1] * P_[3*6+j]);
        }
    }
    for (int i = 0; i < 36; ++i) P_[i] = Pn[i];
}

void Kalman2DPosVel::updatePos(float px_meas, float py_meas) {
    const float y0 = px_meas - x_[0];
    const float y1 = py_meas - x_[1];

    const float S00 = P_[0*6+0] + Rp_[0];
    const float S01 = P_[0*6+1];
    const float S10 = P_[1*6+0];
    const float S11 = P_[1*6+1] + Rp_[1];

    const float det = S00 * S11 - S01 * S10;
    if (det == 0.0f) return;
    const float invS00 =  S11 / det, invS01 = -S01 / det;
    const float invS10 = -S10 / det, invS11 =  S00 / det;

    float PHt[12];
    for (int i = 0; i < 6; ++i) {
        PHt[i*2+0] = P_[i*6+0];
        PHt[i*2+1] = P_[i*6+1];
    }

    float K[12];
    for (int i = 0; i < 6; ++i) {
        K[i*2+0] = PHt[i*2+0] * invS00 + PHt[i*2+1] * invS10;
        K[i*2+1] = PHt[i*2+0] * invS01 + PHt[i*2+1] * invS11;
    }

    for (int i = 0; i < 6; ++i) {
        x_[i] += K[i*2+0] * y0 + K[i*2+1] * y1;
    }

    float Pn[36];
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            Pn[i*6+j] = P_[i*6+j] - (K[i*2+0] * P_[0*6+j] + K[i*2+1] * P_[1*6+j]);
        }
    }
    for (int i = 0; i < 36; ++i) P_[i] = Pn[i];
}

// ============================================================
// OptFlow 构造 / reset
// ============================================================
OptFlow::OptFlow()
    : pll_lx_(control_config::kOptFlowPllBandwidthHz,
              control_config::kOptFlowPllZeroSnapMmPerS,
              control_config::kOptFlowPllMaxJumpMm,
              control_config::kOptFlowPllJumpConfirmFrames),
      pll_ly_(control_config::kOptFlowPllBandwidthHz,
              control_config::kOptFlowPllZeroSnapMmPerS,
              control_config::kOptFlowPllMaxJumpMm,
              control_config::kOptFlowPllJumpConfirmFrames),
      pll_rx_(control_config::kOptFlowPllBandwidthHz,
              control_config::kOptFlowPllZeroSnapMmPerS,
              control_config::kOptFlowPllMaxJumpMm,
              control_config::kOptFlowPllJumpConfirmFrames),
      pll_ry_(control_config::kOptFlowPllBandwidthHz,
              control_config::kOptFlowPllZeroSnapMmPerS,
              control_config::kOptFlowPllMaxJumpMm,
              control_config::kOptFlowPllJumpConfirmFrames),
      left_last_x_(0.0f), left_last_y_(0.0f),
      right_last_x_(0.0f), right_last_y_(0.0f),
      last_time_ms_(0),
      last_process_time_ms_(0),
      left_initialized_(false), right_initialized_(false),
      kf_inited_(false),
      prev_stationary_(false),
      body_limiter_initialized_(false),
      limited_body_vx_(0.0f),
      limited_body_vy_(0.0f)
{
    memset(&state_, 0, sizeof(state_));
    kf_.setNoise(kQPos, control_config::kOptFlowKfQVel,
                 kQBiasAx, kQBiasAy,
                 control_config::kOptFlowKfRVelFixed, kRPos);
}

void OptFlow::reset() {
    memset(&state_, 0, sizeof(state_));
    left_initialized_  = false;
    right_initialized_ = false;
    left_last_x_  = 0.0f;
    left_last_y_  = 0.0f;
    right_last_x_ = 0.0f;
    right_last_y_ = 0.0f;
    last_time_ms_ = 0;
    last_process_time_ms_ = 0;
    kf_inited_   = false;
    prev_stationary_ = false;
    body_limiter_initialized_ = false;
    limited_body_vx_ = 0.0f;
    limited_body_vy_ = 0.0f;

    pll_lx_.reset(0.0f); pll_ly_.reset(0.0f);
    pll_rx_.reset(0.0f); pll_ry_.reset(0.0f);

    kf_.init(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1000.0f);
    kf_.setNoise(kQPos, control_config::kOptFlowKfQVel,
                 kQBiasAx, kQBiasAy,
                 control_config::kOptFlowKfRVelFixed, kRPos);
}

// ============================================================
// process(Data_t)
//   IMU数据通过 InputPort 读取，不再通过 Data_t
// ============================================================
void OptFlow::process(const Data_t& data) {
    state_.time_ms = data.tick_ms;

    const bool have_left  = ((data.valid_mask & OPTFLOW_MASK_LEFT)  != 0u)
        && tick_is_fresh(data.tick_ms, data.left_tick_ms,
                         control_config::kOptFlowSensorStaleTimeoutMs);
    const bool have_right = ((data.valid_mask & OPTFLOW_MASK_RIGHT) != 0u)
        && tick_is_fresh(data.tick_ms, data.right_tick_ms,
                         control_config::kOptFlowSensorStaleTimeoutMs);
    const bool flow_available = have_left || have_right;

    // --- 读取 IMU 数据（通过 Port） ---
    auto ax_opt = imu_acc_x_input_.any(), ay_opt = imu_acc_y_input_.any(), az_opt = imu_acc_z_input_.any();
    auto gx_opt = imu_gyro_x_input_.any(), gy_opt = imu_gyro_y_input_.any(), gz_opt = imu_gyro_z_input_.any();
    const float ax = ax_opt.has_value() ? *ax_opt : 0.0f;
    const float ay = ay_opt.has_value() ? *ay_opt : 0.0f;
    const float az = az_opt.has_value() ? *az_opt : 0.0f;
    const float gx = gx_opt.has_value() ? *gx_opt : 0.0f;
    const float gy = gy_opt.has_value() ? *gy_opt : 0.0f;
    const float gz = gz_opt.has_value() ? *gz_opt : 0.0f;
    const bool imu_valid = ax_opt.has_value() && gz_opt.has_value();

    // 左右独立初始化
    bool left_just_inited  = false;
    bool right_just_inited = false;

    if (have_left && !left_initialized_) {
        left_last_x_ = data.left_x;  left_last_y_ = data.left_y;
        pll_lx_.reset(data.left_x);  pll_ly_.reset(data.left_y);
        left_initialized_ = true;    left_just_inited = true;
    }
    if (have_right && !right_initialized_) {
        right_last_x_ = data.right_x; right_last_y_ = data.right_y;
        pll_rx_.reset(data.right_x);  pll_ry_.reset(data.right_y);
        right_initialized_ = true;    right_just_inited = true;
    }

    if (!left_initialized_ && !right_initialized_) {
        last_time_ms_ = data.tick_ms;
        last_process_time_ms_ = data.tick_ms;
        state_.last_time_ms = data.tick_ms;
        return;
    }
    if (left_just_inited || right_just_inited) {
        last_time_ms_ = data.tick_ms;
        last_process_time_ms_ = data.tick_ms;
        state_.last_time_ms = data.tick_ms;
        return;
    }

    const unsigned int process_t0 = (last_process_time_ms_ != 0u) ? last_process_time_ms_ : last_time_ms_;
    const unsigned int process_dt_ms = elapsed_ms(data.tick_ms, process_t0);
    float dt_s = static_cast<float>(process_dt_ms) / 1000.0f;

    if (dt_s < MIN_DT || dt_s > MAX_DT) {
        last_process_time_ms_ = data.tick_ms;
        state_.last_time_ms = data.tick_ms;
        return;
    }

    float flow_dt_s = dt_s;
    if (flow_available) {
        const unsigned int flow_dt_ms = elapsed_ms(data.tick_ms, last_time_ms_);
        flow_dt_s = static_cast<float>(flow_dt_ms) / 1000.0f;
        if (flow_dt_s < MIN_DT || flow_dt_s > MAX_DT) {
            last_time_ms_ = data.tick_ms;
            last_process_time_ms_ = data.tick_ms;
            state_.last_time_ms = data.tick_ms;
            return;
        }
    }
    state_.dt_s = dt_s;
    state_.last_time_ms = last_time_ms_;

    // ---- PLL 更新 ----
    if (have_left)  { pll_lx_.update(data.left_x, flow_dt_s);  pll_ly_.update(data.left_y, flow_dt_s); }
    if (have_right) { pll_rx_.update(data.right_x, flow_dt_s); pll_ry_.update(data.right_y, flow_dt_s); }

    state_.pll_l_vx = have_left  ? pll_lx_.vel() : 0.0f;
    state_.pll_l_vy = have_left  ? pll_ly_.vel() : 0.0f;
    state_.pll_r_vx = have_right ? pll_rx_.vel() : 0.0f;
    state_.pll_r_vy = have_right ? pll_ry_.vel() : 0.0f;

    // ---- 双路融合 ----
    float frame_dx_robot = 0.0f, frame_dy_robot = 0.0f, frame_dtheta = 0.0f;

    if (have_left && have_right) {
        float dx1 = data.left_x - left_last_x_, dy1 = data.left_y - left_last_y_;
        float dx2 = data.right_x - right_last_x_, dy2 = data.right_y - right_last_y_;
        float dx_robot = 0.5f * (dx1 + dx2), dy_robot = 0.5f * (dy1 + dy2);
        float sin_arg = (dx2 - dx1) / (2.0f * HALF_BASELINE);
        if (sin_arg > 1.0f) sin_arg = 1.0f; else if (sin_arg < -1.0f) sin_arg = -1.0f;
        float dtheta = asinf(sin_arg);
        state_.raw_dtheta = dtheta;
        state_.body_vx = 0.5f * (state_.pll_l_vx + state_.pll_r_vx);
        state_.body_vy = 0.5f * (state_.pll_l_vy + state_.pll_r_vy);
        state_.omega_z = dtheta / flow_dt_s;
        frame_dx_robot = dx_robot; frame_dy_robot = dy_robot; frame_dtheta = dtheta;
    } else if (have_left) {
        float dx1 = data.left_x - left_last_x_, dy1 = data.left_y - left_last_y_;
        state_.body_vx = state_.pll_l_vx; state_.body_vy = state_.pll_l_vy;
        state_.omega_z = 0.0f; state_.raw_dtheta = 0.0f;
        frame_dx_robot = dx1; frame_dy_robot = dy1;
    } else if (have_right) {
        float dx2 = data.right_x - right_last_x_, dy2 = data.right_y - right_last_y_;
        state_.body_vx = state_.pll_r_vx; state_.body_vy = state_.pll_r_vy;
        state_.omega_z = 0.0f; state_.raw_dtheta = 0.0f;
        frame_dx_robot = dx2; frame_dy_robot = dy2;
    } else {
        state_.body_vx = 0.0f; state_.body_vy = 0.0f;
        state_.omega_z = 0.0f; state_.raw_dtheta = 0.0f;
    }

    const float raw_body_vx = state_.body_vx;
    const float raw_body_vy = state_.body_vy;

    // ---- 静止检测 + accel偏置估计（光流+IMU联合） ----
    const float flow_speed_norm = sqrtf(raw_body_vx * raw_body_vx
                                      + raw_body_vy * raw_body_vy);
    if (imu_valid) {
        flow_stat_.update(ax, ay, az, gx, gy, gz, flow_speed_norm);
    }

    state_.is_stationary = flow_stat_.is_stationary();
    state_.imu_bias_ax = flow_stat_.bias_ax();
    state_.imu_bias_ay = flow_stat_.bias_ay();

    if (state_.is_stationary) {
        state_.body_vx = 0.0f;
        state_.body_vy = 0.0f;
        state_.omega_z = 0.0f;
        frame_dx_robot = 0.0f;
        frame_dy_robot = 0.0f;
        frame_dtheta = 0.0f;
        limited_body_vx_ = 0.0f;
        limited_body_vy_ = 0.0f;
        body_limiter_initialized_ = true;
    } else {
        const float max_vel_mm_s = control_config::kOptFlowVelocityLimitMS * 1000.0f;
        const float max_acc_mm_s2 = control_config::kOptFlowAccelLimitMS2 * 1000.0f;
        if (!body_limiter_initialized_) {
            limited_body_vx_ = std::clamp(raw_body_vx, -max_vel_mm_s, max_vel_mm_s);
            limited_body_vy_ = std::clamp(raw_body_vy, -max_vel_mm_s, max_vel_mm_s);
            body_limiter_initialized_ = true;
        } else {
            limited_body_vx_ = limit_velocity_axis(raw_body_vx, limited_body_vx_, dt_s,
                                                   max_vel_mm_s, max_acc_mm_s2);
            limited_body_vy_ = limit_velocity_axis(raw_body_vy, limited_body_vy_, dt_s,
                                                   max_vel_mm_s, max_acc_mm_s2);
        }
        state_.body_vx = limited_body_vx_;
        state_.body_vy = limited_body_vy_;
    }

    // ---- 纯光流积分位姿 ----
    const float actual_dtheta = state_.is_stationary ? 0.0f
        : (imu_valid ? (gz * kDegToRad * dt_s) : frame_dtheta);
    const float theta_mid = state_.flow_yaw + 0.5f * actual_dtheta;
    const float c = cosf(theta_mid), s = sinf(theta_mid);
    state_.flow_px += frame_dx_robot * c + frame_dy_robot * s;
    state_.flow_py += -frame_dx_robot * s + frame_dy_robot * c;
    state_.flow_yaw += actual_dtheta;

    // ---- 简化卡尔曼 ----
    const float ax_mm = (imu_valid && !state_.is_stationary) ? (ax - flow_stat_.bias_ax()) * 1000.0f : 0.0f;
    const float ay_mm = (imu_valid && !state_.is_stationary) ? (ay - flow_stat_.bias_ay()) * 1000.0f : 0.0f;

    if (!kf_inited_ && flow_available) {
        kf_.init(0.0f, 0.0f, state_.body_vx, state_.body_vy, 0.0f, 0.0f, 1000.0f);
        kf_inited_ = true;
    }

    const bool entering_stationary = state_.is_stationary && !prev_stationary_;
    const bool exiting_stationary  = !state_.is_stationary && prev_stationary_;

    if (entering_stationary) {
        kf_.setNoise(kQPos, control_config::kOptFlowKfQVel,
                     kQBiasAx, kQBiasAy,
                     control_config::kOptFlowKfRVelFixed * 0.1f, kRPos);
    }
    if (exiting_stationary) {
        kf_.setNoise(kQPos, control_config::kOptFlowKfQVel,
                     kQBiasAx, kQBiasAy,
                     control_config::kOptFlowKfRVelFixed, kRPos);
    }
    prev_stationary_ = state_.is_stationary;

    if (kf_inited_ && imu_valid) {
        kf_.predict(ax_mm, ay_mm, dt_s);
    }

    if (kf_inited_) {
        if (state_.is_stationary) {
            kf_.updateVel(0.0f, 0.0f);
        } else {
            kf_.updateVel(state_.body_vx, state_.body_vy);
        }
    }

    if (kf_inited_) {
        state_.kf_vx = kf_.vx(); state_.kf_vy = kf_.vy();
        state_.kf_px = kf_.px(); state_.kf_py = kf_.py();
    } else {
        state_.kf_vx = state_.body_vx; state_.kf_vy = state_.body_vy;
        state_.kf_px = 0.0f; state_.kf_py = 0.0f;
    }

    // ---- 发布 OutputPort ----
    kf_vx_port_   = state_.kf_vx;
    kf_vy_port_   = state_.kf_vy;
    body_vx_port_ = state_.body_vx;
    body_vy_port_ = state_.body_vy;
    omega_z_port_ = state_.omega_z;

    // 更新 last 值
    if (have_left)  { left_last_x_ = data.left_x;   left_last_y_ = data.left_y; }
    if (have_right) { right_last_x_ = data.right_x; right_last_y_ = data.right_y; }
    if (flow_available) {
        last_time_ms_ = data.tick_ms;
    }
    last_process_time_ms_ = data.tick_ms;
}
