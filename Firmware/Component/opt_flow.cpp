// ============================================================
// 双光流 + PLL + 六轴IMU偏置 + 简化卡尔曼融合
// ============================================================

#include "opt_flow.hpp"
#include "control_params.hpp"
#include "freertos_vars.h"
#include <cmath>
#include <cstring>
#include "stm32f4xx_hal.h"

volatile float acc_var_debug = 0.0f; // For debugging: expose acceleration variance for tuning stationary detection
volatile uint32_t stationary_confirm_count_debug = 0; // For debugging: expose stationary confirm count for tuning stationary detection

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

    // 跳变保护：N帧确认机制（修订#4）
    if (fabsf(pos_err) > max_jump_mm_) {
        jump_count_++;
        if (jump_count_ >= jump_confirm_frames_) {
            // 连续 N 帧跳变 → 确认异常 → 重置
            reset(measured_pos_mm);
            jump_count_ = 0;
        } else {
            // 单帧飞点：丢弃本帧更新，仅做位置预测
            pos_est_mm_ = pos_pred;
        }
        return;
    }
    jump_count_ = 0;  // 正常帧，清零计数器

    pos_est_mm_   = pos_pred + dt_s * kp_ * pos_err;
    vel_est_mm_s_ += dt_s * ki_ * pos_err;

    // 零速归零
    if (fabsf(vel_est_mm_s_) < zero_snap_threshold_) {
        vel_est_mm_s_ = 0.0f;
    }
}

// ============================================================
// SixAxisImuBias 实现
// ============================================================

namespace {
constexpr float kGravityMS2 = 9.81f;
constexpr float kDegToRad   = 3.1415926535f / 180.0f;
}

SixAxisImuBias::SixAxisImuBias()
    : phase_(Phase::kNormal), confirm_count_(0), collect_count_(0),
      stationary_confirmed_(false),
      sum_ax_(0.0f), sum_ay_(0.0f), sum_gx_(0.0f), sum_gy_(0.0f), sum_gz_(0.0f),
      prev_acc_norm_(kGravityMS2),
      bias_ax_(0.0f), bias_ay_(0.0f),
      bias_gx_(0.0f), bias_gy_(0.0f), bias_gz_(0.0f)
{}

void SixAxisImuBias::update(float ax, float ay, float az,
                             float gx, float gy, float gz,
                             float flow_speed_norm, float dt_s)
{
    (void)dt_s;  // 帧率由 confirm/collect 帧数控制，不依赖绝对时间

    // ---- 静止条件评估 ----
    const float acc_norm = sqrtf(ax*ax + ay*ay + az*az);
    const float acc_var  = fabsf(acc_norm - prev_acc_norm_);
    acc_var_debug = acc_var; // For debugging: expose acceleration variance for tuning stationary detection
    prev_acc_norm_ = acc_norm;

    const bool accel_stable = (acc_var < control_config::kStationaryAccelVarThreshold);
    const bool gyro_still   = (fabsf(gx) < control_config::kStationaryGyroThresholdDegPerS)
                           && (fabsf(gy) < control_config::kStationaryGyroThresholdDegPerS)
                           && (fabsf(gz) < control_config::kStationaryGyroThresholdDegPerS);
    const bool flow_still   = (flow_speed_norm < control_config::kStationaryFlowThresholdMmPerS);
    const bool all_still    = accel_stable && gyro_still && flow_still;

    // ---- 状态机 ----
    switch (phase_) {
    case Phase::kNormal:
        if (all_still) {
            confirm_count_++;
            if (confirm_count_ >= control_config::kStationaryConfirmFrames) {
                // 阶段1→2：确认静止，开始采集
                phase_ = Phase::kCollecting;
                collect_count_ = 0;
                sum_ax_ = 0.0f; sum_ay_ = 0.0f;
                sum_gx_ = 0.0f; sum_gy_ = 0.0f; sum_gz_ = 0.0f;
            }
        } else {
            confirm_count_ = 0;
        }
        stationary_confirmed_ = false;
        break;

    case Phase::kCollecting:
        if (!all_still) {
            // 采集中断，回退
            phase_ = Phase::kNormal;
            confirm_count_ = 0;
            stationary_confirmed_ = false;
            break;
        }
        // 累积样本（6轴）
        sum_ax_ += ax; sum_ay_ += ay;
        sum_gx_ += gx; sum_gy_ += gy; sum_gz_ += gz;
        collect_count_++;
        if (collect_count_ >= control_config::kStationaryWindowFrames) {
            // 阶段2→3：二次验证
            phase_ = Phase::kVerifying;
        }
        // 采集期间仍视为静止（用于零速强制）
        stationary_confirmed_ = true;
        break;

    case Phase::kVerifying:
        if (all_still) {
            // 二次验证通过 → 更新偏置
            const float n = static_cast<float>(collect_count_);
            const float mean_ax = sum_ax_ / n;
            const float mean_ay = sum_ay_ / n;
            const float mean_gx = sum_gx_ / n;
            const float mean_gy = sum_gy_ / n;
            const float mean_gz = sum_gz_ / n;

            const float alpha = control_config::kImuBiasAlpha;
            bias_ax_ = alpha * mean_ax + (1.0f - alpha) * bias_ax_;
            bias_ay_ = alpha * mean_ay + (1.0f - alpha) * bias_ay_;
            bias_gx_ = alpha * mean_gx + (1.0f - alpha) * bias_gx_;
            bias_gy_ = alpha * mean_gy + (1.0f - alpha) * bias_gy_;
            bias_gz_ = alpha * mean_gz + (1.0f - alpha) * bias_gz_;

            // 更新全局 gyro bias 供 IMU Task 使用
            g_imu_bias_gx_dps = bias_gx_;
            g_imu_bias_gy_dps = bias_gy_;
            g_imu_trust_accel  = true;

            // 保持静止状态，继续采集下一轮
            phase_ = Phase::kCollecting;
            collect_count_ = 0;
            sum_ax_ = 0.0f; sum_ay_ = 0.0f;
            sum_gx_ = 0.0f; sum_gy_ = 0.0f; sum_gz_ = 0.0f;
            stationary_confirmed_ = true;
        } else {
            // 二次验证失败 → 丢弃样本
            phase_ = Phase::kNormal;
            confirm_count_ = 0;
            g_imu_trust_accel = false;
            stationary_confirmed_ = false;
        }
        break;
    }

    // 任何一帧不满足静止条件 → 退出
    if (!all_still && phase_ != Phase::kNormal) {
        // 仅在非 verifying 时立即退出；verifying 已在上面处理
        if (phase_ == Phase::kCollecting) {
            phase_ = Phase::kNormal;
            confirm_count_ = 0;
            g_imu_trust_accel = false;
            stationary_confirmed_ = false;
        }
    }
    stationary_confirm_count_debug = confirm_count_; // For debugging: expose stationary confirm count for tuning stationary detection
}

// ============================================================
// Kalman2DPosVel 实现（保持不变）
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

    // 扣除偏置后的有效加速度
    const float ax_eff = ax - x_[4];
    const float ay_eff = ay - x_[5];

    // 状态外推（含偏置补偿的运动学积分）
    x_[0] += dt * x_[2] + hdt2 * ax_eff;
    x_[1] += dt * x_[3] + hdt2 * ay_eff;
    x_[2] += dt * ax_eff;
    x_[3] += dt * ay_eff;
    // x_[4], x_[5] 偏置项保持不变（随机游走模型）

    // ---- P_pred = F · P · Fᵀ + Q ----
    // Step 1: AP = F · P  (6×6)
    float AP[36];
    for (int j = 0; j < 6; ++j) {
        AP[0*6+j] = P_[0*6+j] + dt * P_[2*6+j] - hdt2 * P_[4*6+j];
        AP[1*6+j] = P_[1*6+j] + dt * P_[3*6+j] - hdt2 * P_[5*6+j];
        AP[2*6+j] = P_[2*6+j] - dt * P_[4*6+j];
        AP[3*6+j] = P_[3*6+j] - dt * P_[5*6+j];
        AP[4*6+j] = P_[4*6+j];
        AP[5*6+j] = P_[5*6+j];
    }

    // Step 2: Pn = AP · Fᵀ  (6×6)
    float Pn[36];
    for (int i = 0; i < 6; ++i) {
        Pn[i*6+0] = AP[i*6+0];
        Pn[i*6+1] = AP[i*6+1];
        Pn[i*6+2] = AP[i*6+0] * dt + AP[i*6+2];
        Pn[i*6+3] = AP[i*6+1] * dt + AP[i*6+3];
        Pn[i*6+4] = -AP[i*6+0] * hdt2 - AP[i*6+2] * dt + AP[i*6+4];
        Pn[i*6+5] = -AP[i*6+1] * hdt2 - AP[i*6+3] * dt + AP[i*6+5];
    }

    // Step 3: 加过程噪声 Q（对角线）
    Pn[0]  += Q_[0];   // q_pos
    Pn[7]  += Q_[1];   // q_pos
    Pn[14] += Q_[2];   // q_vel
    Pn[21] += Q_[3];   // q_vel
    Pn[28] += Q_[4];   // q_bias_ax
    Pn[35] += Q_[5];   // q_bias_ay

    for (int i = 0; i < 36; ++i) P_[i] = Pn[i];
}

void Kalman2DPosVel::updateVel(float vx_meas, float vy_meas) {
    // 创新: y = z - H·x,  H = [0,0,1,0,0,0; 0,0,0,1,0,0]
    const float y0 = vx_meas - x_[2];
    const float y1 = vy_meas - x_[3];

    // S = H·P·Hᵀ + Rv  (2×2, 取 P 的 vx/vy 子块)
    const float S00 = P_[2*6+2] + Rv_[0];
    const float S01 = P_[2*6+3];
    const float S10 = P_[3*6+2];
    const float S11 = P_[3*6+3] + Rv_[1];

    const float det = S00 * S11 - S01 * S10;
    if (det == 0.0f) return;
    const float invS00 =  S11 / det;
    const float invS01 = -S01 / det;
    const float invS10 = -S10 / det;
    const float invS11 =  S00 / det;

    // PHt = P · Hᵀ = 取 P 的第 2、3 列 (6×2)
    float PHt[12];
    for (int i = 0; i < 6; ++i) {
        PHt[i*2+0] = P_[i*6+2];
        PHt[i*2+1] = P_[i*6+3];
    }

    // K = PHt · S⁻¹  (6×2)
    float K[12];
    for (int i = 0; i < 6; ++i) {
        K[i*2+0] = PHt[i*2+0] * invS00 + PHt[i*2+1] * invS10;
        K[i*2+1] = PHt[i*2+0] * invS01 + PHt[i*2+1] * invS11;
    }

    // 状态更新: x += K·y
    for (int i = 0; i < 6; ++i) {
        x_[i] += K[i*2+0] * y0 + K[i*2+1] * y1;
    }

    // P 更新: P_new = P - K·H·P  (6×6 全更新)
    float Pn[36];
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            Pn[i*6+j] = P_[i*6+j] - (K[i*2+0] * P_[2*6+j] + K[i*2+1] * P_[3*6+j]);
        }
    }
    for (int i = 0; i < 36; ++i) P_[i] = Pn[i];
}

void Kalman2DPosVel::updatePos(float px_meas, float py_meas) {
    // 创新: y = z - H·x,  H = [1,0,0,0,0,0; 0,1,0,0,0,0]
    const float y0 = px_meas - x_[0];
    const float y1 = py_meas - x_[1];

    // S = H·P·Hᵀ + Rp  (2×2, 取 P 的 px/py 子块)
    const float S00 = P_[0*6+0] + Rp_[0];
    const float S01 = P_[0*6+1];
    const float S10 = P_[1*6+0];
    const float S11 = P_[1*6+1] + Rp_[1];

    const float det = S00 * S11 - S01 * S10;
    if (det == 0.0f) return;
    const float invS00 =  S11 / det;
    const float invS01 = -S01 / det;
    const float invS10 = -S10 / det;
    const float invS11 =  S00 / det;

    // PHt = P · Hᵀ = 取 P 的第 0、1 列 (6×2)
    float PHt[12];
    for (int i = 0; i < 6; ++i) {
        PHt[i*2+0] = P_[i*6+0];
        PHt[i*2+1] = P_[i*6+1];
    }

    // K = PHt · S⁻¹  (6×2)
    float K[12];
    for (int i = 0; i < 6; ++i) {
        K[i*2+0] = PHt[i*2+0] * invS00 + PHt[i*2+1] * invS10;
        K[i*2+1] = PHt[i*2+0] * invS01 + PHt[i*2+1] * invS11;
    }

    // 状态更新: x += K·y
    for (int i = 0; i < 6; ++i) {
        x_[i] += K[i*2+0] * y0 + K[i*2+1] * y1;
    }

    // P 更新: P_new = P - K·H·P  (6×6 全更新)
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
      left_initialized_(false), right_initialized_(false),
      kf_inited_(false),
      prev_stationary_(false)
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
    kf_inited_   = false;
    prev_stationary_ = false;

    pll_lx_.reset(0.0f);
    pll_ly_.reset(0.0f);
    pll_rx_.reset(0.0f);
    pll_ry_.reset(0.0f);

    kf_.init(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1000.0f);
    kf_.setNoise(kQPos, control_config::kOptFlowKfQVel,
                 kQBiasAx, kQBiasAy,
                 control_config::kOptFlowKfRVelFixed, kRPos);
}

// ============================================================
// process(Data_t)
//   传感器安装坐标: 左 (0, +l), 右 (0, -l)
//   dx_robot = (dx1 + dx2) / 2
//   dy_robot = (dy1 + dy2) / 2
//   dtheta   = arcsin((dx2 - dx1) / (2*l))
// ============================================================
void OptFlow::process(const Data_t& data) {
    state_.time_ms = data.tick_ms;

    const bool have_left  = (data.valid_mask & OPTFLOW_MASK_LEFT)  != 0u;
    const bool have_right = (data.valid_mask & OPTFLOW_MASK_RIGHT) != 0u;

    // 左右独立初始化
    bool left_just_inited  = false;
    bool right_just_inited = false;

    if (have_left && !left_initialized_) {
        left_last_x_      = data.left_x;
        left_last_y_      = data.left_y;
        pll_lx_.reset(data.left_x);
        pll_ly_.reset(data.left_y);
        left_initialized_ = true;
        left_just_inited  = true;
    }
    if (have_right && !right_initialized_) {
        right_last_x_      = data.right_x;
        right_last_y_      = data.right_y;
        pll_rx_.reset(data.right_x);
        pll_ry_.reset(data.right_y);
        right_initialized_ = true;
        right_just_inited  = true;
    }

    if (!left_initialized_ && !right_initialized_) {
        last_time_ms_       = data.tick_ms;
        state_.last_time_ms = data.tick_ms;
        return;
    }
    if (left_just_inited || right_just_inited) {
        last_time_ms_       = data.tick_ms;
        state_.last_time_ms = data.tick_ms;
        return;
    }

    // 计算 dt（处理 uint32 溢出回绕）
    unsigned int t0 = last_time_ms_;
    unsigned int t1 = data.tick_ms;
    unsigned int dt_ms = (t1 >= t0) ? (t1 - t0)
                                     : (t1 + (0xFFFFFFFFu - t0) + 1u);
    float dt_s = static_cast<float>(dt_ms) / 1000.0f;

    if (dt_s < MIN_DT || dt_s > MAX_DT) {
        last_time_ms_      = data.tick_ms;
        state_.last_time_ms = data.tick_ms;
        return;
    }
    state_.dt_s         = dt_s;
    state_.last_time_ms = last_time_ms_;

    // ---- PLL 更新（4路独立）----
    if (have_left) {
        pll_lx_.update(data.left_x, dt_s);
        pll_ly_.update(data.left_y, dt_s);
    }
    if (have_right) {
        pll_rx_.update(data.right_x, dt_s);
        pll_ry_.update(data.right_y, dt_s);
    }

    // 读取 PLL 速度
    state_.pll_l_vx = have_left  ? pll_lx_.vel() : 0.0f;
    state_.pll_l_vy = have_left  ? pll_ly_.vel() : 0.0f;
    state_.pll_r_vx = have_right ? pll_rx_.vel() : 0.0f;
    state_.pll_r_vy = have_right ? pll_ry_.vel() : 0.0f;

    // ---- 双路融合 → body 速度 + 角速度 ----
    float frame_dx_robot = 0.0f;
    float frame_dy_robot = 0.0f;
    float frame_dtheta   = 0.0f;

    if (have_left && have_right) {
        float dx1 = data.left_x  - left_last_x_;
        float dy1 = data.left_y  - left_last_y_;
        float dx2 = data.right_x - right_last_x_;
        float dy2 = data.right_y - right_last_y_;

        float dx_robot = 0.5f * (dx1 + dx2);
        float dy_robot = 0.5f * (dy1 + dy2);

        float sin_arg = (dx2 - dx1) / (2.0f * HALF_BASELINE);
        if (sin_arg >  1.0f) sin_arg =  1.0f;
        if (sin_arg < -1.0f) sin_arg = -1.0f;
        float dtheta = asinf(sin_arg);
        state_.raw_dtheta = dtheta;

        // body 速度使用 PLL 融合（不是原始差分）
        state_.body_vx = 0.5f * (state_.pll_l_vx + state_.pll_r_vx);
        state_.body_vy = 0.5f * (state_.pll_l_vy + state_.pll_r_vy);
        state_.omega_z = dtheta / dt_s;
        frame_dx_robot = dx_robot;
        frame_dy_robot = dy_robot;
        frame_dtheta   = dtheta;

    } else if (have_left) {
        float dx1 = data.left_x - left_last_x_;
        float dy1 = data.left_y - left_last_y_;

        state_.body_vx    = state_.pll_l_vx;
        state_.body_vy    = state_.pll_l_vy;
        state_.omega_z    = 0.0f;
        state_.raw_dtheta = 0.0f;
        frame_dx_robot = dx1;
        frame_dy_robot = dy1;
        frame_dtheta   = 0.0f;

    } else if (have_right) {
        float dx2 = data.right_x - right_last_x_;
        float dy2 = data.right_y - right_last_y_;

        state_.body_vx    = state_.pll_r_vx;
        state_.body_vy    = state_.pll_r_vy;
        state_.omega_z    = 0.0f;
        state_.raw_dtheta = 0.0f;
        frame_dx_robot = dx2;
        frame_dy_robot = dy2;
        frame_dtheta   = 0.0f;

    } else {
        state_.body_vx    = 0.0f;
        state_.body_vy    = 0.0f;
        state_.omega_z    = 0.0f;
        state_.raw_dtheta = 0.0f;
    }

    // ---- 六轴静止检测 + IMU 偏置估计 ----
    const float flow_speed_norm = sqrtf(state_.body_vx * state_.body_vx
                                      + state_.body_vy * state_.body_vy);

    if (data.imu_valid) {
        imu_bias_.update(data.imu_acc_x, data.imu_acc_y, data.imu_acc_z,
                         data.imu_gyro_x, data.imu_gyro_y, data.imu_gyro_z,
                         flow_speed_norm, dt_s);
    } else {
        // IMU 无效时仅用光流做简单静止检测
        const bool flow_still = (flow_speed_norm < control_config::kStationaryFlowThresholdMmPerS);
        // 简化：直接用 flow 判断，保持偏置不变
        // imu_bias_ 保持上一次的状态
        state_.is_stationary = flow_still;
    }

    state_.is_stationary = imu_bias_.is_stationary();
    state_.imu_bias_ax = imu_bias_.bias_ax();
    state_.imu_bias_ay = imu_bias_.bias_ay();
    state_.imu_bias_gx = imu_bias_.bias_gx();
    state_.imu_bias_gy = imu_bias_.bias_gy();
    state_.imu_bias_gz = imu_bias_.bias_gz();

    // ---- 纯光流积分位姿 ----
    const float actual_dtheta = data.imu_valid
        ? (data.imu_gyro_z * kDegToRad * dt_s) : frame_dtheta;
    const float theta_mid = state_.flow_yaw + 0.5f * actual_dtheta;
    const float c = cosf(theta_mid);
    const float s = sinf(theta_mid);
    state_.flow_px += frame_dx_robot * c + frame_dy_robot * s;
    state_.flow_py += -frame_dx_robot * s + frame_dy_robot * c;
    state_.flow_yaw += actual_dtheta;

    // ---- 简化卡尔曼 ----
    const bool flow_available = (data.valid_mask != 0u);

    // 加速度：IMU acc → 转换为 mm/s² 并扣除偏置
    const float ax_mm = data.imu_valid
        ? (data.imu_acc_x - imu_bias_.bias_ax()) * 1000.0f : 0.0f;
    const float ay_mm = data.imu_valid
        ? (data.imu_acc_y - imu_bias_.bias_ay()) * 1000.0f : 0.0f;

    // 卡尔曼初始化
    if (!kf_inited_ && flow_available) {
        kf_.init(0.0f, 0.0f, state_.body_vx, state_.body_vy, 0.0f, 0.0f, 1000.0f);
        kf_inited_ = true;
    }

    // ---- 静止状态进入/退出 R 管理（修订#6） ----
    const bool entering_stationary  = state_.is_stationary && !prev_stationary_;
    const bool exiting_stationary   = !state_.is_stationary && prev_stationary_;

    if (entering_stationary) {
        // 进入静止：低 R，高质量零速观测
        kf_.setNoise(kQPos, control_config::kOptFlowKfQVel,
                     kQBiasAx, kQBiasAy,
                     control_config::kOptFlowKfRVelFixed * 0.1f, kRPos);
    }
    if (exiting_stationary) {
        // 退出静止：立即恢复固定 R（修订#6）
        kf_.setNoise(kQPos, control_config::kOptFlowKfQVel,
                     kQBiasAx, kQBiasAy,
                     control_config::kOptFlowKfRVelFixed, kRPos);
    }
    prev_stationary_ = state_.is_stationary;

    // IMU 预测（使用偏置修正后的加速度）
    if (kf_inited_ && data.imu_valid) {
        kf_.predict(ax_mm, ay_mm, dt_s);
    }

    // 速度更新：只要有光流就更新（无质量门控），静止时强制零速
    if (kf_inited_ && flow_available) {
        if (state_.is_stationary) {
            // 零速强制
            kf_.updateVel(0.0f, 0.0f);
        } else {
            kf_.updateVel(state_.body_vx, state_.body_vy);
        }
    }

    if (kf_inited_) {
        state_.kf_vx = kf_.vx();
        state_.kf_vy = kf_.vy();
        state_.kf_px = kf_.px();
        state_.kf_py = kf_.py();
    } else {
        state_.kf_vx = state_.body_vx;
        state_.kf_vy = state_.body_vy;
        state_.kf_px = 0.0f;
        state_.kf_py = 0.0f;
    }

    // ---- 更新 last 值 ----
    if (have_left) {
        left_last_x_ = data.left_x;
        left_last_y_ = data.left_y;
    }
    if (have_right) {
        right_last_x_ = data.right_x;
        right_last_y_ = data.right_y;
    }
    last_time_ms_ = data.tick_ms;
}
