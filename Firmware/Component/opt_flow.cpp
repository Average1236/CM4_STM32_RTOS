// ============================================================
// 双光流 + IMU 卡尔曼融合
// ============================================================

#include "opt_flow.hpp"
#include <cmath>
#include <cstring>
#include "stm32f4xx_hal.h"

namespace {

constexpr float kMaxLinearSpeedMmPerS = 3500.0f;
constexpr float kMaxAngularSpeedRadPerS = 50.0f;

float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

float hold_previous(float value, float previous_value, float limit) {
    return (fabsf(value) > limit) ? previous_value : value;
}

void apply_velocity_filter(OptFlow::State_t& state, const OptFlow::State_t& previous_state) {
    state.left_vx = hold_previous(state.left_vx, previous_state.left_vx, kMaxLinearSpeedMmPerS);
    state.left_vy = hold_previous(state.left_vy, previous_state.left_vy, kMaxLinearSpeedMmPerS);
    state.right_vx = hold_previous(state.right_vx, previous_state.right_vx, kMaxLinearSpeedMmPerS);
    state.right_vy = hold_previous(state.right_vy, previous_state.right_vy, kMaxLinearSpeedMmPerS);
    state.body_vx = hold_previous(state.body_vx, previous_state.body_vx, kMaxLinearSpeedMmPerS);
    state.body_vy = hold_previous(state.body_vy, previous_state.body_vy, kMaxLinearSpeedMmPerS);
    state.omega_z = hold_previous(state.omega_z, previous_state.omega_z, kMaxAngularSpeedRadPerS);

    if (state.omega_z == previous_state.omega_z && fabsf(previous_state.omega_z) <= kMaxAngularSpeedRadPerS) {
        state.raw_dtheta = previous_state.raw_dtheta;
    }
}

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
OptFlow::OptFlow() : left_initialized_(false), right_initialized_(false),
                     left_last_x_(0.0f), left_last_y_(0.0f),
                     right_last_x_(0.0f), right_last_y_(0.0f),
                     last_time_ms_(0),
                     kf_inited_(false),
                     kf_last_px_(0.0f),
                     kf_last_py_(0.0f),
                     flow_zero_streak_(0),
                     cf_omega_z_(0.0f)
                     {
    memset(&state_, 0, sizeof(state_));
    kf_.setNoise(kQPos, kQVel, kQBiasAx, kQBiasAy, kRVelMin, kRPos);
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
    kf_last_px_  = 0.0f;
    kf_last_py_  = 0.0f;
    flow_zero_streak_ = 0;
    cf_omega_z_  = 0.0f;
    kf_.init(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1000.0f);
    kf_.setNoise(kQPos, kQVel, kQBiasAx, kQBiasAy, kRVelMin, kRPos);
}

// ============================================================
// process(Data_t)
//   传感器安装坐标: 左 (0, +l), 右 (0, -l)
//   dx_robot = (dx1 + dx2) / 2
//   dy_robot = (dy1 + dy2) / 2
//   dtheta   = arcsin((dx2 - dx1) / (2*l))
//   vx = dx_robot / dt, vy = dy_robot / dt, omega = dtheta / dt
// ============================================================
void OptFlow::process(const Data_t& data) {
    const State_t previous_state = state_;

    state_.time_ms = data.tick_ms;

    const bool have_left  = (data.valid_mask & OPTFLOW_MASK_LEFT)  != 0u;
    const bool have_right = (data.valid_mask & OPTFLOW_MASK_RIGHT) != 0u;

    // 左右独立初始化
    bool left_just_inited  = false;
    bool right_just_inited = false;

    if (have_left && !left_initialized_) {
        left_last_x_      = data.left_x;
        left_last_y_      = data.left_y;
        left_initialized_ = true;
        left_just_inited  = true;
    }
    if (have_right && !right_initialized_) {
        right_last_x_      = data.right_x;
        right_last_y_      = data.right_y;
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

    float frame_dx_robot = 0.0f;
    float frame_dy_robot = 0.0f;
    float frame_dtheta = 0.0f;

    if (have_left && have_right) {
        float dx1 = data.left_x  - left_last_x_;
        float dy1 = data.left_y  - left_last_y_;
        float dx2 = data.right_x - right_last_x_;
        float dy2 = data.right_y - right_last_y_;

        state_.left_vx  = dx1 / dt_s;
        state_.left_vy  = dy1 / dt_s;
        state_.right_vx = dx2 / dt_s;
        state_.right_vy = dy2 / dt_s;

        float dx_robot = 0.5f * (dx1 + dx2);
        float dy_robot = 0.5f * (dy1 + dy2);

        float sin_arg = (dx2 - dx1) / (2.0f * HALF_BASELINE);
        if (sin_arg >  1.0f) sin_arg =  1.0f;
        if (sin_arg < -1.0f) sin_arg = -1.0f;
        float dtheta = asinf(sin_arg);
        state_.raw_dtheta = dtheta;

        state_.body_vx = dx_robot / dt_s;
        state_.body_vy = dy_robot / dt_s;
        state_.omega_z = dtheta   / dt_s;
        frame_dx_robot = dx_robot;
        frame_dy_robot = dy_robot;
        frame_dtheta   = dtheta;

    } else if (have_left) {
        float dx1 = data.left_x - left_last_x_;
        float dy1 = data.left_y - left_last_y_;

        state_.left_vx  = dx1 / dt_s;
        state_.left_vy  = dy1 / dt_s;
        state_.right_vx = 0.0f;
        state_.right_vy = 0.0f;

        state_.body_vx    = dx1 / dt_s;
        state_.body_vy    = dy1 / dt_s;
        state_.omega_z    = 0.0f;
        state_.raw_dtheta = 0.0f;
        frame_dx_robot = dx1;
        frame_dy_robot = dy1;
        frame_dtheta   = 0.0f;

    } else if (have_right) {
        float dx2 = data.right_x - right_last_x_;
        float dy2 = data.right_y - right_last_y_;

        state_.left_vx  = 0.0f;
        state_.left_vy  = 0.0f;
        state_.right_vx = dx2 / dt_s;
        state_.right_vy = dy2 / dt_s;

        state_.body_vx    = dx2 / dt_s;
        state_.body_vy    = dy2 / dt_s;
        state_.omega_z    = 0.0f;
        state_.raw_dtheta = 0.0f;
        frame_dx_robot = dx2;
        frame_dy_robot = dy2;
        frame_dtheta   = 0.0f;

    } else {
        state_.left_vx    = 0.0f;
        state_.left_vy    = 0.0f;
        state_.right_vx   = 0.0f;
        state_.right_vy   = 0.0f;
        state_.body_vx    = 0.0f;
        state_.body_vy    = 0.0f;
        state_.omega_z    = 0.0f;
        state_.raw_dtheta = 0.0f;
        frame_dx_robot = 0.0f;
        frame_dy_robot = 0.0f;
        frame_dtheta   = 0.0f;
    }

    // 使用 IMU 角速度做姿态增量（光流双路时也优先 IMU）
    const float actual_dtheta = data.imu_valid ? (data.imu_omega_z * dt_s) : frame_dtheta;

    // 纯光流积分位姿（中心点法）
    const float theta_mid = state_.flow_yaw + 0.5f * actual_dtheta;
    const float c = cosf(theta_mid);
    const float s = sinf(theta_mid);
    state_.flow_px += frame_dx_robot * c + frame_dy_robot * s;
    state_.flow_py += -frame_dx_robot * s + frame_dy_robot * c;
    state_.flow_yaw += actual_dtheta;

    apply_velocity_filter(state_, previous_state);

    const bool flow_available = (data.valid_mask != 0u);
    const float ax_mm = data.imu_acc_x * 1000.0f;
    const float ay_mm = data.imu_acc_y * 1000.0f;

    // 卡尔曼初始化
    if (!kf_inited_ && flow_available) {
        kf_.init(0.0f, 0.0f, state_.body_vx, state_.body_vy, 0.0f, 0.0f, 1000.0f);
        kf_last_px_ = 0.0f;
        kf_last_py_ = 0.0f;
        kf_inited_  = true;
    }

    // IMU 预测
    if (kf_inited_ && data.imu_valid) {
        kf_.predict(ax_mm, ay_mm, dt_s);
    }

    // 光流质量评估（用于动态调整 updateVel 增益）
    float availability_score = 0.0f;
    if ((data.valid_mask & (OPTFLOW_MASK_LEFT | OPTFLOW_MASK_RIGHT)) == (OPTFLOW_MASK_LEFT | OPTFLOW_MASK_RIGHT)) {
        availability_score = 1.0f;
    } else if (flow_available) {
        availability_score = 0.55f;
    }

    const float flow_speed_norm = sqrtf(state_.body_vx * state_.body_vx + state_.body_vy * state_.body_vy);
    const float imu_acc_norm = sqrtf(ax_mm * ax_mm + ay_mm * ay_mm);
    const bool suspicious_zero = flow_available && (flow_speed_norm < kZeroSpeedMmPerS)
                                 && data.imu_valid && (imu_acc_norm > kAccelActiveMmPerS2);
    if (suspicious_zero) {
        if (flow_zero_streak_ < 255u) flow_zero_streak_++;
    } else if (flow_zero_streak_ > 0u) {
        flow_zero_streak_--;
    }
    const float zero_penalty = clampf(static_cast<float>(flow_zero_streak_) / static_cast<float>(kZeroStreakBad), 0.0f, 1.0f);

    float residual_score = 1.0f;
    if (kf_inited_ && flow_available) {
        const float rvx = state_.body_vx - kf_.vx();
        const float rvy = state_.body_vy - kf_.vy();
        const float residual = sqrtf(rvx * rvx + rvy * rvy);
        residual_score = 1.0f - clampf(residual / kResidualBadMmPerS, 0.0f, 1.0f);
    }

    float flow_quality = availability_score * (1.0f - zero_penalty) * residual_score;
    flow_quality = clampf(flow_quality, 0.0f, 1.0f);

    // 自适应观测噪声
    const float inv_q = 1.0f - flow_quality;
    const float r_vel_eff = kRVelMin + (inv_q * inv_q) * (kRVelMax - kRVelMin);
    kf_.setNoise(kQPos, kQVel, kQBiasAx, kQBiasAy, r_vel_eff, kRPos);

    const bool allow_flow_update = flow_available && (flow_quality >= kMinUpdateQuality);
    if (kf_inited_ && allow_flow_update) {
        kf_.updateVel(state_.body_vx, state_.body_vy);
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

    // 互补滤波权重亦由质量自适应
    float flow_weight = flow_quality;
    if (!flow_available) {
        flow_weight = 0.0f;
    }
    if (!data.imu_valid) {
        flow_weight = flow_available ? 1.0f : 0.0f;
    }
    if (data.imu_valid) {
        const float alpha = clampf(kCfAlpha * flow_weight, 0.0f, 1.0f);
        cf_omega_z_ = alpha * state_.omega_z + (1.0f - alpha) * data.imu_omega_z;
        state_.kf_omega_z = cf_omega_z_;
    } else {
        state_.kf_omega_z = state_.omega_z;
    }

    state_.flow_quality = flow_quality;
    state_.flow_weight = flow_weight;

    // 更新 last 值
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
