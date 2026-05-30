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
    setNoise(0.1f, 5.0f, 300.0f, 1e6f);
    init(0.0f, 0.0f, 0.0f, 0.0f, 500.0f);
}

void Kalman2DPosVel::setNoise(float q_pos, float q_vel, float r_vel, float r_pos) {
    Q_[0] = q_pos; Q_[1] = q_pos; Q_[2] = q_vel; Q_[3] = q_vel;
    Rv_[0] = r_vel; Rv_[1] = r_vel;
    Rp_[0] = r_pos; Rp_[1] = r_pos;
}

void Kalman2DPosVel::init(float px0, float py0, float vx0, float vy0, float p0) {
    x_[0] = px0; x_[1] = py0; x_[2] = vx0; x_[3] = vy0;
    for (int i = 0; i < 16; ++i) P_[i] = 0.0f;
    P_[0] = p0; P_[5] = p0; P_[10] = p0; P_[15] = p0;
}

void Kalman2DPosVel::predict(float ax, float ay, float dt) {
    if (dt <= 0.0f) return;
    const float dt2_2 = 0.5f * dt * dt;
    float px = x_[0] + dt * x_[2] + dt2_2 * ax;
    float py = x_[1] + dt * x_[3] + dt2_2 * ay;
    float vx = x_[2] + dt * ax;
    float vy = x_[3] + dt * ay;
    x_[0] = px; x_[1] = py; x_[2] = vx; x_[3] = vy;
    float AP[16];
    AP[0]  = P_[0] + dt * P_[8];
    AP[1]  = P_[1] + dt * P_[9];
    AP[2]  = P_[2] + dt * P_[10];
    AP[3]  = P_[3] + dt * P_[11];
    AP[4]  = P_[4] + dt * P_[12];
    AP[5]  = P_[5] + dt * P_[13];
    AP[6]  = P_[6] + dt * P_[14];
    AP[7]  = P_[7] + dt * P_[15];
    AP[8]  = P_[8];
    AP[9]  = P_[9];
    AP[10] = P_[10];
    AP[11] = P_[11];
    AP[12] = P_[12];
    AP[13] = P_[13];
    AP[14] = P_[14];
    AP[15] = P_[15];
    float Pn[16];
    Pn[0]  = AP[0];
    Pn[4]  = AP[4];
    Pn[8]  = AP[8];
    Pn[12] = AP[12];
    Pn[1]  = AP[1];
    Pn[5]  = AP[5];
    Pn[9]  = AP[9];
    Pn[13] = AP[13];
    Pn[2]  = AP[0] * dt + AP[2];
    Pn[6]  = AP[4] * dt + AP[6];
    Pn[10] = AP[8] * dt + AP[10];
    Pn[14] = AP[12] * dt + AP[14];
    Pn[3]  = AP[1] * dt + AP[3];
    Pn[7]  = AP[5] * dt + AP[7];
    Pn[11] = AP[9] * dt + AP[11];
    Pn[15] = AP[13] * dt + AP[15];
    Pn[0]  += Q_[0];
    Pn[5]  += Q_[1];
    Pn[10] += Q_[2];
    Pn[15] += Q_[3];
    for (int i = 0; i < 16; ++i) P_[i] = Pn[i];
}

void Kalman2DPosVel::updateVel(float vx_meas, float vy_meas) {
    float y0 = vx_meas - x_[2];
    float y1 = vy_meas - x_[3];
    float S00 = P_[10] + Rv_[0];
    float S01 = P_[11];
    float S10 = P_[14];
    float S11 = P_[15] + Rv_[1];
    float det = S00 * S11 - S01 * S10;
    if (det == 0.0f) return;
    float invS00 =  S11 / det;
    float invS01 = -S01 / det;
    float invS10 = -S10 / det;
    float invS11 =  S00 / det;
    float PHt[8];
    PHt[0] = P_[2];  PHt[1] = P_[3];
    PHt[2] = P_[6];  PHt[3] = P_[7];
    PHt[4] = P_[10]; PHt[5] = P_[11];
    PHt[6] = P_[14]; PHt[7] = P_[15];
    float K0 = PHt[0] * invS00 + PHt[1] * invS10;
    float K1 = PHt[0] * invS01 + PHt[1] * invS11;
    float K2 = PHt[2] * invS00 + PHt[3] * invS10;
    float K3 = PHt[2] * invS01 + PHt[3] * invS11;
    float K4 = PHt[4] * invS00 + PHt[5] * invS10;
    float K5 = PHt[4] * invS01 + PHt[5] * invS11;
    float K6 = PHt[6] * invS00 + PHt[7] * invS10;
    float K7 = PHt[6] * invS01 + PHt[7] * invS11;
    x_[0] += K0 * y0 + K1 * y1;
    x_[1] += K2 * y0 + K3 * y1;
    x_[2] += K4 * y0 + K5 * y1;
    x_[3] += K6 * y0 + K7 * y1;
    float KH_col2_0 = K0; float KH_col3_0 = K1;
    float KH_col2_1 = K2; float KH_col3_1 = K3;
    float KH_col2_2 = K4; float KH_col3_2 = K5;
    float KH_col2_3 = K6; float KH_col3_3 = K7;
    float Pn[16];
    for (int i = 0; i < 16; ++i) Pn[i] = P_[i];
    Pn[0*4 + 2] = P_[2]  - (KH_col2_0 * P_[10] + KH_col3_0 * P_[14]);
    Pn[1*4 + 2] = P_[6]  - (KH_col2_1 * P_[10] + KH_col3_1 * P_[14]);
    Pn[2*4 + 2] = P_[10] - (KH_col2_2 * P_[10] + KH_col3_2 * P_[14]);
    Pn[3*4 + 2] = P_[14] - (KH_col2_3 * P_[10] + KH_col3_3 * P_[14]);
    Pn[0*4 + 3] = P_[3]  - (KH_col2_0 * P_[11] + KH_col3_0 * P_[15]);
    Pn[1*4 + 3] = P_[7]  - (KH_col2_1 * P_[11] + KH_col3_1 * P_[15]);
    Pn[2*4 + 3] = P_[11] - (KH_col2_2 * P_[11] + KH_col3_2 * P_[15]);
    Pn[3*4 + 3] = P_[15] - (KH_col2_3 * P_[11] + KH_col3_3 * P_[15]);
    Pn[0] = P_[0] - (K0 * P_[2] + K1 * P_[3]);
    Pn[1] = P_[1] - (K0 * P_[6] + K1 * P_[7]);
    Pn[4] = P_[4] - (K2 * P_[2] + K3 * P_[3]);
    Pn[5] = P_[5] - (K2 * P_[6] + K3 * P_[7]);
    for (int i = 0; i < 16; ++i) P_[i] = Pn[i];
}

void Kalman2DPosVel::updatePos(float px_meas, float py_meas) {
    float y0 = px_meas - x_[0];
    float y1 = py_meas - x_[1];
    float S00 = P_[0] + Rp_[0];
    float S01 = P_[1];
    float S10 = P_[4];
    float S11 = P_[5] + Rp_[1];
    float det = S00 * S11 - S01 * S10;
    if (det == 0.0f) return;
    float invS00 =  S11 / det;
    float invS01 = -S01 / det;
    float invS10 = -S10 / det;
    float invS11 =  S00 / det;
    float PHt[8];
    PHt[0] = P_[0];  PHt[1] = P_[1];
    PHt[2] = P_[4];  PHt[3] = P_[5];
    PHt[4] = P_[8];  PHt[5] = P_[9];
    PHt[6] = P_[12]; PHt[7] = P_[13];
    float K0 = PHt[0] * invS00 + PHt[1] * invS10;
    float K1 = PHt[0] * invS01 + PHt[1] * invS11;
    float K2 = PHt[2] * invS00 + PHt[3] * invS10;
    float K3 = PHt[2] * invS01 + PHt[3] * invS11;
    float K4 = PHt[4] * invS00 + PHt[5] * invS10;
    float K5 = PHt[4] * invS01 + PHt[5] * invS11;
    float K6 = PHt[6] * invS00 + PHt[7] * invS10;
    float K7 = PHt[6] * invS01 + PHt[7] * invS11;
    x_[0] += K0 * y0 + K1 * y1;
    x_[1] += K2 * y0 + K3 * y1;
    x_[2] += K4 * y0 + K5 * y1;
    x_[3] += K6 * y0 + K7 * y1;
    float Pn[16];
    for (int i = 0; i < 16; ++i) Pn[i] = P_[i];
    Pn[0]  = P_[0]  - (K0 * P_[0] + K1 * P_[4]);
    Pn[4]  = P_[4]  - (K2 * P_[0] + K3 * P_[4]);
    Pn[8]  = P_[8]  - (K4 * P_[0] + K5 * P_[4]);
    Pn[12] = P_[12] - (K6 * P_[0] + K7 * P_[4]);
    Pn[1]  = P_[1]  - (K0 * P_[1] + K1 * P_[5]);
    Pn[5]  = P_[5]  - (K2 * P_[1] + K3 * P_[5]);
    Pn[9]  = P_[9]  - (K4 * P_[1] + K5 * P_[5]);
    Pn[13] = P_[13] - (K6 * P_[1] + K7 * P_[5]);
    for (int i = 0; i < 16; ++i) P_[i] = Pn[i];
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
    kf_.setNoise(kQPos, kQVel, kRVelMin, kRPos);
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
    kf_.init(0.0f, 0.0f, 0.0f, 0.0f, 1000.0f);
    kf_.setNoise(kQPos, kQVel, kRVelMin, kRPos);
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
        kf_.init(0.0f, 0.0f, state_.body_vx, state_.body_vy, 1000.0f);
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
    kf_.setNoise(kQPos, kQVel, r_vel_eff, kRPos);

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
