#ifndef __OPT_FLOW_HPP
#define __OPT_FLOW_HPP

#include "Task/utils.hpp"

#include <cstdint>
#include <cstddef>

// ============================================================
// 双光流传感器间距参数
//   原: OPTFLOW_OFFSET_X / OPTFLOW_OFFSET_Y (单光流安装偏移)
//   现: 双光流基线半长，单位 mm，对应 md 推导中的 l
// ============================================================
#define OPTFLOW_HALF_BASELINE_MM (34.0f)

// valid_mask 位定义
#define OPTFLOW_MASK_LEFT  (0x01u)
#define OPTFLOW_MASK_RIGHT (0x02u)

// ============================================================
// PositionPLL — 2阶线性位置跟踪锁相环
//   参考 MotorDMH3510::update_wheel_speed_pll_from_pos
//   线性位移无需角度 wrapping
// ============================================================
class PositionPLL {
public:
    PositionPLL(float bandwidth_hz, float zero_snap_threshold, float max_jump_mm,
                uint8_t jump_confirm_frames);
    void update(float measured_pos_mm, float dt_s);
    void reset(float measured_pos_mm);
    float vel() const { return vel_est_mm_s_; }
    float pos() const { return pos_est_mm_; }
    bool  initialized() const { return initialized_; }

private:
    float pos_est_mm_;
    float vel_est_mm_s_;
    float kp_;
    float ki_;
    float zero_snap_threshold_;
    float max_jump_mm_;
    uint8_t jump_confirm_frames_;
    uint8_t jump_count_;
    bool  initialized_;
};

// ============================================================
// SixAxisImuBias — 六轴IMU静止检测 + 偏置估计
//   利用全部6轴 IMU（3加速度+3陀螺仪）+ 光流PLL速度
//   静止窗口内估计 accel bias + gyro bias
// ============================================================
class SixAxisImuBias {
public:
    SixAxisImuBias();

    // 每帧调用：检测静止 + 管理采样窗口 + 更新偏置
    //   ax, ay, az    : 加速度 m/s²
    //   gx, gy, gz    : 陀螺仪 deg/s
    //   flow_speed_norm: 光流速度模长 mm/s
    //   dt_s          : 帧间隔
    void update(float ax, float ay, float az,
                float gx, float gy, float gz,
                float flow_speed_norm, float dt_s);

    bool  is_stationary() const { return stationary_confirmed_; }
    float bias_ax() const { return bias_ax_; }
    float bias_ay() const { return bias_ay_; }
    float bias_gx() const { return bias_gx_; }
    float bias_gy() const { return bias_gy_; }
    float bias_gz() const { return bias_gz_; }

private:
    // 状态机
    enum class Phase { kNormal, kCollecting, kVerifying };
    Phase phase_;
    uint8_t confirm_count_;
    uint8_t collect_count_;

    // 静止确认
    bool stationary_confirmed_;

    // 采样累加器（加速度 m/s², 陀螺 deg/s）
    float sum_ax_, sum_ay_;
    float sum_gx_, sum_gy_, sum_gz_;

    // 先验加速度幅值（用于幅值稳定性比较）
    float prev_acc_norm_;

    // EMA 偏置估计
    float bias_ax_, bias_ay_;
    float bias_gx_, bias_gy_, bias_gz_;
};

// ============================================================
// Kalman2DPosVel — 6状态卡尔曼（保留，简化使用方式）
// ============================================================
class Kalman2DPosVel {
public:
    Kalman2DPosVel();
    void setNoise(float q_pos, float q_vel, float q_bias_ax, float q_bias_ay, float r_vel, float r_pos);
    void init(float px0, float py0, float vx0, float vy0, float bax0, float bay0, float p0);
    void predict(float ax, float ay, float dt);   // IMU 加速度预测（内部扣除偏置）
    void updateVel(float vx_meas, float vy_meas); // 光流速度更新
    void updatePos(float px_meas, float py_meas); // 光流位置更新（可选）
    float px() const { return x_[0]; }
    float py() const { return x_[1]; }
    float vx() const { return x_[2]; }
    float vy() const { return x_[3]; }
    float bax() const { return x_[4]; }
    float bay() const { return x_[5]; }
private:
    float Q_[6];   // 过程噪声对角线: [q_pos, q_pos, q_vel, q_vel, q_bias_ax, q_bias_ay]
    float Rv_[2];  // 速度观测噪声
    float Rp_[2];  // 位置观测噪声
    float x_[6];   // 状态 [px, py, vx, vy, b_ax, b_ay]，单位（mm, mm/s, mm/s²）
    float P_[36];  // 协方差矩阵（6×6 行主序）
};

// ============================================================
// OptFlow — 双光流传感器处理主类
// ============================================================
class OptFlow {
public:
    struct Data_t {
        float        left_x;
        float        left_y;
        float        right_x;
        float        right_y;
        unsigned int tick_ms;
        unsigned int valid_mask;
        unsigned int left_tick_ms;   // Left separate timestamp
        unsigned int right_tick_ms;  // Right separate timestamp
        // --- IMU 6轴字段 ---
        float imu_acc_x;   // 本体坐标系加速度 X（m/s²）
        float imu_acc_y;   // 本体坐标系加速度 Y（m/s²）
        float imu_acc_z;   // 本体坐标系加速度 Z（m/s²）
        float imu_gyro_x;  // 陀螺仪 X 轴角速度（deg/s）
        float imu_gyro_y;  // 陀螺仪 Y 轴角速度（deg/s）
        float imu_gyro_z;  // 陀螺仪 Z 轴角速度（deg/s）→ rad/s after conversion
        bool  imu_valid;   // IMU 数据是否有效
    };

    struct State_t {
        // 左右传感器 PLL 速度（调试用）
        float pll_l_vx;
        float pll_l_vy;
        float pll_r_vx;
        float pll_r_vy;

        // 机器人本体速度（双路PLL融合结果）
        float body_vx;   // mm/s
        float body_vy;   // mm/s
        float omega_z;   // rad/s（dtheta / dt）

        // IMU 偏置估计
        float imu_bias_ax;   // m/s²
        float imu_bias_ay;   // m/s²
        float imu_bias_gx;   // deg/s
        float imu_bias_gy;   // deg/s
        float imu_bias_gz;   // deg/s

        // 静止检测
        bool  is_stationary;

        // 每帧原始角度增量（调试用）
        float raw_dtheta;

        // 纯光流积分得到的中心位姿
        float flow_px;
        float flow_py;
        float flow_yaw;

        // 时间
        unsigned int time_ms;
        unsigned int last_time_ms;
        float        dt_s;

        // 卡尔曼融合输出
        float kf_vx;              // 融合后本体速度 X（mm/s）
        float kf_vy;              // 融合后本体速度 Y（mm/s）
        float kf_px;              // 融合后位置 X（mm）
        float kf_py;              // 融合后位置 Y（mm）
    };

    OptFlow();
    ~OptFlow() = default;

    void process(const Data_t& sensor_data);

    const State_t& get_state() const { return state_; }
    void reset();

private:
    static constexpr float HALF_BASELINE = OPTFLOW_HALF_BASELINE_MM;
    static constexpr float MIN_DT = 0.001f;
    static constexpr float MAX_DT = 0.1f;

    // 卡尔曼固定噪声（不写成类内常量，用 control_params.hpp 的值初始化）
    static constexpr float kQPos = 0.1f;
    static constexpr float kQBiasAx = 0.01f;
    static constexpr float kQBiasAy = 0.01f;
    static constexpr float kRPos = 1e6f;

    State_t      state_;

    // PLL 实例（4路独立：左x 左y 右x 右y）
    PositionPLL  pll_lx_;
    PositionPLL  pll_ly_;
    PositionPLL  pll_rx_;
    PositionPLL  pll_ry_;

    // 六轴偏置估计器
    SixAxisImuBias imu_bias_;

    // 上一帧左右传感器位置
    float        left_last_x_;
    float        left_last_y_;
    float        right_last_x_;
    float        right_last_y_;
    unsigned int last_time_ms_;

    // 左/右独立初始化标志
    bool         left_initialized_;
    bool         right_initialized_;

    // 卡尔曼
    Kalman2DPosVel kf_;
    bool kf_inited_;

    // 上一个静止状态（用于检测进入/退出）
    bool prev_stationary_;
};

#endif // __OPT_FLOW_HPP
