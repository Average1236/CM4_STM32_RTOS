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

class Kalman2DPosVel {
public:
    Kalman2DPosVel();
    void setNoise(float q_pos, float q_vel, float r_vel, float r_pos);
    void init(float px0, float py0, float vx0, float vy0, float p0);
    void predict(float ax, float ay, float dt);   // IMU 加速度预测
    void updateVel(float vx_meas, float vy_meas); // 光流速度更新
    void updatePos(float px_meas, float py_meas); // 光流位置更新（可选）
    float px() const { return x_[0]; }
    float py() const { return x_[1]; }
    float vx() const { return x_[2]; }
    float vy() const { return x_[3]; }
private:
    float Q_[4];   // 过程噪声对角线
    float Rv_[2];  // 速度观测噪声
    float Rp_[2];  // 位置观测噪声
    float x_[4];   // 状态 [px, py, vx, vy]，单位（mm 和 mm/s）
    float P_[16];  // 协方差矩阵（行主序）
};

class OptFlow {
public:
    // ============================================================
    // Data_t: 双光流双路输入
    //   字段: left_x/y, right_x/y, tick_ms, valid_mask + IMU 字段
    // ============================================================
    struct Data_t {
        float        left_x;
        float        left_y;
        float        right_x;
        float        right_y;
        unsigned int tick_ms;
        unsigned int valid_mask;
        unsigned int left_tick_ms;   // Left separate timestamp
        unsigned int right_tick_ms;  // Right separate timestamp
        // --- IMU 字段 ---
        float imu_acc_x;   // 本体坐标系加速度 X（m/s²）
        float imu_acc_y;   // 本体坐标系加速度 Y（m/s²）
        float imu_omega_z; // 陀螺仪 Z 轴角速度（rad/s）
        bool  imu_valid;   // IMU 数据是否有效
    };

    // ============================================================
    // State_t: 光流输出
    //   left/right_vx/vy   左右传感器各自速度（调试用）
    //   body_vx/vy         机器人本体速度（mm/s）
    //   omega_z            本体角速度（rad/s）
    //   raw_dtheta         每帧角度增量
    //   flow_px/py/yaw     纯光流积分得到的位姿
    //   kf_*               卡尔曼融合输出
    //   flow_quality/weight 光流质量评估
    // ============================================================
    struct State_t {
        // 左右传感器各自速度（调试用）
        float left_vx;
        float left_vy;
        float right_vx;
        float right_vy;

        // 机器人本体速度（md 推导结果）
        float body_vx;   // dx_robot / dt
        float body_vy;   // dy_robot / dt
        float omega_z;   // dtheta / dt（rad/s）

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
        float kf_omega_z;         // 融合后角速度（互补滤波）
        float flow_quality;       // 光流质量评分 [0,1]
        float flow_weight;        // 光流权重 [0,1]
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
    static constexpr float kQPos = 0.1f;
    static constexpr float kQVel = 5.0f;
    static constexpr float kRVelMin = 300.0f;
    static constexpr float kRVelMax = 12000.0f;
    static constexpr float kRPos = 1e6f;
    static constexpr float kMinUpdateQuality = 0.08f;
    static constexpr float kResidualBadMmPerS = 1800.0f;
    static constexpr float kZeroSpeedMmPerS = 25.0f;
    static constexpr float kAccelActiveMmPerS2 = 400.0f;
    static constexpr uint8_t kZeroStreakBad = 6u;

    State_t      state_;

    // 左右独立初始化标志
    bool         left_initialized_;
    bool         right_initialized_;

    // 上一帧左右传感器位置和时间戳
    float        left_last_x_;
    float        left_last_y_;
    float        right_last_x_;
    float        right_last_y_;
    unsigned int last_time_ms_;

    // 卡尔曼私有成员
    Kalman2DPosVel kf_;
    bool kf_inited_;
    float kf_last_px_;
    float kf_last_py_;
    uint8_t flow_zero_streak_;

    // 互补滤波私有成员（用于 omega_z）
    float cf_omega_z_;
    static constexpr float kCfAlpha = 0.7f; // 光流权重
};

#endif // __OPT_FLOW_HPP
