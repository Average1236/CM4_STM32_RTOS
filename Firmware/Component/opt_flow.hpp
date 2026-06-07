#ifndef __OPT_FLOW_HPP
#define __OPT_FLOW_HPP

#include "component.hpp"
#include "Task/utils.hpp"

#include <cstdint>
#include <cstddef>

// ============================================================
// 双光流传感器间距参数
// ============================================================
#define OPTFLOW_HALF_BASELINE_MM (34.0f)

// valid_mask 位定义
#define OPTFLOW_MASK_LEFT  (0x01u)
#define OPTFLOW_MASK_RIGHT (0x02u)

// ============================================================
// PositionPLL — 2阶线性位置跟踪锁相环
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
// FlowStationaryDetector — 光流+IMU联合静止检测 + accel偏置估计
//   陀螺偏置已迁移到 IMU 内部
// ============================================================
class FlowStationaryDetector {
public:
    FlowStationaryDetector();

    void update(float ax, float ay, float az,
                float gx, float gy, float gz,
                float flow_speed_norm);

    bool  is_stationary() const { return stationary_confirmed_; }
    float bias_ax() const { return bias_ax_; }
    float bias_ay() const { return bias_ay_; }

private:
    enum class Phase { kNormal, kCollecting, kVerifying };
    Phase phase_;
    uint8_t confirm_count_;
    uint8_t collect_count_;

    bool  stationary_confirmed_;
    float sum_ax_, sum_ay_;
    float prev_acc_norm_;

    float bias_ax_, bias_ay_;
};

// ============================================================
// Kalman2DPosVel — 6状态卡尔曼
// ============================================================
class Kalman2DPosVel {
public:
    Kalman2DPosVel();
    void setNoise(float q_pos, float q_vel, float q_bias_ax, float q_bias_ay, float r_vel, float r_pos);
    void init(float px0, float py0, float vx0, float vy0, float bax0, float bay0, float p0);
    void predict(float ax, float ay, float dt);
    void updateVel(float vx_meas, float vy_meas);
    void updatePos(float px_meas, float py_meas);
    float px() const { return x_[0]; }
    float py() const { return x_[1]; }
    float vx() const { return x_[2]; }
    float vy() const { return x_[3]; }
    float bax() const { return x_[4]; }
    float bay() const { return x_[5]; }
private:
    float Q_[6];
    float Rv_[2];
    float Rp_[2];
    float x_[6];
    float P_[36];
};

// ============================================================
// OptFlow — 双光流传感器处理主类（Port 化接口）
// ============================================================
class OptFlow {
public:
    // 光流传感器原始数据（IMU数据通过Port读取，不再在此结构体）
    struct Data_t {
        float        left_x;
        float        left_y;
        float        right_x;
        float        right_y;
        unsigned int tick_ms;
        unsigned int valid_mask;
        unsigned int left_tick_ms;
        unsigned int right_tick_ms;
    };

    struct State_t {
        float pll_l_vx, pll_l_vy, pll_r_vx, pll_r_vy;
        float body_vx, body_vy;   // mm/s
        float omega_z;            // rad/s

        // Accel bias from flow+IMU stationary detection
        float imu_bias_ax;   // m/s²
        float imu_bias_ay;   // m/s²

        bool  is_stationary;
        float raw_dtheta;
        float flow_px, flow_py, flow_yaw;

        unsigned int time_ms;
        unsigned int last_time_ms;
        float        dt_s;

        float kf_vx, kf_vy, kf_px, kf_py;
    };

    OptFlow();
    ~OptFlow() = default;

    void process(const Data_t& sensor_data);
    const State_t& get_state() const { return state_; }
    void reset();

    // IMU data inputs (connect to IMU OutputPorts)
    InputPort<float>* imu_acc_x_input()  { return &imu_acc_x_input_; }
    InputPort<float>* imu_acc_y_input()  { return &imu_acc_y_input_; }
    InputPort<float>* imu_acc_z_input()  { return &imu_acc_z_input_; }
    InputPort<float>* imu_gyro_x_input() { return &imu_gyro_x_input_; }
    InputPort<float>* imu_gyro_y_input() { return &imu_gyro_y_input_; }
    InputPort<float>* imu_gyro_z_input() { return &imu_gyro_z_input_; }

    // Velocity outputs (consume via any())
    OutputPort<float>* kf_vx_output()   { return &kf_vx_port_; }
    OutputPort<float>* kf_vy_output()   { return &kf_vy_port_; }
    OutputPort<float>* body_vx_output() { return &body_vx_port_; }
    OutputPort<float>* body_vy_output() { return &body_vy_port_; }
    OutputPort<float>* omega_z_output() { return &omega_z_port_; }

private:
    static constexpr float HALF_BASELINE = OPTFLOW_HALF_BASELINE_MM;
    static constexpr float MIN_DT = 0.001f;
    static constexpr float MAX_DT = 0.1f;
    static constexpr float kQPos = 0.1f;
    static constexpr float kQBiasAx = 0.01f;
    static constexpr float kQBiasAy = 0.01f;
    static constexpr float kRPos = 1e6f;

    State_t      state_;

    PositionPLL  pll_lx_, pll_ly_, pll_rx_, pll_ry_;
    FlowStationaryDetector flow_stat_;

    float        left_last_x_, left_last_y_;
    float        right_last_x_, right_last_y_;
    unsigned int last_time_ms_;
    bool         left_initialized_, right_initialized_;

    Kalman2DPosVel kf_;
    bool kf_inited_;
    bool prev_stationary_;

    // IMU input ports (connected to IMU OutputPorts)
    InputPort<float> imu_acc_x_input_;
    InputPort<float> imu_acc_y_input_;
    InputPort<float> imu_acc_z_input_;
    InputPort<float> imu_gyro_x_input_;
    InputPort<float> imu_gyro_y_input_;
    InputPort<float> imu_gyro_z_input_;

    // Velocity output ports
    OutputPort<float> kf_vx_port_{0.0f};
    OutputPort<float> kf_vy_port_{0.0f};
    OutputPort<float> body_vx_port_{0.0f};
    OutputPort<float> body_vy_port_{0.0f};
    OutputPort<float> omega_z_port_{0.0f};
};

#endif // __OPT_FLOW_HPP
