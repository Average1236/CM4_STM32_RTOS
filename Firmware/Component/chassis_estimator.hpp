#ifndef __CHASSIS_ESTIMATOR_HPP
#define __CHASSIS_ESTIMATOR_HPP

#include "component.hpp"
#include "Task/utils.hpp"
#include <array>
#include <cstddef>

// 1-state Kalman for 2-source velocity fusion (wheel + optflow)
struct FusionKalman1D {
    float v_est = 0.0f;
    float P = 1.0f;

    void predict(float Q) {
        P += Q;
    }

    void update(float meas, float R) {
        const float K = P / (P + R);
        v_est += K * (meas - v_est);
        P = (1.0f - K) * P;
    }
};

class ChassisEstimator {
public:
    ChassisEstimator();

    InputPort<float>* wheel_velocity_input_port(std::size_t index) {
        return (index < wheel_velocity_input_ports_.size()) ? &wheel_velocity_input_ports_[index] : nullptr;
    }

    InputPort<float>* imu_yaw_input_port() { return &imu_yaw_input_port_; }
    InputPort<float>* imu_omega_z_input_port() { return &imu_omega_z_input_port_; }
    InputPort<float>* optflow_vx_input_port() { return &optflow_vx_input_port_; }
    InputPort<float>* optflow_vy_input_port() { return &optflow_vy_input_port_; }
    InputPort<float>* imu_gyro_x_input_port() { return &imu_gyro_x_input_port_; }
    InputPort<float>* imu_gyro_y_input_port() { return &imu_gyro_y_input_port_; }

    OutputPort<float>* chassis_vx_output_port() { return &chassis_vx_output_port_; }
    OutputPort<float>* chassis_vy_output_port() { return &chassis_vy_output_port_; }
    OutputPort<float>* chassis_yaw_output_port() { return &chassis_yaw_output_port_; }
    OutputPort<float>* chassis_omega_z_output_port() { return &chassis_omega_z_output_port_; }
    OutputPort<float>* wheel_chassis_vx_output_port() { return &wheel_chassis_vx_output_port_; }
    OutputPort<float>* wheel_chassis_vy_output_port() { return &wheel_chassis_vy_output_port_; }
    OutputPort<float>* fused_chassis_vx_output_port() { return &fused_chassis_vx_output_port_; }
    OutputPort<float>* fused_chassis_vy_output_port() { return &fused_chassis_vy_output_port_; }

    void set_reference_accel(const float acc_ref[3]) {
        acc_ref_m_s2_[0] = acc_ref[0];
        acc_ref_m_s2_[1] = acc_ref[1];
    }

    void step(float dt_s);
    void reset();

private:
    bool inverse3x3(const float in[3][3], float out[3][3]) const;
    void precompute_mappings();

    std::array<InputPort<float>, 4> wheel_velocity_input_ports_;
    InputPort<float> imu_yaw_input_port_;
    InputPort<float> imu_omega_z_input_port_;
    InputPort<float> optflow_vx_input_port_;
    InputPort<float> optflow_vy_input_port_;
    InputPort<float> imu_gyro_x_input_port_;
    InputPort<float> imu_gyro_y_input_port_;

    FusionKalman1D kf_vx_;
    FusionKalman1D kf_vy_;

    OutputPort<float> chassis_vx_output_port_{0.0f};
    OutputPort<float> chassis_vy_output_port_{0.0f};
    OutputPort<float> chassis_yaw_output_port_{0.0f};
    OutputPort<float> chassis_omega_z_output_port_{0.0f};
    OutputPort<float> wheel_chassis_vx_output_port_{0.0f};
    OutputPort<float> wheel_chassis_vy_output_port_{0.0f};
    OutputPort<float> fused_chassis_vx_output_port_{0.0f};
    OutputPort<float> fused_chassis_vy_output_port_{0.0f};

    float j2_pinv_[3][4] = {{0.0f}};
    float acc_ref_m_s2_[2] = {0.0f, 0.0f};

    float last_wheel_vel_rpm_[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    bool has_last_raw_yaw_rad_ = false;
    float last_raw_yaw_rad_ = 0.0f;
    float accumulated_yaw_rad_ = 0.0f;
    float last_yaw_rad_ = 0.0f;
    float last_omega_z_rad_s_ = 0.0f;

};

#endif // __CHASSIS_ESTIMATOR_HPP
