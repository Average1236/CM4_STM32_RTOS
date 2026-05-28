#ifndef __CHASSIS_ESTIMATOR_HPP
#define __CHASSIS_ESTIMATOR_HPP

#include "component.hpp"
#include "Task/utils.hpp"
#include <array>
#include <cstddef>

class ChassisEstimator {
public:
    ChassisEstimator();

    InputPort<float>* wheel_velocity_input_port(std::size_t index) {
        return (index < wheel_velocity_input_ports_.size()) ? &wheel_velocity_input_ports_[index] : nullptr;
    }

    InputPort<float>* imu_yaw_input_port() { return &imu_yaw_input_port_; }
    InputPort<float>* imu_omega_z_input_port() { return &imu_omega_z_input_port_; }

    OutputPort<float>* chassis_vx_output_port() { return &chassis_vx_output_port_; }
    OutputPort<float>* chassis_vy_output_port() { return &chassis_vy_output_port_; }
    OutputPort<float>* chassis_yaw_output_port() { return &chassis_yaw_output_port_; }
    OutputPort<float>* chassis_integrated_yaw_output_port() { return &chassis_integrated_yaw_output_port_; }
    OutputPort<float>* chassis_omega_z_output_port() { return &chassis_omega_z_output_port_; }

    void step(float dt_s);
    void reset();

private:
    bool inverse3x3(const float in[3][3], float out[3][3]) const;
    void precompute_mappings();

    std::array<InputPort<float>, 4> wheel_velocity_input_ports_;
    InputPort<float> imu_yaw_input_port_;
    InputPort<float> imu_omega_z_input_port_;

    OutputPort<float> chassis_vx_output_port_{0.0f};
    OutputPort<float> chassis_vy_output_port_{0.0f};
    OutputPort<float> chassis_yaw_output_port_{0.0f};
    OutputPort<float> chassis_integrated_yaw_output_port_{0.0f};
    OutputPort<float> chassis_omega_z_output_port_{0.0f};

    float j2_pinv_[3][4] = {{0.0f}};

    float last_wheel_vel_rpm_[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    bool has_last_raw_yaw_rad_ = false;
    float last_raw_yaw_rad_ = 0.0f;
    float accumulated_yaw_rad_ = 0.0f;
    float integrated_yaw_rad_ = 0.0f;
    float last_yaw_rad_ = 0.0f;
    float last_omega_z_rad_s_ = 0.0f;

    float yaw_pll_kp_ = 0.0f;
    float yaw_pll_ki_ = 0.0f;
    float yaw_pll_pos_est_rad_ = 0.0f;
    float yaw_pll_vel_est_rad_s_ = 0.0f;
    float yaw_omega_ramp_alpha_ = 0.0f;
    bool yaw_pll_initialized_ = false;

    ButterworthLowPass2 imu_omega_filter_{{0.0f, 0.0f}};
};

#endif // __CHASSIS_ESTIMATOR_HPP
