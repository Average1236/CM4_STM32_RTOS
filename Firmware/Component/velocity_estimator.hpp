#ifndef __VELOCITY_ESTIMATOR_HPP
#define __VELOCITY_ESTIMATOR_HPP

#include "component.hpp"
#include <array>
#include <cstddef>
#include <cstdint>

class VelocityEstimator : public ComponentBase {
public:
    VelocityEstimator();

    InputPort<float>* wheel_velocity_input_port(std::size_t index) {
        return (index < wheel_velocity_input_ports_.size()) ? &wheel_velocity_input_ports_[index] : nullptr;
    }

    InputPort<float>* imu_acc_x_input_port() { return &imu_acc_x_input_port_; }
    InputPort<float>* imu_acc_y_input_port() { return &imu_acc_y_input_port_; }
    InputPort<float>* imu_yaw_input_port() { return &imu_yaw_input_port_; }
    InputPort<float>* imu_omega_z_input_port() { return &imu_omega_z_input_port_; }

    OutputPort<float>* vx_output_port() { return &vx_output_port_; }
    OutputPort<float>* vy_output_port() { return &vy_output_port_; }
    OutputPort<float>* omega_z_output_port() { return &omega_z_output_port_; }
    OutputPort<float>* speed_output_port() { return &speed_output_port_; }

    void update(uint32_t timestamp) override;
    void step(float dt_s);
    void reset();
    void reset_ports();

private:
    bool inverse3x3(const float in[3][3], float out[3][3]) const;
    void precompute_mappings();
    void compute_wheel_velocity_measurement(float chassis_vel_meas[3]);
    bool stationary_detected(float vx_meas, float vy_meas, float omega_z_rad_s, float ax, float ay) const;

    std::array<InputPort<float>, 4> wheel_velocity_input_ports_;
    InputPort<float> imu_acc_x_input_port_;
    InputPort<float> imu_acc_y_input_port_;
    InputPort<float> imu_yaw_input_port_;
    InputPort<float> imu_omega_z_input_port_;

    OutputPort<float> vx_output_port_{0.0f};
    OutputPort<float> vy_output_port_{0.0f};
    OutputPort<float> omega_z_output_port_{0.0f};
    OutputPort<float> speed_output_port_{0.0f};

    float j2_pinv_[3][4] = {{0.0f}};

    float vel_est_[2] = {0.0f, 0.0f};
    float acc_bias_[2] = {0.0f, 0.0f};

    float last_wheel_vel_rpm_[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float last_acc_mps2_[2] = {0.0f, 0.0f};
    float last_yaw_deg_ = 0.0f;
    float last_omega_z_deg_s_ = 0.0f;

    uint16_t stationary_counter_ = 0;
};

#endif // __VELOCITY_ESTIMATOR_HPP
