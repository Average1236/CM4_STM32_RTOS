#ifndef __CHASSIS_CONTROLLER_HPP
#define __CHASSIS_CONTROLLER_HPP

#include "component.hpp"
#include "Task/utils.hpp"
#include <array>
#include <cstddef>

class ChassisController {
public:
    ChassisController();

    InputPort<float>* chassis_vx_input_port() { return &chassis_vx_input_port_; }
    InputPort<float>* chassis_vy_input_port() { return &chassis_vy_input_port_; }
    InputPort<float>* chassis_omega_z_input_port() { return &chassis_omega_z_input_port_; }
    InputPort<float>* chassis_yaw_input_port() { return &chassis_yaw_input_port_; }

    OutputPort<float>* wheel_torque_ff_output_port(std::size_t index) {
        return (index < wheel_torque_ff_output_ports_.size()) ? &wheel_torque_ff_output_ports_[index] : nullptr;
    }

    InputPort<float>* wheel_sent_torque_input_port(std::size_t index) {
        return (index < wheel_sent_torque_input_ports_.size()) ? &wheel_sent_torque_input_ports_[index] : nullptr;
    }

    void set_reference(const float vel_ref[3], const float acc_ref[3]);
    void set_use_imu_yaw(bool enable);
    void set_yaw_target(float target_pos, float target_vel);
    void set_yaw_angle_target(float target_filt, float yaw_max_vel);
    void set_vxvy_acc_limits(float acc_x, float acc_y);
    void set_velocity_pid_gains(const float pid_x[3], const float pid_y[3]);
    float omega_ref() const { return omega_ref_; }
    float f_task(std::size_t index) const { return (index < 3) ? f_task_[index] : 0.0f; }
    void step(float dt_s);
    void reset();

private:
    bool inverse3x3(const float in[3][3], float out[3][3]) const;
    void precompute_mappings();

    InputPort<float> chassis_vx_input_port_;
    InputPort<float> chassis_vy_input_port_;
    InputPort<float> chassis_omega_z_input_port_;
    InputPort<float> chassis_yaw_input_port_;

    std::array<OutputPort<float>, 4> wheel_torque_ff_output_ports_ = {
        OutputPort<float>(0.0f),
        OutputPort<float>(0.0f),
        OutputPort<float>(0.0f),
        OutputPort<float>(0.0f),
    };

    float vel_ref_[3] = {0.0f, 0.0f, 0.0f};
    float acc_ref_[3] = {0.0f, 0.0f, 0.0f};

    PID vx_pid_;
    PID vy_pid_;
    PID::Parameter_t vx_pid_param_;
    PID::Parameter_t vy_pid_param_;

    // 2nd-order LESO states: [z1=omega_z, z2=disturbance] for yaw
    float yaw_leso_[2] = {0.0f, 0.0f};

    float yaw_target_pos_ = 0.0f;
    float yaw_target_vel_ = 0.0f;
    bool use_imu_ = false;

    // Angle PID state (outer loop → ω_ref)
    float yaw_angle_pid_integ_ = 0.0f;
    float yaw_angle_target_ = 0.0f;
    float yaw_angle_pid_max_vel_ = 25.0f;

    // Runtime acc limits from SPI (clamp vx/vy PID output)
    float vx_acc_limit_ = 7.0f;
    float vy_acc_limit_ = 7.0f;

    float j1_[3][4] = {{0.0f}};
    float j1_pinv_[4][3] = {{0.0f}};

    std::array<InputPort<float>, 4> wheel_sent_torque_input_ports_ = {};

    float last_chassis_vx_m_s_ = 0.0f;
    float last_chassis_vy_m_s_ = 0.0f;
    float last_chassis_omega_z_rad_s_ = 0.0f;
    float last_chassis_yaw_rad_ = 0.0f;

    ButterworthLowPass2 omega_z_filter_;
    ButterworthLowPass2 yaw_angle_diff_filter_;  // separate D-term LPF for angle PID
    ButterworthLowPass2 acc_ff_x_filter_;
    ButterworthLowPass2 acc_ff_y_filter_;
    float omega_ref_ = 0.0f;
    float f_task_[3] = {0.0f, 0.0f, 0.0f};
};

#endif // __CHASSIS_CONTROLLER_HPP
