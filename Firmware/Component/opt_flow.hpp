#ifndef __OPT_FLOW_HPP
#define __OPT_FLOW_HPP

#include "Task/utils.hpp"

#include <cstdint>
#include <cstddef>

#define OPTFLOW_OFFSET_X (115.0f)
#define OPTFLOW_OFFSET_Y (10.5f)

class OptFlow {
public:
    struct Data_t {
        float x;
        float y;
    };
    
    struct State_t {
        float raw_x;
        float raw_y;
        float last_x;
        float last_y;
        float delta_x;
        float delta_y;
        float global_x;
        float global_y;
        float raw_vx;
        float raw_vy;
        float raw_omega;
        uint32_t time_ms;
        uint32_t last_time_ms;
        uint32_t time_us;
        uint32_t last_time_us;
        float dt_s;
        float delta_yaw;
        float angle;
        float e;
        float f;
        float all_distance;
    };
    
    OptFlow();
    ~OptFlow() = default;
    
    void process(const Data_t& sensor_data, float imu_omega_z, float imu_angle_z);
    const State_t& get_state() const { return state_; }
    void reset();
    
private:
    static constexpr float OFFSET_X = 115.0f;
    static constexpr float OFFSET_Y = 10.5f;
    static constexpr float MIN_DT = 0.001f;
    static constexpr float MAX_DT = 0.1f;
    
    State_t state_;
    bool initialized_;
};

#endif // __OPT_FLOW_HPP
