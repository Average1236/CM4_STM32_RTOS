#include "opt_flow.hpp"
#include <cmath>
#include <cstring>
#include "stm32f4xx_hal.h"

OptFlow::OptFlow() : initialized_(false) {
    memset(&state_, 0, sizeof(state_));
}

void OptFlow::reset() {
    memset(&state_, 0, sizeof(state_));
    initialized_ = false;
}

float cosdt_debug=0;
float sindt_debug=0;

float dy_rot_debug=0;

void OptFlow::process(const Data_t& sensor_data, float imu_omega_z, float imu_angle_z) {
    // Update timestamp
    state_.time_ms = HAL_GetTick();
    state_.time_us = TIM13->CNT;
    
    // Store raw data (axes swapped in hardware)
    state_.raw_x = sensor_data.y;
    state_.raw_y = -sensor_data.x;
    
    // First sample guard
    if (!initialized_) {
        state_.last_x = state_.raw_x;
        state_.last_y = state_.raw_y;
        state_.last_time_ms = state_.time_ms;
        state_.last_time_us = state_.time_us;
        initialized_ = true;
        return;
    }
    
    // Compute delta
    state_.delta_x = state_.raw_x - state_.last_x;
    state_.delta_y = state_.raw_y - state_.last_y;
    state_.all_distance += sqrtf(state_.delta_x * state_.delta_x + state_.delta_y * state_.delta_y);
    
    // Compute dt in seconds
    // uint32_t t0 = state_.last_time_ms;
    // uint32_t t1 = state_.time_ms;
    // uint32_t dt_ms = (t1 >= t0) ? (t1 - t0) : (t1 + (0xFFFFFFFFu - t0) + 1u);
    // float dt_s = static_cast<float>(dt_ms) / 1000.0f;

    uint32_t t0 = state_.last_time_us;
    uint32_t t1 = state_.time_us;
    uint32_t dt_us = (t1 >= t0) ? (t1 - t0) : (t1 + ((1<<16) - t0) + 1u);
    float dt_s = static_cast<float>(dt_us) / 1000000.0f;
    
    // Clamp dt
    if (dt_s < MIN_DT) dt_s = MIN_DT;
    if (dt_s > MAX_DT) dt_s = MAX_DT;
    state_.dt_s = dt_s;
    
    // Get IMU yaw rate for rigid-body correction
    state_.delta_yaw = dt_s * imu_omega_z / 180.0f * PI;
    
    // Rigid-body correction
    float cosdt = cosf(state_.delta_yaw);
    float sindt = sinf(state_.delta_yaw);
    float dx_rot = (cosdt - 1.0f) * OFFSET_X - sindt * OFFSET_Y;
    float dy_rot = sindt * OFFSET_X + (cosdt - 1.0f) * OFFSET_Y;

    sindt_debug = sindt;
    cosdt_debug = cosdt;

    dy_rot_debug = dy_rot;

    state_.e = state_.delta_x - dx_rot;
    state_.f = state_.delta_y - dy_rot;
    
    // Compute velocities
    state_.raw_vx = state_.e / dt_s;
    state_.raw_vy = state_.f / dt_s;
    state_.raw_omega = state_.delta_yaw / dt_s;
    
    state_.angle = atan2f(state_.e, state_.f);
    
    // Update global position
    state_.global_x += state_.e;
    state_.global_y += state_.f;
    
    float yaw_now_rad = (imu_angle_z - (-108.67f)) / 180.0f * PI;
    float yaw_mid_rad = yaw_now_rad + 0.5f * state_.delta_yaw;
    state_.global_x += state_.e * cosf(yaw_mid_rad) + state_.f * sinf(yaw_mid_rad);
    state_.global_y += -state_.e * sinf(yaw_mid_rad) + state_.f * cosf(yaw_mid_rad);
    
    state_.last_x = state_.raw_x;
    state_.last_y = state_.raw_y;
    state_.last_time_ms = state_.time_ms;
    state_.last_time_us = state_.time_us;
}
