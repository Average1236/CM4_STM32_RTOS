#include "cmsis_os.h"
#include "freertos_vars.h"
#include "can_callbacks.h"
#include "z_main.h"

extern "C" void StartOptFlowRxTask(void*) {
    DualOptFlowSnapshot_t snapshot;
    for (;;) {
        if (osMessageQueueGet(q_optflow_dataHandle, &snapshot, nullptr, osWaitForever) != osOK) continue;
        OptFlow::Data_t data{snapshot.left_x, snapshot.left_y, snapshot.right_x, snapshot.right_y,
            HAL_GetTick(), snapshot.valid_mask, snapshot.left_tick_ms, snapshot.right_tick_ms};
        if (!opt_flow.process(data)) continue;
        const auto& measured = opt_flow.get_state();
        if (osMutexAcquire(mtx_robot_stateHandle, 10) != osOK) continue;
        const auto now = motion_now_us();
        const auto age = static_cast<std::uint32_t>(HAL_GetTick() - measured.sample_time_ms) * 1000ULL;
        if (age < now) {
            robot.latest_flow_.sample_time_us = now - age;
            robot.latest_flow_.velocity_body_m_s = motion::Vec2(measured.body_vx * .001f, measured.body_vy * .001f);
            ++robot.latest_flow_.sequence;
            robot.latest_flow_.valid = true;
        }
        osMutexRelease(mtx_robot_stateHandle);
    }
}
