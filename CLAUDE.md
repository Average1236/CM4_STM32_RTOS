# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project overview

FreeRTOS-based robot soccer control firmware running on STM32F405 (Cortex-M4F). Controls a 4-wheel omni-directional robot with DMH3510 hub motors over CAN bus, an ICM42688 IMU over SPI, and a dribble motor. Communicates with an off-board host (CM4/raspi) via SPI and UART.

## Build commands

Requires `arm-none-eabi-gcc` toolchain and CMake >= 3.22 with Ninja.

```bash
# Configure (from Firmware/ directory)
cmake --preset Debug -B build/Debug

# Build
cmake --build build/Debug

# Flash via J-Link (open Firmware/stm32_cm4_rtos.jdebug in J-Link debugger)
```

CMakePresets.json defines `Debug` and `Release` presets. Both use the Ninja generator and the bundled toolchain file at `Firmware/cmake/gcc-arm-none-eabi.cmake`.

## High-level architecture

The firmware is built around a component-based data flow pattern (`Firmware/Component/component.hpp`). Components exchange data through typed `InputPort<T>` / `OutputPort<T>` pairs, which handle per-control-iteration freshness tracking.

```
Task/         — FreeRTOS tasks + main entry point
Component/    — Device drivers (motor, IMU, optflow) & control algorithms
Communication/— CAN bus protocol layer
Interface/    — Base interface classes
Board/        — STM32CubeMX HAL, FreeRTOS middleware, linker script, startup code
```

**Control loop:** `ctrl_task.cpp` runs a fixed-frequency (TIM2-driven) loop that:
1. Reads SPI commands from the off-board host (target robot velocity)
2. Runs `Robot::motion_planner()` — jerk-limited trapezoidal acceleration planning per axis (vx, vy, ωz)
3. `ChassisEstimator::step()` fuses wheel velocities + IMU yaw into chassis velocity estimates (vx, vy, ωz)
4. `MixedLesoChassisController::step()` — 2nd-order LESO per axis estimates total disturbance, computes torque feedforward via pseudo-inverse Jacobian
5. Sends MIT-mode commands (kd, vel_ref, torque_ff) to each wheel motor over CAN

**Key design decisions:**
- All tunable control parameters are `constexpr` in `Firmware/Component/control_params.hpp` — edit here to tune
- Wheel motors operate in MIT control mode (`kModeMITControl`): position control loop is disabled (kp=0), only velocity damping (kd) + torque feedforward is used
- DMH3510 motor velocity is estimated by a PLL (`MotorDMH3510` in `wheel_motor.hpp`) from position feedback, not direct velocity readings
- Yaw estimation in `ChassisEstimator` uses a PLL on IMU yaw, with configurable source: IMU gyro direct or PLL output (`control_config::kChassisOmegaZSource`)
- FreeRTOS heap is placed in CCMRAM for performance (`main.cpp`)
- RTTI, exceptions, and thread-safe statics are disabled (`-fno-rtti -fno-exceptions -fno-threadsafe-statics`)

**RTOS synchronization primitives** (declared in `Firmware/freertos_vars.h`):
- Message queues: `q_can1_rxHandle`, `q_motor_fbHandle`, `q_imu_dataHandle`, `q_optflow_dataHandle`
- Semaphores: `sem_ctrl_triggerHandle` (control loop trigger), `sem_spi_triggerHandle`, `sem_imu_readyHandle`, `sem_can_txHandle`
- Mutexes: `mtx_robot_stateHandle`, `mtx_spi_bufferHandle`

## Naming conventions (per Docs/README.md)

- **Classes/Types:** CamelCase (e.g., `MotorController`, `ChassisEstimator`)
- **Variables/Functions:** snake_case (e.g., `motor_speed`, `calculate_pid_output()`)

## CubeMX regeneration hazard

When regenerating code with STM32CubeMX, `Firmware/Board/Core/Src/main.c` may auto-generate `HAL_TIM_PeriodElapsedCallback`, causing a duplicate-definition linker error. Manually comment it out in `main.c` after regeneration.
