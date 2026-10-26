# AGENTS.md

This file provides guidance to Codex (Codex.ai/code) when working with code in this repository.

## Project overview

FreeRTOS-based robot soccer control firmware running on STM32F405 (Cortex-M4F). Controls a 4-wheel omni-directional robot with DMH3510 hub motors over CAN bus, an ICM42688 IMU over SPI, and a dribble motor. Communicates with an off-board host (CM4/raspi) through the fixed-size SPI motion protocol.

## Build commands

Requires `arm-none-eabi-gcc` toolchain and CMake >= 3.22 with Ninja.

```bash
# Configure (from Firmware/ directory)
cmake --preset Debug -B build/Debug

# Build
cmake --build build/Debug

# Flash via J-Link (open Firmware/stm32_cm4_rtos.jdebug in J-Link debugger)
```

**After every code change**, run `cmake --build build/Debug` from `Firmware/` and verify zero errors before continuing.

CMakePresets.json defines `Debug` and `Release` presets. Both use the Ninja generator and the bundled toolchain file at `Firmware/cmake/gcc-arm-none-eabi.cmake`.
## High-level architecture

The firmware is built around a component-based data flow pattern (`Firmware/Component/component.hpp`). Components exchange data through typed `InputPort<T>` / `OutputPort<T>` pairs, which handle per-control-iteration freshness tracking.

```
Task/         — FreeRTOS tasks + main entry point
Component/    — Device drivers (motor, IMU, optflow) & control algorithms
Communication/— CAN bus protocol layer
Board/        — STM32CubeMX HAL, FreeRTOS middleware, linker script, startup code
```

**Control loop:** `ctrl_task.cpp` samples fresh wheel feedback and queued raw
IMU/optical-flow observations, steps the shared `MotionEngine`, then applies its
safety state before all wheel, dribbler, and kick outputs. The shared kernel in
`../../motion/` owns fixed-lag EKF/current prediction, independent XY/yaw
trajectories, feedforward and wheel-speed PI. There is one control architecture;
wheel drivers encode pure torque with MIT Kp=Kd=0.

**Key design decisions:**
- `Component/control_params.hpp` contains board physical constants and dribbler/driver settings; shared motion configuration owns estimator/controller limits.
- IMU driver publishes raw SI samples only; attitude and bias belong to the shared estimator.
- Optional optical flow produces independent, timestamped body velocity observations.
- Wheel velocity uses a position PLL and real CAN feedback timestamps; stale feedback faults the controller.
- FreeRTOS heap is placed in CCMRAM in `Task/main.cpp`.
- RTTI, exceptions, and thread-safe statics are disabled.
- CAN TX must run from `ctrl_task.cpp` under `sem_can_txHandle`. Components build messages for that task to send.

## Naming conventions (per Docs/README.md)

- **Classes/Types:** CamelCase (e.g., `MotionEngine`, `MotorDMH3510`)
- **Variables/Functions:** snake_case (e.g., `motor_speed`, `calculate_pid_output()`)

## CubeMX regeneration hazard

When regenerating code with STM32CubeMX, `Firmware/Board/Core/Src/main.c` may auto-generate `HAL_TIM_PeriodElapsedCallback`, causing a duplicate-definition linker error. Manually comment it out in `main.c` after regeneration.
