# CLAUDE.md

This file provides guidance to Codex (Codex.ai/code) when working with code in this repository.

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

**After every code change**, run `cmake --build build/Debug` from `Firmware/` and verify zero errors before continuing.

CMakePresets.json defines `Debug` and `Release` presets. Both use the Ninja generator and the bundled toolchain file at `Firmware/cmake/gcc-arm-none-eabi.cmake`.

## Raspberry Pi side workflow

The Raspberry Pi project may be present locally at `rpi_timer/robot/` and is deployed to `/home/pi/rpi_timer/robot/` on the Pi (`pi@192.168.31.245`, password`123`). Use `paramiko` from the base Python environment for password-based SFTP/SSH when key auth is unavailable; do not commit credentials.

Before making any local changes under `rpi_timer/robot/`, copy/sync the current remote project from the Pi first so the local and deployed Raspberry Pi code stay in sync.

When changing Raspberry Pi protobuf files:

```bash
# On the Pi, regenerate generated protobuf code after editing share/proto/*.proto
cd /home/pi/rpi_timer/robot/share/proto
protoc --cpp_out=. zss_cmd.proto

# Then rebuild the Pi project
cd /home/pi/rpi_timer/robot
cmake --build build
```

After regenerating protobuf code on the Pi, sync the generated `share/proto/*.pb.h` and `share/proto/*.pb.cc` files back to the local `rpi_timer/robot/share/proto/` copy so local code matches the deployed code. Prefer normal generated protobuf accessors (for example `raw_vision_vel_x()`) over parsing `UnknownFieldSet` manually.

## High-level architecture

The firmware is built around a component-based data flow pattern ([component.hpp](Firmware/Component/component.hpp)). Components exchange data through typed `InputPort<T>` / `OutputPort<T>` pairs, which handle per-control-iteration freshness tracking.

```
Task/         — FreeRTOS tasks + main entry point
Component/    — Device drivers (motor, IMU, optflow) & control algorithms
Communication/— CAN bus protocol layer
Interface/    — Base interface classes
Board/        — STM32CubeMX HAL, FreeRTOS middleware, linker script, startup code
```

### Task structure

7 FreeRTOS tasks run at different rates, synchronized via semaphores, queues, and mutexes:

| Task | Entry | Trigger | Frequency | Role |
|------|-------|---------|-----------|------|
| IMU RX | [`imu_task.cpp`](Firmware/Task/imu_task.cpp) | `sem_imu_readyHandle` (TIM7 ISR) | 800 Hz | Read ICM42688, compute yaw/roll/pitch, gyro bias estimation |
| SPI Exchange | [`spi_task.cpp`](Firmware/Task/spi_task.cpp) | `sem_spi_triggerHandle` (SPI DMA IRQ) | ~500 Hz | Full-duplex SPI with CM4, decode commands / encode telemetry |
| Motor RX | [`motor_task.cpp`](Firmware/Task/motor_task.cpp) | `q_motor_fbHandle` (queue) | Per motor msg | Parse DMH3510 CAN feedback — run PLL + Luenberger observer |
| **CTRL** | [`ctrl_task.cpp`](Firmware/Task/ctrl_task.cpp) | `sem_ctrl_triggerHandle` (TIM2 ISR) | **500 Hz (2ms)** | **Main control loop** — motion planning, IK, LESO, PID, CAN TX |
| OptFlow RX | [`optflow_task.cpp`](Firmware/Task/optflow_task.cpp) | `q_optflow_dataHandle` (queue) | Sensor rate | Process dual optical flow + IMU fusion (6-state Kalman) |
| Health | [`health_task.cpp`](Firmware/Task/health_task.cpp) | `osDelay(100)` | 10 Hz | Watchdog feed, ADC battery/cap voltage read |
| Telemetry | [`telemetry_task.cpp`](Firmware/Task/telemetry_task.cpp) | `osDelay(100)` | 10 Hz | Debug CSV output, timestamp monitoring |

### Control loop data flow (per iteration, 500 Hz)

```
SPI DMA Complete (IRQ)
  → SPI Task
      robot.pi_decode_spi()     // Decode CM4 → STM32 96-byte SPI frame
      robot.pi_encode_spi()     // Encode STM32 → CM4 telemetry
      robot.watchdog_feed()     // Reset watchdog counter (default 800 frames ≈ 1.6s timeout)

TIM2 @ 500 Hz (IRQ) → sem_ctrl_triggerHandle

CTRL Task (acquires mtx_robot_stateHandle)
  │
  ├─ imu.reset_ports() + motor.reset_ports()  // Clear stale OutputPort ages
  │
  ├─ Dribbler: infra-red voltage asymmetric LPF (instant rise, α=0.1 fall)
  │
  ├─ Dribbler: hybrid state machine
  │     Torque phase (-0.05 Nm ball capture)
  │       → 20 consecutive ball-hold frames → Speed phase (-100 rps, 0.15 Nm torque limit)
  │       → 10 consecutive lost-ball frames → back to Torque phase
  │
  ├─ Kick trigger (infra-red threshold or manual test mode)
  │
  ├─ robot.motion_planner(dt_us)
  │     Vx/Vy: Butterworth 2nd-order LPF (fc=10 Hz) on SPI velocity targets
  │     Yaw: TD-based planner fallback (non-IMU mode)
  │
  ├─ robot.prepare_yaw_control(dt_s)          // Active when kUseWheelSpeedPidFallback=true
  │     Circular LPF on wrapped yaw target → Angle PID (Kp=10,Ki=2,Kd=0.3) → ω_ref
  │
  ├─ robot.ik_solve()
  │     Inverse kinematics: [vx, vy, ωz] → 4 wheel velocity targets (RPM)
  │     Jacobian from wheel geometry: front ±30°, rear ±45°, r=33mm, d=78.5mm
  │
  ├─ robot.update_torque_feedforward(dt_us)
  │   │
  │   ├─ chassis_estimator.step(dt_s)
  │   │   Wheel Jacobian pinv → chassis vx/vy (×0.8, ×0.5 scale, rate-limited 10 m/s²)
  │   │   Adaptive 1-state Kalman fusion (wheel + optflow, 5 source modes)
  │   │   IMU yaw unwrapping across ±π + ωz
  │   │   Output: chassis_vx/vy, chassis_yaw, chassis_ωz
  │   │
  │   └─ chassis_controller.step(dt_s)
  │       Vx PID (Kp=30,Ki=300) + Vy PID → Fx, Fy (mass-scaled ×4.0kg)
  │       Yaw 2nd-order LESO [ωz, disturbance]:
  │         Outer: Angle PID (Kp=20,Ki=3) on wrapped yaw error → ω_ref
  │         Inner: Rate P (ωc=75 rad/s) + disturbance rejection → F_ψ
  │         Observer bandwidth ωo=250 rad/s (fixed, min==max)
  │       Jacobian pinv: [Fx, Fy, F_ψ] → 4 wheel torque feedforward values
  │
  └─ Build & send CAN messages (under sem_can_txHandle)
      Per wheel: MIT mode 8-byte frame (pos, vel, kp=0, kd, τ_ff)
        Final torque = τ_ff + τ_wheel_speed_PID + τ_virtual_damping
      Dribbler: torque or velocity CAN per ZFOC protocol
      Round-robin wheel sends, priority within 400 μs CAN TX budget
```

### Component details

#### Robot ([robot.hpp](Firmware/Component/robot.hpp), [robot.cpp](Firmware/Component/robot.cpp))

Top-level orchestrator owning all sub-components. Manages SPI protocol, motion planning, IK, kicker timing, and watchdog.

**SPI protocol** (96 bytes/frame, ~500 Hz full-duplex DMA SPI1):
- **CM4 → STM32** (32 bytes packed): velocity targets (vx,vy,ωz), dribbler mode/power, kick type+discharge time, per-axis motion limits (max_acc, max_jerk, max_dec), raw vision velocity (for fusion), vision source selector
- **STM32 → CM4** (62 bytes packed): IMU data (acc/gyro/attitude), wheel velocities (RPM), odometry vx/vy, multiple velocity estimates (optflow, wheel, fused), dribbler ball status, battery/cap voltage, controller internal state (Fx, Fy, Fψ, ω_ref, planned velocities), per-axis max velocity (computed from limits)

**Port connections** (wired in constructor):
```
wheel_motors[i].velocity_cmd_input_port    ← IK motor_vel[i]
chassis_estimator.wheel_velocity_input[i]  ← wheel_motors[i].velocity_output_port
wheel_motors[i].torque_ff_cmd_input_port   ← chassis_controller.wheel_torque_ff_output[i]
chassis_controller.wheel_sent_torque_input[i] ← wheel_motors[i].torque_cmd_output_port
chassis_controller.chassis_{vx,vy,ωz,yaw}_input ← chassis_estimator.chassis_{vx,vy,ωz,yaw}_output
chassis_estimator.vision_{vx,vy}_input     ← SPI raw_vision_vel
```

#### ChassisEstimator ([chassis_estimator.hpp](Firmware/Component/chassis_estimator.hpp))

Multi-source velocity estimation and sensor fusion.

- **Wheel-based velocity**: Jacobian pseudo-inverse (4 wheel ω → chassis vx/vy), scaled vx×0.8, vy×0.5, rate-limited to 10 m/s²
- **IMU yaw**: Unwraps across ±π boundary by tracking cumulative angular displacement. ωz source configurable: gyro direct or PLL output (`kChassisOmegaZSource`)
- **Velocity source** (5 modes, `kChassisVelocitySource`):
  - `0`: Wheel-only | `1`: OptFlow-only | **`2` (default)**: Adaptive 1-state Kalman (wheel + optflow) | `3`: OptFlow + vision | `4`: Wheel + vision
- **Adaptive Kalman** (mode 2): Per-axis 1-state KF. R_wheel is velocity-scheduled (low R at high speed), penalized by tilt rate (gyro x/y), commanded acceleration, and wheel-optflow residual divergence (slip detection)

#### ChassisController ([chassis_controller.hpp](Firmware/Component/chassis_controller.hpp))

Force allocation with active disturbance rejection on yaw.

- **Vx/Vy PID**: `Kp=30, Ki=300, Kd=0, output_limit=10 m/s²`, anti-windup via back-calculation. Output × robot mass (4.0 kg) → Fx, Fy force
- **Yaw LESO** (2nd-order Linear Extended State Observer, yaw axis only):
  - States: `z1 = ωz` (angular velocity estimate), `z2 = total disturbance`
  - Gains: `L1 = 2·ωo`, `L2 = ωo²` where `ωo = 250 rad/s` (fixed — min and max are both 250, no scheduling)
  - Feedforward: actual angular acceleration from motor torques fed back as `u_ψ` into the observer
  - Control law: `F_ψ = I · (ωc·(ω_ref − z1) − z2 + coupling)` where `ωc = 75 rad/s`
- **Outer angle PID** (IMU mode): Wrapped yaw error → ω_ref, Kp=20, Ki=3, Kd=0, derivative-on-measurement
- **Non-IMU fallback**: Pure feedforward + ωz feedback via `kVelFeedbackGainYaw`
- **Torque allocation**: `τ = J₁_pinv · [Fx, Fy, F_ψ]` where J₁_pinv is the precomputed 4×3 force Jacobian pseudo-inverse. Torque clamped to `kWheelTorqueFfLimitNm`

#### MotorDMH3510 / WheelMotorBase ([wheel_motor.hpp](Firmware/Component/wheel_motor.hpp))

Per-wheel motor driver (DMH3510 hub motor) with multi-layer estimation.

- **Velocity PLL** (2nd-order position-locked loop): `kp = 2×75 = 150 rad/s`, `ki = 0.25×kp² = 5625`. Predicts position + corrects from measurement, zero-snap at 0.1 RPM. Velocity ramps up over 0.2s on startup. Gain stability check: if `dt·kp ≥ 1`, reduce kp to `0.95/dt`
- **3-state Luenberger observer** [θ, ω, T_dist]: Observer bandwidth ωo = 100 rad/s. Semi-implicit Euler prediction with torque FF + viscous damping, corrected from single-turn position measurement. T_dist clamped to motor tmax
- **Wheel speed PID** (active when `kUseWheelSpeedPidFallback=true`): Kp=0.06 (ramped over 1.5s), Ki=0.6, output_limit=0.5 Nm
- **Virtual damping** (active when fallback disabled): Speed-scheduled `kd` from 0.027 → 0.005 Nm/(rad/s), knee at 10 rad/s. Suppresses airborne wheel oscillation at low speed while reducing drag at high speed
- **Final torque**: `τ_cmd = τ_ff + τ_pid + τ_damp`, clamped to motor tmax (±0.5 Nm rated)
- **CAN protocol**: MIT control mode (16b position, 12b velocity, 12b kp, 12b kd, 12b torque FF). Register writes via CAN ID 0x7FF (mode, accel/decel, pmax, vmax, tmax, ASR Kp/Ki)
- **Motor recovery**: On detected error, waits 1000ms → sends clear-error CAN → waits 10ms → re-enables. Kd ramped from 0 to nominal over 1.0s on MIT mode entry to avoid current spikes

#### DribblerZfoc ([dribbler_zfoc.hpp](Firmware/Component/dribbler_zfoc.hpp))

Dribbler motor controlled via ZFOC CAN protocol (node_id=5, CAN1).

- **Control modes**: Torque (mode=1) — fixed τ, Velocity (mode=2) — speed with chassis vx compensation, **Hybrid (mode=3, default)** — two-phase state machine
- **Hybrid state machine**: Torque phase (-0.05 Nm capture) ↔ Speed phase (-100 rps with 0.15 Nm torque limit, chassis vx feedforward ×23.0×1.3). Ball detected via infra-red ADC threshold 0.5V
- **Infra-red filter**: Asymmetric LPF — instant rise (ball capture), slow fall α=0.1 (debounce)
- **ZFOC state machine**: Idle (1s confirmation window) → ClosedLoopControl. Re-enters on error

#### IMU ([imu.hpp](Firmware/Component/imu.hpp))

ICM42688 6-axis IMU (SPI2, 1 kHz ODR, ±4G accel, ±1000 dps gyro).

- **Yaw**: Gyro Z integration with dynamic gyro bias correction via 3-phase stationary detection. Optional Butterworth LPF on ωz (disabled: `kImuOmegaButterworthCutoffHz=0.0`)
- **Roll/Pitch**: Complementary filter (α=0.98) — gyro integration fused with accel-derived attitude (atan2). Accel anchor only trusted during stationary phases (`trust_accel` flag)
- **Stationary detection** (3-phase window): Confirm (10 frames all-still, accel variance<0.15, gyro<1.5°/s) → Collect (50-frame window) → Verify (accel norm near 1g±0.45, accel std<0.10, gyro mean<3°/s, gyro std<0.20°/s). Bias updated via EMA (α=0.02, max step 0.05°/s). Requires 2 valid windows before biases are applied. Startup ignore: 2400 frames (3s)

#### OptFlow ([opt_flow.hpp](Firmware/Component/opt_flow.hpp))

Dual PMW3901 optical flow sensor fusion (CAN1 IDs 0x300/0x301).

- **PositionPLL**: 2nd-order locked loop per sensor axis (4 PLLs total), bandwidth 300 Hz. Jump detection: 100mm threshold, 3-frame confirmation
- **Dual-sensor fusion**: Body vx/vy from averaged PLL velocities. ωz from stereo baseline: `Δθ = asin((dx2−dx1) / (2×34mm))`
- **6-state Kalman filter** [px, py, vx, vy, bias_ax, bias_ay]: Predict with IMU acceleration (bias-corrected), update velocity from flow, position from integration. Stationary detection reduces measurement noise R
- **Flow stationary detector**: Independent 3-phase window (confirm → collect → verify), estimates accelerometer bias

### CAN bus topology

**CAN1** (1 Mbps — dribbler + optical flow):

| CAN ID | Direction | Content |
|--------|-----------|---------|
| 0x300 | Left flow sensor → STM32 | float32 x, float32 y (8 bytes) |
| 0x301 | Right flow sensor → STM32 | float32 x, float32 y (8 bytes) |
| 0x0A1 | Dribbler ZFOC → STM32 | Heartbeat: axis_error(32b), state(8b), error_flags(8b), infra_adc(16b) |
| 0x0A6 | STM32 → Dribbler ZFOC | SetRequestedState: int32 state |
| 0x0AA | STM32 → Dribbler ZFOC | SetControllerModes: int32 ctrl_mode, int32 input_mode |
| 0x0AC | STM32 → Dribbler ZFOC | SetInputVelocity: float32 vel, float32 torque_ff |
| 0x0AD | STM32 → Dribbler ZFOC | SetInputTorque: float32 torque, float32 velocity |

**CAN2** (1 Mbps — 4× DMH3510 wheel motors):

| CAN ID | Direction | Content |
|--------|-----------|---------|
| 1–4 | STM32 → DMH3510 | MIT mode command: pos(16b), vel(12b), Kp(12b), Kd(12b), τ_ff(12b) |
| 1–4 | DMH3510 → STM32 | Feedback: state(4b), pos(16b), vel(12b), torque(12b) |
| 0x7FF | STM32 → DMH3510 | Register write: ESC ID, reg address, value (mode/acc/dec/pmax/vmax/tmax/ASR gains) |

**CAN RX routing:**
- CAN1 FIFO0 → optflow callback → `q_optflow_dataHandle` → OptFlow RX task
- CAN1 FIFO0 → dribbler heartbeat callback → direct `robot.dribbler.parse_heartbeat()`
- CAN2 FIFO1 → motor feedback callback → `q_motor_fbHandle` → Motor RX task

### Control algorithms reference

| Algorithm | Location | Purpose |
|-----------|----------|---------|
| Butterworth 2nd-order LPF | `motion_planner()` | Smooth SPI vx/vy targets (fc=10 Hz) |
| Circular LPF | `prepare_yaw_control()` | Smooth wrapped yaw angle target |
| Tracking Differentiator (TD) | `yaw_s_curve.hpp` | Non-IMU yaw planner (deprecated path) |
| Inverse Kinematics (Jacobian pinv) | `ik_solve()` | Robot [vx,vy,ωz] → 4 wheel RPM |
| Position PLL (2nd-order) | `MotorDMH3510` | Estimate wheel velocity from CAN position feedback |
| Luenberger observer (3-state) | `MotorDMH3510` | Estimate θ, ω, T_disturbance per wheel |
| Adaptive 1-state Kalman filter | `ChassisEstimator` | Fuse wheel + optflow chassis velocity (mode 2) |
| 6-state Kalman filter | `OptFlow` | Fuse IMU accel + flow velocity + position |
| Complementary filter (α=0.98) | `IMU` | Fuse gyro + accel for roll/pitch attitude |
| 2nd-order LESO | `ChassisController` | Estimate + reject yaw disturbance (ωz axis only) |
| Angle PID | `ChassisController` | Yaw angle error → ω_ref (outer loop) |
| Velocity PID ×2 | `ChassisController` | Vx/Vy error → force (inner loop) |
| Force Jacobian pinv | `ChassisController` | [Fx, Fy, Fψ] → 4 wheel torques |
| Wheel speed PID | `MotorDMH3510` | Per-wheel velocity fallback (when `kUseWheelSpeedPidFallback`) |
| Virtual damping | `MotorDMH3510` | Speed-scheduled Kd for airborne oscillation suppression |
| Asymmetric LPF | `DribblerZfoc` | Infra-red ball detection (fast rise, slow fall) |
| Hybrid state machine | `DribblerZfoc` | Torque capture → speed hold ball control |

### Key design decisions

- All tunable control parameters are `constexpr` in [`control_params.hpp`](Firmware/Component/control_params.hpp) — edit here to tune
- Wheel motors operate in MIT control mode (`kModeMITControl`): position control loop disabled (kp=0), only velocity damping (kd) + torque feedforward is used
- DMH3510 motor velocity is estimated by a PLL from position feedback, not direct velocity readings (direct velocity is noisy at low RPM)
- Yaw axis only uses LESO for disturbance rejection — Vx/Vy use standard PID, not LESO
- `kUseWheelSpeedPidFallback` (currently `true`) switches between two yaw control paths: (a) per-wheel speed PID (fallback) vs (b) torque-based LESO + Jacobian allocation. When fallback is active, yaw control runs in `prepare_yaw_control()` using wheel speed PID; torque FF limit is set to 0
- **CAN TX rule:** `ZCAN::send_message()` must only be called from [`ctrl_task.cpp`](Firmware/Task/ctrl_task.cpp), and must be preceded by acquiring `sem_can_txHandle`. Components (e.g. `DribblerZfoc`) build `can_Message_t` objects and expose them via output buffers; they must never call `send_message()` directly.
- FreeRTOS heap is placed in CCMRAM for performance
- RTTI, exceptions, and thread-safe statics are disabled (`-fno-rtti -fno-exceptions -fno-threadsafe-statics`)

### RTOS synchronization primitives

(declared in [`freertos_vars.h`](Firmware/freertos_vars.h))

**Semaphores (binary):**
- `sem_ctrl_triggerHandle` — TIM2 ISR → CTRL task (500 Hz control loop trigger)
- `sem_spi_triggerHandle` — SPI1 DMA completion IRQ → SPI task
- `sem_imu_readyHandle` — TIM7 ISR → IMU task (800 Hz IMU read trigger)
- `sem_can_txHandle` — Guards CAN TX access (must be acquired before `ZCAN::send_message()`)

**Mutexes:**
- `mtx_robot_stateHandle` — Guards `Robot` object across tasks (SPI writes, CTRL reads, Health reads)
- `mtx_spi_bufferHandle` — Guards SPI TX/RX buffers

**Queues:**
- `q_motor_fbHandle` (8 items) — CAN2 motor feedback: ISR → Motor RX task
- `q_optflow_dataHandle` (8 items) — CAN1 optical flow data: ISR → OptFlow RX task
- `q_imu_dataHandle` — IMU data queue (may be deprecated/unused)
- `q_can1_rxHandle` — CAN1 receive queue

### Key configurable parameters

(from [`control_params.hpp`](Firmware/Component/control_params.hpp), namespace `control_config`)

**Yaw control:**
| Parameter | Value | Description |
|-----------|-------|-------------|
| `kYawTargetLowPassCutoffHz` | 10.0 | Yaw target smoothing |
| `kYawAnglePidKp` | 20.0 | Outer angle P gain (rad/s per rad) |
| `kYawAnglePidKi` | 3.0 | Outer angle I gain |
| `kYawAnglePidKd` | 0.0 | Outer angle D gain (on measurement) |
| `kYawRateControllerBandwidth` | 75.0 | Inner rate P gain ωc (rad/s) |
| `kLesoAngleObserverBandwidth` | 250.0 | LESO bandwidth ωo (min==max, fixed) |
| `kYawVyCoupling` | 0.0 | Lateral velocity feedforward to yaw |

**Chassis velocity PID:**
| Parameter | Value | Description |
|-----------|-------|-------------|
| `kChassisVxRefButterworthCutoffHz` | 10.0 | Target velocity LPF |
| `kChassisVelPidKpX` / `kChassisVelPidKpY` | 30.0 | Velocity P gain |
| `kChassisVelPidKiX` / `kChassisVelPidKiY` | 300.0 | Velocity I gain |
| `kChassisVelPidOutputLimitX` / `kChassisVelPidOutputLimitY` | 10.0 | Output clamp (m/s²) |

**Velocity fusion:**
| Parameter | Value | Description |
|-----------|-------|-------------|
| `kChassisVelocitySource` | 2 | Default: adaptive wheel+optflow Kalman |

**Wheel motor:**
| Parameter | Value | Description |
|-----------|-------|-------------|
| `kWheelSpeedPidKp` | 0.06 | Fallback speed P gain |
| `kWheelSpeedPidKi` | 0.6 | Fallback speed I gain |
| `kWheelSpeedPllBandwidth` | 75.0 | Velocity PLL bandwidth (rad/s) |
| `kWheelObsVelocityBandwidth` | 100.0 | Luenberger observer bandwidth (rad/s) |
| `kWheelVirtualDampingNmPerRadPS` | 0.027 | Virtual damping coefficient |
| `kWheelVirtualDampingMin` | 0.005 | Damping floor at high speed |
| `kWheelDampScheduleSpeedRadPS` | 10.0 | Damping scheduling knee (rad/s) |
| `kWheelTorqueFfLimitNm` | 0.0 | Torque FF clamp (disabled when using fallback) |
| `kUseWheelSpeedPidFallback` | true | Use per-wheel speed PID instead of torque-based yaw control |

**Robot physical model:**
| Parameter | Value | Description |
|-----------|-------|-------------|
| `kRobotMassKg` | 4.0 | Robot mass |
| `kRobotInertiaKgM2` | 0.008 | Yaw moment of inertia |
| `kWheelRadiusM` | 0.033 | Wheel radius |
| `kWheelCenterDistanceM` | 0.0785 | Wheel center distance from robot center |

**Dribbler hybrid control:**
| Parameter | Value | Description |
|-----------|-------|-------------|
| `kDribblerHybridTorqueNm` | -0.05 | Ball capture torque |
| `kDribblerHybridSpeedRps` | -100.0 | Ball hold speed |
| `kDribblerHybridBallHoldFrames` | 20 | Frames to confirm ball hold (40ms) |
| `kDribblerHybridBallLostFrames` | 10 | Frames to confirm ball lost (20ms) |

**Motor safety:**
| Parameter | Value | Description |
|-----------|-------|-------------|
| `kMotorRecoverDelayMs` | 1000 | Wait before clear-error on motor fault |
| `kMotorClearErrorToEnableDelayMs` | 10 | Wait after clear-error before re-enable |
| `kMITKdRampTimeSec` | 1.0 | Kd ramp-up time on MIT mode entry |
| `kWheelSpeedPidKpRampTimeSec` | 1.5 | Kp ramp-up time for fallback PID |

## Naming conventions (per Docs/README.md)

- **Classes/Types:** CamelCase (e.g., `MotorController`, `ChassisEstimator`)
- **Variables/Functions:** snake_case (e.g., `motor_speed`, `calculate_pid_output()`)

## CubeMX regeneration hazard

When regenerating code with STM32CubeMX, `Firmware/Board/Core/Src/main.c` may auto-generate `HAL_TIM_PeriodElapsedCallback`, causing a duplicate-definition linker error. Manually comment it out in `main.c` after regeneration.
