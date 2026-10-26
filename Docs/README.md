# FreeRTOS 机器人主控项目说明文档

## 1. 项目架构
本项目固件代码位于 `Firmware` 目录下，各子文件夹职责如下：

- **Board/**
  包含板级支持代码。主要包括 STM32CubeMX 生成的底层驱动 (`Drivers/`)、中间件 (`Middlewares/`，包含 FreeRTOS)、核心配置 (`Core/`) 以及板级初始化代码 (`board.cpp`, `board.h`)。

- **Communication/**
  负责通信协议的实现，例如 CAN 总线通信 (`can/`) 等。

- **Component/**
  包含机器人各个功能组件的驱动与逻辑实现，如 IMU (`imu.cpp`)、轮电机 (`wheel_motor.cpp`) 和吸球电机 (`dribbler_zfoc.cpp`)、光流 (`opt_flow.cpp`) 以及机器人主体类 (`robot.cpp`)。

- **Task/**
  包含 FreeRTOS 的各个任务实现（如控制任务 `ctrl_task`、IMU 任务 `imu_task` 等）以及 C++ 主程序入口 (`main.cpp`, `z_main.h`)。

## 2. 命名规范
后续开发请严格遵循以下命名范式：

- **类 (Class) 和类型 (Type)**：使用 **大驼峰命名法 (CamelCase)**。
  - 示例：`MotorController`, `ImuDataStruct`
- **变量 (Variable) 和函数 (Function)**：使用 **下划线命名法 (snake_case)**。
  - 示例：`motor_speed`, `calculate_pid_output()`

## 3. 开发指南

- 估计、轨迹、运动学和控制算法统一位于 Kun2 的 `motion/` 共享内核。固件通过 `Component/motion_engine.hpp` 接入，不另建重复控制环。
- `Component/` 只维护设备驱动、传感器快照和执行器编码；`Task/` 负责调度、互斥和硬件访问。
- 当前 SPI 帧版本为 3；CM4、Kun2 与固件必须配套更新。不保留旧协议或旧控制后端。
- 代码改动后在 `Firmware/` 执行 `cmake --build build/Debug`。本机完整验收入口为 Kun2 根目录的 `./kun2 test motion-all`，不使用硬件。

## 4. 已知问题与注意事项
- **HAL_TIM_PeriodElapsedCallback 重复定义错误**
  - **问题描述**：STM32CubeMX 重新生成代码时，可能会在 `Firmware/Board/Core/Src/main.c` 中自动生成 `HAL_TIM_PeriodElapsedCallback` 函数，导致与项目其他位置定义的同名函数冲突，产生编译错误。
  - **解决方法**：请手动在 `Firmware/Board/Core/Src/main.c` 文件中找到该函数，并将其**注释掉**。
