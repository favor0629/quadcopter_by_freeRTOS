# balance_car_freeRTOS

## 项目概述

这是一个基于 STM32F103 的 FreeRTOS 平衡车控制项目。核心目的是通过 MPU6050 采集角度/陀螺仪数据，使用串级 PID 控制算法驱动双电机，实现自平衡、速度控制和转向控制。本项目是对江协科技平衡车项目的重构freeRTOS版本。

## 主要功能

- FreeRTOS 多任务调度
- MPU6050 IMU 采集与互补滤波姿态估计
- 角度环 PID 控制（自平衡核心）
- 速度环 PID 控制
- 转向环 PID 控制
- 按键启动/停止控制
- OLED 实时显示状态与 PID 参数
- 蓝牙遥控与 PID 参数在线调节
- 离地保护与离地恢复机制
- 编码器测速度，电机 PWM 输出控制

## 目录结构

- `freertos/` - FreeRTOS 内核与配置文件
- `Hardware/` - 外围设备驱动：按键、LED、OLED、MPU6050、Motor、Encoder、蓝牙串口
- `Library/` - STM32 标准库驱动与辅助函数
- `Start/` - 启动文件与芯片头文件
- `System/` - 系统级支持模块（定时器、中断等）
- `User/` - 应用层代码：主程序、演示、FreeRTOS 任务逻辑
- `Project.uvprojx` / `Project.uvoptx` - Keil MDK5 工程文件

## 关键模块说明

### `User/main.c`

主控制程序，包含：

- `Module_Init()`：初始化 OLED、LED、Key、Encoder、Motor、USART、MPU6050、蓝牙和 PID
- `IMU_Task()`：10ms 读取 MPU6050，计算加速度角度、陀螺仪积分值并做互补滤波
- `Control_Task()`：1ms 主控制任务，负责按键开关、角度环、速度环、转向控制、离地保护、离地恢复
- `Bluetooth_Task()`：蓝牙命令解析与 PID 参数在线调节
- `OLED_Task()`：100ms 刷新 OLED 显示当前角度、速度、PID 参数等
- `Key_Task()`、`LED_Task()`：按键与 LED 状态管理



## 注意事项

- `FreeRTOSConfig.h` 已配置 FreeRTOS 基础功能，确保 `configUSE_TIMERS`、`configUSE_TASK_NOTIFICATIONS` 等满足需求
- `NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4)` 是 FreeRTOS 在 Cortex-M3 上推荐的优先级分组
- `run_flag` 状态：`0=停止`，`1=运行`，`2=离地恢复`

## 编译与调试

- 推荐使用 Keil MDK5 打开 `Project.uvprojx`
- 目标芯片为 STM32F103 系列
- 如果要调试串口输出，可使用 `usart_printf`

## 进一步扩展

- 增加蓝牙传感器数据上报与调试命令
- 加入更多电机保护、编码器错误检测
- 优化 PID 参数自适应与整车调参界面
- 改善离地恢复策略与安全限幅
