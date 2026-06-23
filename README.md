````markdown
# quadcopter_freeRTOS

A quadcopter flight controller based on **STM32F103** and **FreeRTOS**.

This project implements the core components of a real-time flight control system, including remote-control communication, sensor acquisition, attitude estimation, flight state management, flight control, motor mixing, and safety protection.

The project is intended as a learning-oriented implementation of an embedded real-time flight controller and demonstrates how a complete flight control system can be organized using FreeRTOS.
[link to the remote control code](https://github.com/favor0629/quadcopter_remote)
---

# Features

- FreeRTOS-based multi-task architecture
- nRF24L01 wireless remote-control communication
- MPU6050 IMU data acquisition
- Attitude estimation using IMU and filtering algorithms
- Flight state management (Arm / Disarm / Flying / Failsafe)
- PID-based flight controller
- Quadcopter motor mixer
- Safety protection against signal loss and abnormal conditions

---

# System Architecture

The flight controller operates as a closed-loop control system.

```
Remote Controller
        │
        ▼
 nRF24L01 Receiver
        │
        ▼
 Remote Input Module
        │
        ▼
 Flight State & Safety
        │
        ▼
 Attitude Estimation
        │
        ▼
 Flight Controller
        │
        ▼
   Motor Mixer
        │
        ▼
    PWM Output
        │
        ▼
    ESC & Motors
```

The control loop works as follows:

1. Receive commands from the remote controller.
2. Validate the received remote data.
3. Read raw sensor data from the MPU6050.
4. Estimate the current attitude using the IMU module.
5. Execute the flight controller based on the desired input and current attitude.
6. Mix the controller outputs into four motor commands.
7. Output PWM signals to the ESCs.
8. Switch to protection mode if remote communication or sensors fail.

---

# Project Structure

```
.
├── User/          System initialization and FreeRTOS task creation
├── modules/       High-level flight modules
├── driver/        Device drivers (nRF24L01, MPU6050, etc.)
├── Hardware/      Board Support Package (USART, SPI, PWM, I2C, LED, Motor...)
├── hal/           Algorithms (PID, IMU, Filters, Math utilities)
└── freertos/      FreeRTOS kernel and configuration
```

---

# FreeRTOS Task Scheduling

The tasks are created in `User/freeRTOS_demo.c`.

| Task | Description | Period | Priority |
|------|-------------|--------|----------|
| `StartTaskCreate` | Initializes modules and creates all application tasks | Once | 3 |
| `RemoteTask` | Receives and parses remote-control packets | 10 ms | 4 |
| `SensorTask` | Reads MPU6050 data and updates attitude estimation | 30 ms | 5 |
| `FlightTask` | Executes safety checks, flight controller, and motor mixer | 15 ms | 6 |
| `LEDTask` | Updates system status LEDs | Independent | 3 |
You can adjust the priority and cycle of tasks as long as they meet your needs or are reasonable.
---

# Startup Sequence

System initialization follows the sequence below:

1. `freeRTOS_demo()` creates `StartTaskCreate`.
2. `StartTaskCreate()` initializes all modules:

   - `Remote_Init()`
   - `FlightState_Init()`
   - `ArmDetector_Init()`
   - `FlightControl_Init()`

3. Create all FreeRTOS tasks.
4. Start the scheduler.
5. Tasks begin executing periodically.

---

# Development Notes

- Remove the propellers during the first hardware tests.
- Verify the motor rotation direction before flight.
- Ensure the nRF24L01 communication link is stable.
- Verify MPU6050 data before enabling the controller.
- Keep serial logging lightweight to maintain real-time performance.
- Assign FreeRTOS priorities carefully to prevent CPU starvation.
- Tune controller parameters gradually after verifying the basic system.

---

# Debugging Checklist

If the system is not working as expected, verify the following in order:

1. Remote communication is working correctly.
2. IMU data is updating normally.
3. Attitude estimation is stable.
4. Flight state enters `FLIGHT_FLYING`.
5. Controller outputs are reasonable.
6. Motor mixer outputs are correct.
7. PWM signals reach the ESCs.

If the system remains in failsafe mode, check:

- `remote.valid`
- `remote.failsafe`
- `safety.remote_lost`

---

# Safety

This project controls real motors.

Always remove the propellers during development and debugging.

Test each subsystem independently before performing a complete flight test.

A recommended validation order is:

```
Hardware Drivers
        ↓
Sensors
        ↓
Remote Control
        ↓
Safety State Machine
        ↓
Flight Controller
        ↓
Motor Mixer
        ↓
Motor Output
```

---

# License

This project is released for learning and educational purposes.
````
