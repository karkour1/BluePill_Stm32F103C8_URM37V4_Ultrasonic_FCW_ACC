# FCW with STM32F103C8 BluePill, URM37 V4 Ultrasonic Sensor, and TFLuna Lidar Sensor

## Introduction
This project implements Forward Collision Warning (FCW) using the STM32F103C8 BluePill microcontroller, the URM37 V4 ultrasonic sensor, and the TFLuna Lidar sensor. The system detects obstacles and alerts the driver to potential front-end collisions using dynamic equations to determine the safe distance and the level of warning.

## Components
- STM32F103C8 BluePill
- URM37 V4 Ultrasonic Sensor
- TFLuna Lidar Sensor
- Power Supply (e.g., battery pack)
- Connecting wires
- Breadboard (optional)

## Wiring Diagram

### Connect the components as follows:
- **URM37 V4 Ultrasonic Sensor to BluePill:**
  - VCC to 5V
  - GND to GND
  - TX to a selectable UART TX pin (e.g., PA9 for USART1_TX)
  - RX to a selectable UART RX pin (e.g., PA10 for USART1_RX)
  
- **TFLuna Lidar Sensor to BluePill:**
  - VCC to 5V
  - GND to GND
  - TX to a selectable UART TX pin (e.g., PA2 for USART2_TX)
  - RX to a selectable UART RX pin (e.g., PA3 for USART2_RX)
  
- **Warning levels:**
  - Low-level warning (indicates possible collision) to PA15
  - High-level warning (indicates immediate need to brake) to PA12

## Software
The software is written in C/C++ and uses the STM32 HAL library. Key features include:
- USART communication with URM37 V4 and TFLuna Lidar
- Distance measurement and collision detection
- Calculation of safe distance using dynamic equations
- Warning level outputs on PA12 and PA15

## Installation
1. **Clone the repository:**
   ```bash
   git clone https://github.com/your_username/FCW_Stm32F103C8.git
   cd FCW_Stm32F103C8
2. **Install the required development tools:**
   - STM32CubeMX
   - Keil uVision or STM32CubeIDE
3. **Open the project in your IDE and build it:**
   - Configure your STM32 microcontroller settings using STM32CubeMX.
   - Open the generated project in Keil uVision or STM32CubeIDE.
   - Build the project to compile the firmware.

## Usage
1. **Flash the firmware to the BluePill:**
   - Connect your ST-Link programmer to the BluePill board.
   - Flash the compiled firmware using your IDE's programming tool.

2. **Power the system and observe the behavior:**
   - Ensure all connections are secure and power on the BluePill board.
   - The system should initialize and start monitoring for obstacles.
   - When a potential collision is detected:
     - A low-level warning (PA15) indicates a possible collision.
     - A high-level warning (PA12) indicates an immediate need to brake.
