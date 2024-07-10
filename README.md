FCW with STM32F103C8 BluePill, URM37 V4 Ultrasonic Sensor, and TFLuna Lidar Sensor
This project implements Forward Collision Warning (FCW) using the STM32F103C8 BluePill microcontroller, the URM37 V4 ultrasonic sensor, and the TFLuna Lidar sensor. The system detects obstacles and alerts the driver to potential front-end collisions using dynamic equations to determine the safe distance and the level of warning.

Table of Contents
Introduction
Components
Wiring Diagram
Software
Installation
Usage
Introduction
Forward Collision Warning (FCW) alerts the driver of potential front-end collisions. This project integrates this functionality using an STM32F103C8 BluePill microcontroller, a URM37 V4 ultrasonic sensor, and a TFLuna Lidar sensor. The system uses dynamic equations to determine the safe distance between the vehicle and an obstacle, and to decide the level of warning.

Components
STM32F103C8 BluePill
URM37 V4 Ultrasonic Sensor
TFLuna Lidar Sensor
Power Supply (e.g., battery pack)
Connecting wires
Breadboard (optional)
Wiring Diagram

Connect the components as follows:

URM37 V4 Ultrasonic Sensor to BluePill:
VCC to 5V
GND to GND
TX to a selectable UART TX pin (e.g., PA9 for USART1_TX)
RX to a selectable UART RX pin (e.g., PA10 for USART1_RX)
TFLuna Lidar Sensor to BluePill:
VCC to 5V
GND to GND
TX to a selectable UART TX pin (e.g., PA2 for USART2_TX)
RX to a selectable UART RX pin (e.g., PA3 for USART2_RX)
Warning levels:
Low-level warning (indicates possible collision) to PA15
High-level warning (indicates immediate need to brake) to PA12
Software
The software is written in C/C++ and uses the STM32 HAL library. Key features include:

USART communication with URM37 V4 and TFLuna Lidar
Distance measurement and collision detection
Calculation of safe distance using dynamic equations
Warning level outputs on PA12 and PA15
Installation
Clone the repository:

bash
Copy code
git clone https://github.com/your_username/FCW_Stm32F103C8.git
cd FCW_Stm32F103C8
Install the required development tools:

STM32CubeMX
Keil uVision or STM32CubeIDE
Open the project in your IDE and build it.

Usage
Flash the firmware to the BluePill using a suitable programmer (e.g., ST-Link).
Power the system and observe the behavior:
The system should alert when a potential collision is detected.
Low-level warnings (indicating possible collision) will be indicated on PA15.
High-level warnings (indicating immediate need to brake) will be indicated on PA12.