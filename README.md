# Line Follower Robot with QTR Sensor

## Overview
This project involves building an autonomous **Line Follower Robot** using a **QTR sensor array** for path detection. The bot is controlled by an **Arduino Nano**, which processes real-time sensor data and adjusts motor speeds using a **PID controller algorithm** to ensure smooth and precise movement.

## Hardware Components Required
- **QTR Sensor Array** - Detects the line and provides feedback on position.
- **Arduino Nano** - Microcontroller for processing sensor data and controlling the motors.
- **Motor Driver (DRV8835 or L298N)** - Controls motor speed and direction.
- **Li-ion Battery** - Powers the system.
- **N20 Geared Motors / DC Motors** - Provides movement.
- **Jumper Wires** - For circuit connections.
- **Chassis and Wheels** - Forms the body and structure of the bot.

## Circuit Overview
- The **Arduino Nano** is connected to the **motor driver** and **QTR sensor array**.
- The **motor driver** controls the motors based on speed values calculated by the Arduino Nano.
- The **QTR sensor array** detects the position of the line and sends data to the Arduino.
- The Arduino processes this data and adjusts motor speeds using a **PID controller** to keep the bot on track.

## Software
### Libraries Used
- **QTRSensors.h**

### Setting Up Libraries
1. Open the **Arduino IDE**.
2. In the Arduino IDE, go to **Sketch** > **Include Library** > **Manage Libraries**.
3. Search for **QTRSensors** in the search bar.
4. Click the **QTRSensors** entry in the list.
5. Click the **Install** button to install the library.

## PID Controller Algorithm
The **QTR sensor array** provides values ranging from **0 to 7000**, where **3500** represents the center of the line. This allows the robot to determine how far it has deviated from the line.

### How PID Works:
1. **Proportional Term (P):**
   - Based on the current error value (distance from the center of the line).
   - Determines how much the robot should turn to correct its position.
2. **Integral Term (I):**
   - Accumulates past errors to minimize steady-state error.
3. **Derivative Term (D):**
   - Predicts future errors based on the rate of change of the error value.
   - Helps reduce overshooting and oscillations.

By fine-tuning the **P, I, and D gains**, the controller ensures that the **robot follows the line smoothly and accurately**.

## Usage
- Place the bot on a **black line on a white surface**.
- Turn it on and let it calibrate for a few seconds.
- Observe as it autonomously follows the path, adjusting its speed dynamically.
