# BLE Duplex Communication using nRF52832

## Overview
This project demonstrates bidirectional (duplex) Bluetooth Low Energy (BLE)
communication between two nRF52832 boards, configured as Central and
Peripheral using the Arduino IDE. The system enables command transfer
from the Central to the Peripheral and continuous sensor data transmission
from the Peripheral back to the Central.

## Hardware
- nRF52832 (ISC Devkit)
- HC-SR04 Ultrasonic Sensor
- Capacitive Touch Sensor
- 16×2 I2C LCD Display
- LED, jumper wires

## Software
- Arduino IDE
- ISC nRF52 Arduino Core
- Embedded C/C++

## Description
The Central and Peripheral exchange data in real time using BLE characteristics
implemented via the Nordic UART Service (NUS). The Central handles scanning, connection,
command transmission, and data display, while the Peripheral manages sensor acquisition,
BLE advertising, and notifications.

## Code Files
- Ble_Central.ino – Central device firmware
- Ble_Peripheral.ino – Peripheral device firmware

## System Architecture
- **Central Node**
  - BLE Central role
  - Touch sensor for user input
  - I2C LCD for displaying received data

- **Peripheral Node**
  - BLE Peripheral role
  - Ultrasonic distance sensor
  - LED for command acknowledgment

## Development Approach
- Individual bring-up and testing of sensors and display
- BLE advertising, scanning, and connection validation
- Integration of peripherals with BLE roles
- Implementation of full-duplex data exchange
- Debugging and optimization for reconnection and stability

## Challenges & Fixes
- BLE auto-reconnect failure resolved by switching to name-based scanning
- LCD corruption fixed by correcting power connection
- Command reception issue solved using proper newline termination
- Touch sensor debounce handled using toggle-state logic
- LCD overflow corrected by manual line splitting

## Results
- Stable BLE connection with < 1 second auto-reconnection
- Accurate ultrasonic sensor readings
- Responsive command control
- Clean and consistent LCD output
- Reliable behavior across repeated resets and disconnections

## Conclusion
This project presents a practical implementation of a reliable Central–Peripheral BLE system
on the nRF52832 platform. Through iterative debugging and structured testing, stable duplex communication
and real-time data exchange were successfully achieved using an Arduino-based development workflow.

## References
- Nordic Semiconductor – nRF52832 Product Specification
- ISC nRF52 Arduino Core Documentation
- BLE GATT and Nordic UART Service (NUS) Documentation
- HC-SR04 Ultrasonic Sensor Datasheet
