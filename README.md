# Smart Insole Microcontroller Code
This repository contains Arduino C++ code developed for a smart insole microcontroller based on the [Seeed Studio XIAO nRF52840 Sense](https://www.seeedstudio.com/Seeed-XIAO-BLE-Sense-nRF52840-p-5253.html) board. The project is designed to integrate multiple features for efficient data acquisition and communication.

Key Features:
Sensor Management:

Real-time scanning and reading of an IMU sensor and a 2×8 FSR sensor array.
Buffered data storage for reliable processing.
Bluetooth Communication:

BLE transmission using the BLE UART protocol for wireless data transfer.
Task Management:

Utilizes FreeRTOS for efficient multitasking and resource management.
Additional Functionalities:

Real-Time Clock (RTC) support.
Deep sleep and wake-up functionality for low-power operation.
Battery voltage monitoring for system health.
This repository provides a modular and robust foundation for smart insole development and related wearable applications.
