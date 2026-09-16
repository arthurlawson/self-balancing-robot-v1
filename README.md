# 🤖 Self-Balancing Robot V1

### The Foundational Autonomous Proof-of-Concept Platform

The original, foundational self-balancing robot utilizing the **ESP32-S3 (N16R8)** computing core, driven by a custom **Kalman Filter** and high-frequency **PID control loop** system. 

This initial version was a raw engineering prototype built entirely within the ArduinoIDE toolchain to validate our baseline control loops and structural balancing physics before expanding into the upgraded V2 architecture.

> ⚠️ **LEGACY PROJECT NOTICE:** This repository is provided entirely **as-is**. I am no longer actively developing, updating, or providing technical troubleshooting for V1 as my focus has shifted to the upgraded, screaming V2 Edition. **Take the code with a grain of salt!**

---

## Core Features

*   **State Estimation:** Custom Kalman Filter implementation paired with an MPU6050 IMU for precise autonomous roll angle determination.
*   **Non-Linear Control:** Closed-loop feedback PID controller mechanics featuring manual integral leak adjustments to prevent deadzone windup.
*   **Dynamic Tuning:** Exponential gain-scaling algorithms that automatically scale motor loop coefficients during high-deviation tilts.
*   **Power Management:** Active analog voltage checks safeguarding the 2S LiPo system from dropping beneath unsafe cell margins.
*   **Crash Recovery:** Automated emergency motor shutdown constraints to instantly cut power if the platform tips over past 80 degrees.

---

## Firmware Deployment & Setup Guide

Since all required script files and external vendor dependencies (`Adafruit_MPU6050`) are packaged directly inside the local local project architecture, compilation is quick and straightforward:

> **CRITICAL COMPILATION NOTE:** For the project to compile successfully in Arduino IDE, **all downloaded firmware files must remain in the exact same directory folder**. Do not separate the auxiliary headers and source files (`pid.*`, `kalman.*`, `drv8833.*`, and `Adafruit_MPU6050.*`) from the main `balancing_robot.ino` file, as the code uses relative local path definitions (`#include "..."`) rather than global system libraries.

1. **Open the Project:** Launch Arduino IDE and open the main controller file: `firmware/balancing_robot.ino`.
2. **Install Board Support:** Go to **Tools** > **Board** > **Boards Manager**, search for `esp32` by Espressif Systems, and ensure it is installed.
3. **Configure Target Settings:** Select your target board under **Tools** > **Board** > **ESP32 Arduino** > **ESP32S3 Dev Module** and set these exact configurations:
   * **USB CDC On Boot:** `Enabled` *(Crucial for native Serial communications monitoring)*
   * **Flash Size:** `16MB (128Mb)`
   * **Partition Scheme:** `16MB Flash (3MB APP/9.9MB FATFS)`
   * **PSRAM:** `OPI PSRAM`
4. **Compile & Upload:** Connect your ESP32-S3 via USB, select the active COM Port, and click the **Upload** arrow.

---

## Quick Navigation Hub

To explore the source files or get started with the foundational prototype hardware, navigate through the directories below:

*   **[Firmware Core Files](./firmware/):** Access the prototype control stack, featuring our custom Kalman matrix filter math, DRV8833 slow-decay active braking driver, and the primary `balancing_robot.ino` loop configuration.
*   **[Hardware Assets & CAD](./hardware/):** Access to the raw SolidWorks (`.SLDPRT`) master part model and print-ready, sliced assembly configurations (`.3mf`) tailored directly for PLA fabrication layouts.

---

## Upgrading to V2?

If you want the fully polished, remote-controlled iteration featuring a custom main controller PCB, active battery safety tracking, dynamic addressable lighting arrays, and the functionality where **the robot screams in agony whenever it falls over**, head over to the main repository:

👉 **[Self-Balancing Robot V2 (Screaming Edition)](https://github.com/arthurlawson/self-balancing-robot-v2)**

---

## Project Credits & Licensing

This repository is distributed under a custom **Non-Commercial Open-Source License**. See the full terms inside the accompanying [LICENSE](./LICENSE) file.

*   **Kalman & PID Architecture:** Core state estimation filter mathematics designed by [@temperancee](https://github.com/temperancee).
*   **Refactoring & Systems Engineering:** Firmware structural cleanups, architecture optimizations, and safety layer adaptations implemented by [Arthur Lawson](https://github.com/arthurlawson).
