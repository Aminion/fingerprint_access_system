# Fingerprint Access System

[![Rust](https://img.shields.io/badge/Rust-1.75%2B-orange.svg)](https://www.rust-lang.org/)
[![Embassy](https://img.shields.io/badge/Framework-Embassy-blue.svg)](https://embassy.dev/)
[![Hardware](https://img.shields.io/badge/Hardware-STM32G031-green.svg)](https://www.st.com/en/microcontrollers-microprocessors/stm32g0-series.html)

## Project Overview

This project implements a complete biometric access control. It handles asynchronous UART communication for fingerprint authentication, ADC measurements for battery diagnostics, and PWM control for physical actuation. The firmware is written entirely in `no_std` Rust, utilizing the Embassy framework for concurrent, event-driven execution without the overhead of a traditional RTOS.

### Core Capabilities
* **Biometric Authentication:** Secure, asynchronous UART communication with a fingerprint module for user enrollment and verification.
* **PWM-Optimized Actuation:** Implementation of a "peak-and-hold" solenoid drive mechanism using Hardware PWM, significantly reducing active power consumption and thermal load.
* **Ratiometric Battery Diagnostics:** A custom ADC measurement strategy utilizing an external TL431 reference to accurately calculate battery capacity, regardless of internal 3.3V rail voltage sag.
* **Zero-State Power Gating:** Physical isolation of analog subsystems (voltage dividers, reference chips) via GPIO-controlled switching, ensuring 0mA draw from these components during system standby.

## Software Stack & Hardware Architecture

### Software
* **Language:** Rust (`no_std`)
* **Framework:** Embassy (Embedded Async)
* **Concurrency Model:** `embassy-executor` for lightweight task scheduling and interrupt management.

### Hardware & Peripherals
* **Microcontroller:** STM32G031K8 (Nucleo-32)
* **Sensor:** GROW R503
* **Power Supply:** 4xAAA NiMH Battery Pack (~4.0V - 6.0V)
* **Peripheral Utilization:**
  * **PWM (Pulse Width Modulation):** Configured to manage the solenoid's current draw, as well as generate user-feedback tones.
  * **ADC (Analog-to-Digital Converter):** Utilizes extended sample timing (`Cycles160_5`) and floating-point calibration math to bypass hardware limitations.
  * **UART (Universal Asynchronous Receiver-Transmitter):** Handles packet-based serial data transfer with the biometric sensor.
  * **GPIO:** Configured for push-pull power switching and standard digital I/O.

## Engineering Challenges Solved

### 1. Solenoid Power Optimization (Peak-and-Hold)
Solenoids require a high initial "pull-in" current to overcome mechanical inertia, but require significantly less current to maintain the unlocked state. Driving a solenoid at 100% duty cycle for the duration of the unlock period wastes battery capacity and generates excess heat.
* **Solution:** The firmware utilizes hardware PWM to drive the actuating transistor. It applies a 100% duty cycle for the initial actuation phase (e.g., 100ms), then instantly drops the duty cycle to a calculated holding percentage (e.g., 30%). This reduces the holding power consumption by up to 70% while keeping the lock disengaged.

### 2. Ratiometric ADC Calibration
On low pin-count STM32 microcontrollers, the analog reference pin (`AREF`) is internally bonded to `VDDA`. Consequently, when the main 3.3V rail sags under load, the ADC reference shifts, causing inaccurate battery readings.
* **Solution:** A TL431 (2.495V) precision external reference was introduced to a standard ADC channel. The firmware samples both the unknown battery voltage and the known TL431 voltage concurrently. By dynamically calculating the deviation of the TL431 reading, the software mathematically reconstructs the true scale of the 3.3V rail, resulting in a highly accurate battery percentage regardless of system load.
