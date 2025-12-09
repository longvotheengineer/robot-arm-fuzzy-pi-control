# Intelligent 1-DoF Robotic Actuator with Adaptive Fuzzy-PI Control

[![Platform](https://img.shields.io/badge/Platform-STM32%20ARM%20Cortex--M3-blue)](https://www.st.com/en/microcontrollers-microprocessors/stm32f103c8.html)
[![Control](https://img.shields.io/badge/Control-Fuzzy%20Logic%20%7C%20PID-orange)](https://www.mathworks.com/discovery/fuzzy-logic.html)
[![Simulation](https://img.shields.io/badge/Simulation-MATLAB%20%2F%20Simulink-orange)](https://www.mathworks.com/products/simulink.html)
[![Interface](https://img.shields.io/badge/HMI-C%23%20Windows%20Forms-purple)](https://dotnet.microsoft.com/)
[![License](https://img.shields.io/badge/License-MIT-lightgrey.svg)](LICENSE)

##  Project Abstract

This repository houses the firmware, simulation models, and control software for a **High-Precision Single-Degree-of-Freedom (1-DoF) Robotic Joint**. The system is engineered to solve non-linear dynamic tracking problems inherent in DC motor actuation under varying load disturbances.

By implementing a **Hybrid Fuzzy-PI Controller** on an STM32 microcontroller, this project achieves superior trajectory tracking and disturbance rejection compared to classical linear PID methods. The system features a closed-loop architecture with real-time telemetry visualized via a custom C# Host Interface.

## Key Features

* **Advanced Control Algorithms:** Implementation of a Mamdani-type Fuzzy Inference System (FIS) to dynamically tune PI gains based on error ($\epsilon$) and error rate ($\dot{\epsilon}$).
* **Model-Based Design:** Full dynamic modeling and validation using MATLAB/Simulink (`DC_pos_AN.slx`) prior to hardware deployment.
* **Real-Time Embedded Firmware:** High-frequency control loop (PWM generation & Encoder feedback) running on the STM32F103C8T6.
* **Hardware-in-the-Loop (HIL) Monitoring:** Custom Windows GUI for setting position targets ($45^\circ, 90^\circ, 180^\circ$) and visualizing step response data in real-time.
* **Robustness:** Experimental verification showing stability under load variations (Inertia $J$ variation up to $0.005 kg.m^2$).

## System Architecture

### 1. Mathematical Modeling
The actuator is modeled as a second-order dynamic system governed by the electromechanical equation:

$$\Gamma = K \cdot V = J\ddot{\theta} + \nu\dot{\theta} + \tau_{load}$$

*Where:*
* $K$: Motor constant ($0.2082 Nm/V$)
* $J$: Moment of inertia
* $\nu$: Viscous friction coefficient

### 2. Control Strategy (Fuzzy-PI)
Unlike fixed-gain PID controllers, the Fuzzy-PI controller adapts to system non-linearities.
* **Inputs:** Normalized Position Error ($E$) and Integral of Error ($IntE$).
* **Fuzzy Sets:** 3 MFs for Error (Neg, Zero, Pos) and 5 MFs for Integral (NegBig to PosBig).
* **Inference Engine:** 15 distinct rules derived from heuristic control knowledge.
* **Defuzzification:** Centroid method to determine the precise control signal ($u$).

## Hardware Specification

| Component | Description | Function |
| :--- | :--- | :--- |
| **MCU** | STM32F103C8T6 (Blue Pill) | Main Control Unit (72MHz) |
| **Actuator** | GA25-370 DC Geared Motor | 12V High-Torque Drive |
| **Feedback** | Quadrature Encoder | High-resolution position feedback (224 PPR) |
| **Driver** | L298N H-Bridge | Power amplification and PWM drive |
| **Interface** | CP2102 USB-TTL | Serial communication with Host PC |

## Software & Toolchain

### Firmware (Embedded C)
* **IDE:** Keil uVision 5
* **HAL:** STM32Cube HAL
* **Peripherals:** TIM1 (PWM Generation), TIM2 (Encoder Mode), TIM3 (Sampling Loop), UART1 (Telemetry).

### Host Application (C# .NET)
* **Framework:** .NET Windows Forms
* **Features:**
    * Serial Port Configuration.
    * Live plotting of Setpoint vs. Actual Position.
    * Manual and Automatic Trajectory generation.

### Simulation (MATLAB)
* **File:** `DC_pos_AN.slx` & `DCVelMamdani3.fis`
* **Purpose:** Pre-silicon validation of the Fuzzy Logic rules and stability analysis.

## Performance Results

The system was tested under no-load and loaded conditions (approx 1.2kg load).

| Controller Type | Rise Time | Overshoot | Steady-State Error | Robustness |
| :--- | :--- | :--- | :--- | :--- |
| **Classic PI** | Moderate | High (with load) | Low | Low |
| **Fuzzy PI** | **Fast** | **Minimal** | **~0%** | **High** |

> *The experimental results demonstrate that the Fuzzy PI controller significantly reduces overshoot and settling time compared to the conventional PI controller, especially when external disturbances (load) are applied.*

## Getting Started

### 1. Simulation
1.  Open MATLAB.
2.  Load the Fuzzy Inference System: `readfis('DCVelMamdani3.fis')`.
3.  Open and run `DC_pos_AN.slx`.

### 2. Firmware
1.  Open `Led_Button.uvprojx` in Keil uVision 5.
2.  Compile and Flash to the STM32F103.

### 3. Control Interface
1.  Open `GUI_DLCN.sln` in Visual Studio.
2.  Build and Run.
3.  Connect the USB-TTL to the PC and select the COM port.

## Authors & Acknowledgments

**Project Team - HCMUT (Ho Chi Minh city University of Technology):**
* **Nguyen Tuan Anh** - Modeling & Simulation
* **Truong Tuan An** - Control Theory & Analysis
* **Vo Que Long** - Hardware Integration & Experimental Validation

**Supervisor:**
* Assoc. Prof. Dr. Huynh Thai Hoang

---
*© 2025 Intelligent Control Systems Lab - HCMUT*
