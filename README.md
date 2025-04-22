# Autonomous Thermal Search and Rescue Robot

This project presents a custom-built autonomous ground robot designed to simulate a post-earthquake search and rescue scenario. The robot explores an unknown environment using SLAM, identifies potential human survivors based on heat signatures, and marks their locations by deploying a visual flare mechanism (ping pong ball shooter).

The system was developed on the TurtleBot3 Burger platform, integrating mechanical actuation, custom electronics, and a complete ROS2-based navigation and sensing software stack.

---

## System Overview

The robot autonomously locates **two heat sources**, simulating trapped humans, and activates a mechanical actuator to launch a flare at each source. It avoids revisiting previously detected targets and explores until both are found.

---

## Mechanical Subsystem

A custom **cam-cantilever-spring mechanism** forms the core of the actuator, which launches a ping pong ball upon confirmation of a heat source.

- **Mechanism**: Surface cam, spring-loaded lever, and firing frame  
- **Stroke**: 14 mm  
- **Actuation**: Driven by a motor with gearbox

Refer to the [`mechanical/`](./mechanical/) directory for design files and CAD models.

---

## Electrical Subsystem

The electrical architecture is built around a **custom-designed PCB** mounted on the Raspberry Pi, ensuring robust connectivity and easier integration of power and signal lines.

- **Power Supply**: PCB powered via OpenCR 5V output  
- **Components**: Connects thermal sensor, motor driver (L298N), and GPIO-controlled MOSFET  
- **Testing**: JLCPCB performed electrical tests; assembly and validation were done at E2 Electronics Lab  
- **Note**: GPIO pin 24 was intentionally left unused due to a layout issue

All schematics, BOMs, and datasheets are available in the [`electrical/`](./electrical/) folder.

---

## Software Subsystem

The software is implemented using **ROS2 Humble**, running on a Raspberry Pi. It integrates exploration, navigation, thermal detection, and actuator control.

**Key components**:

- **SLAM**: Real-time mapping using `slam_toolbox`  
- **Exploration**: Custom ROS2 node interfacing with the Nav2 stack  
- **Heat Source Detection**: AMG8833 thermal sensor, processed with thresholding  
- **Flare Deployment**: GPIO control of actuator once source is confirmed

Full implementation, test scripts, and configuration are located in the [`software/`](./software/) directory.

---

## Testing & Integration

- Integrated system was tested in a controlled environment  
- PCB validation, sensor communication, and actuator firing were verified independently before full integration  
- Software stack was tested with both real-time inputs and simulated data

---

## Team

- Srivastava Harshit
- Wong Jian Bin
- Singh Siddhant Narayan
- Pranav Vangal Swaminathan
- Ni Hang, Alex
