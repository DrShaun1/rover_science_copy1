# rover_science

Deployment-ready **ROS 2–based software stack** used to control the **science assay subsystem** on the University of New Haven’s **University Rover Challenge (URC) 2025 rover**.

This repository reflects the **final version deployed during competition**. Prior to URC, the system was intentionally simplified by removing non-essential or unreliable features in favor of **robustness, predictability, and ease of debugging under competition conditions**. As a result, this version performed as intended during the science mission and represents a stability-focused deployment snapshot rather than a research prototype.

---

## Overview

The rover science system enables remote control and monitoring of the science assay hardware using a **ROS 2–based architecture**. Control is distributed between a rover-mounted **Raspberry Pi 5** and an operator-side **base station**, communicating through ROS 2 nodes and custom hardware interfaces.

While referred to as a “base station,” the control software can be deployed on **any Linux machine** with the required Python dependencies and ROS 2 installed.

---

## Key Features

- ROS 2 used as the system-level control and communication layer
- Custom Python hardware interfaces built on modified open-source libraries
- Operator GUI written in Python for real-time control and feedback
- Designed for deployment on Raspberry Pi 5
- Hardware communication via embedded interfaces (UART, SPI, I²C, CAN)
- Competition-hardened implementation prioritizing reliability over features

---

## System Architecture (High-Level)

- **Base Station (Linux laptop)**
  - Python GUI for operator control
  - ROS 2 nodes for command and telemetry

- **Rover (Raspberry Pi 5)**
  - ROS 2 nodes handling science control logic
  - Custom Python hardware interfaces to sensors and actuators

ROS 2 serves as the “umbrella” layer coordinating communication between the base station and rover hardware, enabling modular control and simplified debugging.

---

## Competition Context

This software was developed and deployed by the **University of New Haven Robotics Club** for the **2025 University Rover Challenge**, an international collegiate robotics competition hosted by the Mars Society.

The system was used during the **actual science mission** under real-world constraints including time pressure, limited debugging access, and environmental variability.

---

## Related Links

- UNH at the International Rover Competition  
  https://www.newhaven.edu/news/blog/2025/international-rover-competition.php

- URC 2025 Rover Video  
  https://www.youtube.com/watch?v=aSnGvKKtzas

- Charger Robotics (UNH Robotics Club)  
  https://www.instagram.com/charger_robotics/?hl=en

- University Rover Challenge  
  https://urc.marssociety.org/

- University of New Haven Robotics  
  https://robotics.newhaven.edu/robotics-club/

---

## Notes

This repository represents a **deployment-focused engineering tradeoff**: stability and determinism were prioritized over experimental features. Some functionality present during development was intentionally removed prior to competition to reduce system complexity and failure modes.

---

## Contact

If you have questions about the system design, implementation details, or competition context, feel free to reach out:

**Shaunessy Reynolds**  
📧 shaunessy.business@gmail.com
