# rover_science

Deployment-ready **ROS 2–based software stack** used to control the **science assay subsystem** on the University of New Haven’s **University Rover Challenge (URC) 2025 rover**.

This repository reflects the **final version deployed during competition**. Prior/During to URC, the system was intentionally simplified by removing non-essential or unreliable features in favor of **robustness, predictability, and ease of debugging under competition conditions**. As a result, this version performed as intended during the science mission and represents a stability-focused deployment snapshot rather than a research prototype, despite thousands of hours of R&D across electrical, software, mechanical, and forensic science disciplines.

---

## Overview

The rover science system enables remote control and monitoring of the science assay hardware using a **ROS 2 based architecture**. Control is distributed between a rover-mounted **Raspberry Pi 5** and an operator-side **base station**, communicating through ROS 2 nodes and custom hardware interfaces.

While referred to as a “base station,” the control software can be deployed on **any Linux machine** with the required Python dependencies and ROS 2 installed.

---

## Key Features

- ROS 2 used as the system-level (umbrella) control and communication layer
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

The system was used during the **actual science mission** under real-world constraints including time pressure, limited debugging access, and significant environmental variability.

---

## Relevant Websites

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

This repository reflects a **deployment-focused engineering tradeoff** made for competition: stability, determinism, and recoverability were prioritized over experimental or convenience features. Some functionality present during development was intentionally removed prior to URC to reduce system complexity and minimize failure modes.

Because the full rover software stack remains private, certain interfaces or dependencies that interact with this repository may be missing. While no major issues have been observed so far, users attempting to redeploy this system outside its original context may encounter incomplete functionality.

This system was designed for a **highly specific operational environment**. The architecture, hardware interfaces, and control logic were tightly coupled to an existing bill of materials, a restricted budget, URC 2025 science mission rules and regulations, our own design parameters to ensure quality testing, and the requirement to operate while mounted on a moving rover in extreme desert conditions. During the science mission, ambient temperatures reached approximately 102°F, contributing to material and thermal constraints that directly influenced design decisions.

For the two life detection tests, it isn't noted much in the code here besides the controls for their set ups. (((Though at some point I will upload more documentation etc on everything to show the design and team decision process, specifically from our capstone final report and presentations))).

---

## Project Commentary / Personal Notes

### Overview
The science assay subsystem included two life-detection tests and required **extensive cross-disciplinary integration** across electrical, software, mechanical, and forensic science systems. Significant R&D went into both subsystems, which pushed the rover’s modularity, reliability, and functionality under extreme competition conditions.

### Bioluminescence Assay
This assay was largely theoretical from a forensic science perspective, but the **mechatronic delivery system**—which drills, liquefies, and delivers soil samples to the reaction chamber—was far more complex than initially anticipated. Designing a reliable collection and delivery system was a major challenge throughout the competition.

Key points:  
- Extensive iteration on sample collection and handling methods  
- Required integration of mechanical drills, modular sample caching, and precise liquid handling  
- Solutions were constrained by URC rules, rover B.O.M., and environmental conditions (heat, dust, arid terrain)

### Drill System & Sample Handling
To manage both assays, the team implemented a **dual-drill system**, with one drill dedicated to each test. The **modular sample cache** allowed independent handling of each assay, enabling repeatable operation under real-world constraints.

Design challenges included:  
- Limited rover weight and space  
- Reliable operation while mounted on a moving platform  
- Thermal constraints affecting materials (e.g., PLA at ~102°F)

### High-Temperature Burn Test
The burn test involved heating soil samples to 500 °C+ using a **ZVS induction driver**, controlled via a relay interfaced with a **VESC motor controller**, for post-burn carbon analysis using the SCD-41 sensor.

This subsystem required:  
- Hundreds of hours of **R&D, prototyping, and debugging**  
- Iterative sensor testing and hardware redesigns  
- Robust control logic and hardware selection to ensure reliability and safety

### Design Iteration & Team Effort
The team spent **thousands of hours** designing, implementing, and debugging these systems. Engineering decisions were heavily influenced by:  
- URC 2025 science mission rules  
- Rover hardware and B.O.M. constraints  
- Extreme environmental conditions  
- Remote operation and modularity requirements  

The system required constant collaboration across disciplines to achieve **reliability, repeatability, and mission readiness**.

### Additional Documentation
Future additions may include:  
- Capstone report  
- Bill of Materials (B.O.M.)  
- Team presentations  

Anyone attempting to **recreate or repurpose this system** is encouraged to reach out, as much of the engineering context is tightly coupled to the URC 2025 mission and specific constraints of the rover.

---

## My Contact

**Shaunessy Reynolds (Electrical and Software Lead for the Science Assay)**  
📧 shaunessy.business@gmail.com  
🔗 https://www.linkedin.com/in/shaunessy-reynolds/

---

## URC UNewHaven Science Assay Subteam LinkedIn Profiles

I wanted to acknowledge the entire subteam who contributed to the science assay, as this was truly a team effort:

- **Emma Greybill** – Forensic Science Co-lead  
  https://www.linkedin.com/in/emma-graybill-a1bba1326/

- **Ashlyn Evans** – Forensic Science Co-lead  
  https://www.linkedin.com/in/ashlyn-c-evans/

- **Joseph Marcello** – Mechanical Lead for Competition  
  https://www.linkedin.com/in/joseph-marcello-687464237/

- **Erik Parker** – Electrical and Software Support for Science Assay  
  https://www.linkedin.com/in/erik-parker-engineer/

- **Shaunessy Reynolds** – Electrical and Software Lead  
  https://www.linkedin.com/in/shaunessy-reynolds/

