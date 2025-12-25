# rover_science

Deployment-ready **ROS 2–based software stack** used to control the **science assay subsystem** on the University of New Haven’s **University Rover Challenge (URC) 2025 rover**.

This repository reflects the **final version deployed during competition**, optimized for **stability, predictability, and ease of debugging**. It represents thousands of hours of R&D across electrical, software, mechanical, and forensic science disciplines.

---

## Overview

- Remote control and monitoring of science assay hardware via **ROS 2 architecture**  
- Distributed control: rover-mounted **Raspberry Pi 5** ↔ operator-side **base station**  
- Base station software deployable on **any Linux machine** with Python & ROS 2  

---

## Key Features

- ROS 2 as umbrella control layer for commands and telemetry  
- Custom Python hardware interfaces (modified open-source libraries)  
- Operator GUI in Python for real-time control and feedback  
- Hardware interfaces: UART, SPI, I²C, CAN  
- Competition-hardened: reliability prioritized over extra features  

---

## System Architecture

**Base Station (Linux Laptop):**  
- Python GUI for operator control  
- ROS 2 nodes for command & telemetry  

**Rover (Raspberry Pi 5):**  
- ROS 2 nodes for science control logic  
- Custom Python hardware interfaces to sensors & actuators  

ROS 2 coordinates communication between the base station and rover hardware, enabling modular control and simplified debugging.

---

## Competition Context

- Developed for the **University Rover Challenge (URC 2025)** by UNH Robotics Club  
- Deployed in the **actual science mission** under extreme conditions:  
  - Time pressure & limited debugging access  
  - Ambient temperature ~102°F affecting materials  
  - Harsh desert terrain  

---

## Life-Detection Tests (Project Highlights)

### Bioluminescence Assay
- Mechatronic system drills, liquefies, and delivers soil samples to reaction chamber  
- Dual-drill system with a **detachable sample cache** for per individual untested soil sample collection
- Key focus on reliability under URC constraints  

### High-Temperature Burn Test
- Soil heated to **500 °C+** using **ZVS induction driver**, controlled via **VESC motor controller**  
- Post-burn carbon analysis using **SCD-41 sensor**  
- Hundreds of hours of **R&D, prototyping, and debugging**  

### Team Effort & Design Iteration
- Thousands of hours across **electrical, software, mechanical, and forensic science domains**  
- Engineering decisions influenced by URC rules, rover B.O.M., extreme environment, and modularity requirements  
- Emphasis on reliability, repeatability, and mission readiness  

---

## Relevant Links

- [UNH at the International Rover Competition](https://www.newhaven.edu/news/blog/2025/international-rover-competition.php)  
- [URC 2025 Rover Video](https://www.youtube.com/watch?v=aSnGvKKtzas)  
- [Charger Robotics (UNH Robotics Club)](https://www.instagram.com/charger_robotics/?hl=en)  
- [University Rover Challenge](https://urc.marssociety.org/)  
- [UNH Robotics Club](https://robotics.newhaven.edu/robotics-club/)  

---

## My Contact

**Shaunessy Reynolds – Electrical & Software Lead (Science Assay)**  
📧 shaunessy.business@gmail.com  
🔗 [LinkedIn](https://www.linkedin.com/in/shaunessy-reynolds/)  

---

## URC UNewHaven Science Assay Subteam LinkedIn Profiles

- **Emma Greybill** – Forensic Science Co-lead: [LinkedIn](https://www.linkedin.com/in/emma-graybill-a1bba1326/)  
- **Ashlyn Evans** – Forensic Science Co-lead: [LinkedIn](https://www.linkedin.com/in/ashlyn-c-evans/)  
- **Joseph Marcello** – Mechanical Lead: [LinkedIn](https://www.linkedin.com/in/joseph-marcello-687464237/)  
- **Erik Parker** – Electrical/Software Support: [LinkedIn](https://www.linkedin.com/in/erik-parker-engineer/)  
- **Shaunessy Reynolds** – Electrical & Software Lead: [LinkedIn](https://www.linkedin.com/in/shaunessy-reynolds/)  
