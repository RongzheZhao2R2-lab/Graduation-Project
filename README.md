# Design and Implementation of Autonomous UAV Attitude Control , Trajectory Tracking and Map Construction System

[![ROS Noetic](https://img.shields.io/badge/ROS-Noetic-blue)](http://wiki.ros.org/noetic)
[![PX4 Autopilot](https://img.shields.io/badge/PX4-Autopilot-black)](https://px4.io/)
[![VINS-Fusion](https://img.shields.io/badge/SLAM-VINS%20Fusion-purple)](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion)
[![OpenCV](https://img.shields.io/badge/Computer_Vision-OpenCV%204.0-green)](https://opencv.org/)
[![Language](https://img.shields.io/badge/C++-14-orange)](https://isocpp.org/)

**Architecture:** Distributed Embedded System (Jetson Xavier NX + Pixhawk 4)

---

## 1. System Abstract
This project implements a full-stack autonomy framework for a quadrotor UAV, capable of operating in GPS-denied environments. The system leverages **VINS-Fusion** for high-precision Visual-Inertial Odometry (VIO), fusing data from a RealSense D435i (Stereo + IMU). It features a tightly coupled architecture integrating **3D Occupancy Grid Mapping**, **Kinodynamic A_star Planning**, and **Geometric Tracking Control**.

Key Capabilities:
*   **Robust State Estimation:** Tightly-coupled fusion of camera and IMU data.
*   **Dense Mapping:** Real-time 3D voxel mapping using Octomap.
*   **Reactive Planning:** Dynamic obstacle avoidance in complex clutter.


---

## 2. Video demonstration for the autonomous UAV system


The following footage provides a visual verification of the system's core competencies, ranging from initial stability tests to complex autonomous missions.

https://github.com/user-attachments/assets/c19eedae-692c-4f87-8f63-48b1215a5443

*If the video above does not play, please [click here to view the raw file](https://github.com/user-attachments/assets/c19eedae-692c-4f87-8f63-48b1215a5443).*

---

## 3. Key Capabilities

The demonstration highlights four critical pillars of the system's performance:

### a. Precision Attitude Control
*   **Core Competency:** Maintains exceptional stability under varying flight conditions.
*   **Performance:** Rapid convergence to target orientation with minimal overshoot, ensuring a stable platform for onboard sensors.

### b. Dynamic Trajectory Tracking
*   **Core Competency:** Executes complex flight paths with high fidelity.
*   **Performance:** Real-time adjustments to control inputs allow the UAV to follow pre-planned or dynamically generated paths with centimeter-level precision.

### 3. Robust Obstacle Avoidance
*   **Core Competency:** Detects and navigates around static and dynamic obstacles.
*   **Performance:** Operates effectively in cluttered environments, utilizing sensor fusion to calculate collision-free corridors instantly.

### c. Real-Time SLAM (Simultaneous Localization and Mapping)
*   **Core Competency:** Constructs detailed maps while tracking position.
*   **Performance:** Generates accurate environmental models on-the-fly, facilitating fully autonomous exploration in GPS-denied or unknown environments.




---

## 4. System Architecture & Logic Flow

The software follows a multi-threaded architecture ensuring thread safety between perception (high latency) and control loops (real-time hard constraints).

```mermaid  
graph TD
    subgraph "Sensing & State Estimation"
    A["RealSense D435i"] -->|"Images + IMU"| B("VINS-Fusion Node")
    B -->|"Odometry (Pose + Twist)"| C["State Estimator"]
    A -->|"Depth Cloud"| D("PCL Filter / Raycast")
    D --> E["Octomap Server"]
    end

    subgraph "Decision & Planning"
    E -->|"Occupancy Grid"| F("A* Planner Node")
    C -->|"Current Pose"| F
    F -->|"Trajectory Points"| G["Finite State Machine (FSM)"]
    end

    subgraph "Control Layer (Real-time)"
    G -->|"Target Setpoint"| H("Geometric Controller")
    C -->|"Feedback"| H
    H -->|"Attitude/Thrust"| I["MAVROS Bridge"]
    I -->|"UART/Serial"| J["PX4 Flight Controller"]
    end

    subgraph "Embedded Implementation"
    K["Thread: SLAM"] -.->|"Mutex Lock"| L["Shared Map Data"]
    M["Thread: Control"] -.->|"Mutex Lock"| N["Shared State Data"]
    end

