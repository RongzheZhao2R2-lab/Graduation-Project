# Autonomous Quadrotor: Visual SLAM, Planning & Control Framework

[![ROS Noetic](https://img.shields.io/badge/ROS-Noetic-blue)](http://wiki.ros.org/noetic)
[![PX4 Autopilot](https://img.shields.io/badge/PX4-Autopilot-black)](https://px4.io/)
[![VINS-Fusion](https://img.shields.io/badge/SLAM-VINS%20Fusion-purple)](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion)
[![OpenCV](https://img.shields.io/badge/Computer_Vision-OpenCV%204.0-green)](https://opencv.org/)
[![Language](https://img.shields.io/badge/C++-14-orange)](https://isocpp.org/)

**Developer:** Rongzhe Zhao  
**Architecture:** Distributed Embedded System (Jetson Xavier NX + Pixhawk 4)

---

## 1. System Abstract
This project implements a full-stack autonomy framework for a quadrotor UAV, capable of operating in GPS-denied environments. The system leverages **VINS-Fusion** for high-precision Visual-Inertial Odometry (VIO), fusing data from a RealSense D435i (Stereo + IMU). It features a tightly coupled architecture integrating **3D Occupancy Grid Mapping**, **Kinodynamic A_star Planning**, and **Geometric Tracking Control**.

Key Capabilities:
*   **Robust State Estimation:** Tightly-coupled fusion of camera and IMU data.
*   **Dense Mapping:** Real-time 3D voxel mapping using Octomap.
*   **Reactive Planning:** Dynamic obstacle avoidance in complex clutter.

---

## 2. System Architecture & Logic Flow

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

