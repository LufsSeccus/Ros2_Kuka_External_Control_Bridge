# ROS2 KUKA External Control Bridge

A lightweight ROS 2 bridge for controlling a **KUKA KMR iiwa 14 R820** running **Sunrise.OS** using the **KUKA RoboticsAPI** over UDP.

This project implements a custom communication layer between ROS 2 and a Sunrise Java application, allowing ROS 2 nodes to command the robot through the RoboticsAPI.

> **Project Status:** 🚧 Under Development

---

# Features

- ROS 2 ↔ Sunrise.OS UDP communication
- Velocity control
- Pose control
- Two-way communication
- Easy to extend command interface
- Designed for KUKA KMR platforms

---

# System Architecture

```
                ROS2
                  │
                  │
          UDP Ethernet
                  │
                  ▼
        UDP_bridge.java
      (Sunrise RoboticsAPI)
                  │
          KUKA RoboticsAPI
                  │
             KUKA KMR iiwa
```

---

# Requirements

## Robot Side

- KUKA KMR iiwa 14 R820
- Sunrise.OS
- Sunrise.Workbench
- RoboticsAPI

## PC Side

- Ubuntu (WSL or Native)
- ROS 2 Jazzy
- Ethernet connection to the robot

---

# Installation

Clone the repository

```bash
git clone https://github.com/LufsSeccus/Ros2_Kuka_External_Control_Bridge.git
```

Build the workspace

```bash
cd ~/kuka_ws

colcon build

source install/setup.bash
```

---

# Robot (Sunrise) Setup

## 1. Deploy the Sunrise application

Open the project in Sunrise.Workbench.

Synchronize (Deploy) the project to the Sunrise Cabinet.

---

## 2. Configure Ethernet

Configure the robot and client PC so they are on the same subnet.

Example:

Robot

```
172.32.1.10
```

PC

```
172.32.1.67
```

> **Note**
>
> The PC IP address can be any valid address in the subnet except the robot's address.

---

## 3. Switch the robot to AUT mode

After deploying,

- Switch the SmartPAD to **AUT**
- Start the **UDP_bridge** Sunrise application

Once running, the robot will wait for incoming ROS 2 commands.

---

# ROS2 Setup

Open a terminal.

Source your workspace

```bash
source /opt/ros/jazzy/setup.bash
source ~/kuka_ws/install/setup.bash
```

Run the bridge

```bash
ros2 run kuka_udp_bridge_node udp_bridge_node \
    --ros-args \
    -p robot_id:=<robot_id>
```

Replace

```
<robot_id>
```

with the robot ID shown on the SmartPAD.

---

# Testing

Open another terminal

```bash
source ~/kuka_ws/install/setup.bash
```

Publish a command

```bash
ros2 topic pub /arm_cmd ...
```

If the connection has been established successfully, the LBR should begin executing the received command.

---

# Notes

This project has currently only been tested on

- KUKA KMR iiwa 14 R820
- Sunrise.OS

Users may need to modify

- `robot_id`
- Robot IP address
- Robot UDP port

depending on their robot configuration.

The default communication port can be changed inside the source code if necessary.

---

# Current Development

The project is still under active development.

Planned features include

- KMP.move() implementation
- Set_Arm_Joint_Deg
- Set_Arm_Joint_Rad
- Set_Arm_World
- Improved feedback interface
- Additional RoboticsAPI commands
- Safety watchdog
- Better error handling

---

# Repository Structure

```
Ros2_Kuka_External_Control_Bridge

├── src/
│   ├── kuka_udp_bridge_node
│   └── UDP_bridge.java
│
├── docs/
│
└── README.md
```

---

# Disclaimer

This project is an unofficial research project and is not affiliated with or endorsed by KUKA.

Use at your own risk. Always ensure the robot is operated in a safe environment and follow all KUKA safety procedures before enabling motion.

---

# Future Work

- Full KMP support
- Joint state feedback
- Multi-robot support
