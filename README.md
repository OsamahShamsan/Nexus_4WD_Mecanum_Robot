# NEXUS 4WD Mecanum Robot - ROS 2 Package

This is a ROS 2 Jazzy workspace developed to control and simulate the Nexus 4WD Mecanum robot platform.
 The project includes arduino program and packages for teleoperation (through Terminal or PS4 Controller), odometry, serial communication with an Arduino 328 controller and robot description for visualization and simulation.

<img src="Nexus_4WD_mecanum_wheels.jpg" alt="Nexus 4WD Mecanum Wheels" width="40%">

#### Disclaimer: 
I am only a user of the Nexus 4WD mecanum wheels robot and develop some packages for study purposes. The owners of the external packages joy, teleop_twist_joy, nexus_4wd_mecanum meshes and URDF are listed in the Acknowledgments section below. 

## Project Overview

The Nexus 4WD Mecanum robot is equipped with 4 mecanum wheels, enabling omnidirectional movement. This ROS 2 package provides essential functionalities for:

- **Serial communication** with the onboard Arduino 328 controller.
- **Teleoperation nodes** for remote and manual control.
- **Odometry node** to estimate the robot's position.
- **Robot description** for visualization in RViz and simulation ().
- **Bringup launch file** to start the system.

---

## Workspace Structure Tree
```
.  
└── src/  
    ├── nexus_bringup/  
    │   ├── config/  
    │   │   ├── params.yaml  
    │   │   └── ps4-holonomic.yaml  
    │   ├── launch/  
    │   │   └── nexus_bringup.launch.py  
    │   ├── package.xml  
    │   └── setup.py  
    │  
    ├── nexus_control/  
    │   ├── nexus_control/  
    │   │   ├── nexus_move_node.py  
    │   │   └── nexus_twist_node.py  
    │   ├── package.xml  
    │   └── setup.py  
    │  
    ├── nexus_description/  
    │   ├── launch/  
    │   │   ├── display.launch.py  
    │   │   └── gazebo.launch.py  
    │   ├── meshes/  
    │   │   ├── mecanum_wheel_left.STL  
    │   │   ├── mecanum_wheel_right.STL  
    │   │   ├── nexus_base_link_collision.STL  
    │   │   ├── nexus_base_link.STL  
    │   │   ├── urm04.STL  
    │   │   └── wheel_shaft.STL
    │   ├── rviz/  
    │   │   └── display.rviz  
    │   ├── urdf/  
    │   │   ├── nexus_4wd_mecanum_gazebo.xacro  
    │   │   ├── nexus_4wd_mecanum_ros2_control.xacro  
    │   │   └── nexus_4wd_mecanum.urdf.xacro  
    │   ├── package.xml
    │   └── setup.py
    │  
    ├── nexus_odom/  
    │   ├── nexus_odom/  
    │   │   └── nexus_odom_node  
    │   ├── package.xml  
    │   └── setup.py  
    │  
    └── nexus_serial_conn/  
        ├── nexus_serial_conn/  
        │   └── nexus_serial_conn_node  
        ├── package.xml  
        └── setup.py   
```

---

## System Architecture

The system consists of multiple ROS 2 nodes communicating via topics, as illustrated in the following graph:

![Node and Topic Graph](rqt_graph_nodes_topics.png)

---

## Nodes

| Node                  | Description                                                                                                    |
|-----------------------|----------------------------------------------------------------------------------------------------------------|
| `/joy_node`           | (External) Reads the inputs from PS4 Controller and map them                                                   |
| `/teleop_node`        | (External) Sends command velocities from PS4 Controller      to nexus_twist_node                               |
| `/nexus_move_node`    |            Sends command velocities from terminal (optional) to nexus_twist_node                               |
| `/nexus_twist_node`   |            Converts `/cmd_vel` to scaled and ready to be sent `/twist_nexus` commands (No prioritizing yet)    |
| `/nexus_serial_conn`  |            Handles serial data to and from Arduino (only recieve & publish data)                               |
| `/nexus_odom`         |            Recieves wheel velocities (from 4 encoders ), computes and publish odom data                        |
| `/nexus_bringup`      |            Launch file for starting the system                                                                 |

---

## Topics

| Topic                         | Message Type                           | Description                                           |
|-------------------------------|----------------------------------------|-------------------------------------------------------|
| `/joy`                        | `sensor_msgs/Joy`                      | Joystick inputs                                       |
| `/joy/set_feedback`           | `sensor_msgs/JoyFeedbackArray`         | Feedback to joystick                                  |
| `/cmd_vel`                    | `geometry_msgs/Twist`                  | Standard robot velocity commands                      |
| `/twist_nexus`                | `std_msgs/Int32MultiArray`             | Nexus-specific velocity commands                      |
| `/encoder_wheels`             | `std_msgs/Int32MultiArray`             | Wheels encoder raw data                               |
| `/serial_debug`               | `std_msgs/String`                      | Debug messages from serial connection (internal use)  |
| `/odom`                       | `nav_msgs/Odometry`                    | Robot odometry                                        |
| `/odom_degree`                | `std_msgs/Float32MultiArray`           | Robot only x, y and yaw/heading in degrees            |
| `/tf`                         | `tf2_msgs/TFMessage`                   | TF transforms                                         |

---


## How to Use

1. **Create the workspace:**
   ```bash
   cd
   mkdir Nexus_4WD_Mecanum_Robot
   cd Nexus_4WD_Mecanum_Robot
   git clone https://github.com/OsamahShamsan/Nexus_4WD_Mecanum_Robot.git
2. **Build the workspace:**
   ```bash
   colcon build
3. **Source the workspace:**
    ```bash
    source /opt/ros/jazzy/setup.bash
    source ~/nexus_4wd_mecanum_ws/install/setup.bash
4. **Establish the PI <=> arduino USB connection:**
    - Connect the Raspberry PI to the arduino through USB.
    - Make sure they are both connected in '/dev/ttyUSB0', otherwise change in the [nexus_serial_conn_node.py](src/nexus_serial_conn/nexus_serial_conn/nexus_serial_conn_node.py) and [params.yaml.](src/nexus_bringup/config/params.yaml).
5. **Uplaod the arduino sketch to the Arduino board:**
    - Open the arduino code in [sketch_nexus.ino](nexus_arduino/sketch_nexus/sketch_nexus.ino) and upload the code. (You have many options to do this: Arduino-cli, Arduino IDE, Visual Studio Code with the arduino extension, ..etc.).

6. **To move the robot:**
    - **Through PS4 Controller:**  
        Turn on the ps4 controller (if another Joystick e.g. ps3 or xbox => see github-ros2-teleop_twist_joy)

   - **Through Terminal:**
        ```bash
        ros2 run nexus_control nexus_move_node --ros-args   -p vx:=1.0 -p vy:=0.0 -p w:=0.0 -p t:=2.0 -p M:=l
7. **Launch the robot system:**
    ```bash
    ros2 launch nexus_bringup bringup.launch.py
## Requirements:
The ROS2 workspace was tested on the following environment:
- Nexus 4WD mecanum wheel robot including built-in 4 Encoders.
- Arduino 328 & IO Expansion Board.
- Raspberry PI 5 8GB RAM and 64GB SD-Card.
- Ubuntu 24.04. 
- ROS 2 Jazzy.

## Acknowledgments

This project makes use of the following open-source packages and resources:

- [joy](https://index.ros.org/p/joy/) - ROS package for joystick drivers and input handling.
- [teleop_twist_joy](https://index.ros.org/r/teleop_twist_joy/) - ROS package to convert joystick inputs to velocity commands.
- [nexus_4wd_mecanum_simulator by RBinsonB](https://github.com/RBinsonB/nexus_4wd_mecanum_simulator/tree/master) - Reference for Nexus 4WD Mecanum simulation setup and configurations.

Special thanks to the ROS community and the contributors of these projects for their valuable work, which made this project possible.

## License

This project is distributed under the terms of the [GNU General Public License v3.0](https://www.gnu.org/licenses/gpl-3.0.html).

### Notes on components:
- Parts of this project (original code) are originally licensed under the MIT License.
- Due to the inclusion of GPL v3 licensed components (such as `joy`, `teleop_twist_joy`, and `nexus_4wd_mecanum_simulator`), the entire project is distributed under the terms of GPL v3.


## Author:
Osamah Shamsan


