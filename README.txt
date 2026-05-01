# Robotics project - paper 3: NMPC-Based Local Path Planning for UAV Navigation in Dynamic Environments

**Institution:** Politecnico di Torino - Master Degree in Mechatronic Engineering

**Authors:**
- Davide Alban - 344963
- Davide De Bortoli - 344975
- Leonardo Urbani - 344982

## Folder Contents
- ROS2 workspace with code developed
- Instructions to run the code
- PowerPoint presentation
- Report
- Video

## Overview
This repository contains a robotics project aimed at reproducing the results of the paper regarding Nonlinear MPC for Collision Avoidance and Control of UAVs With Dynamic Obstacles. The project implements a Non-linear Model Predictive Control (NMPC) architecture that allows Unmanned Aerial Vehicles (UAVs) to dynamically avoid collisions with moving obstacles (projectiles) that intersect their trajectory.

## Control Architecture
* **System Model:** The UAV is described as an 8-state non-linear system (position, velocity, roll, and pitch) in a yaw-compensated global reference frame. The control input vector consists of roll, pitch, and total thrust.
* **NMPC Formulation:** The model is discretized using the forward Euler method, utilizing a prediction horizon of N=40 steps. The optimal control inputs are computed by minimizing a cost function that penalizes deviations from the reference states and inputs, as well as aggressive input rates.
* **Collision Avoidance:** Obstacle avoidance is enforced using a penalty method. A constraint function ensures the drone maintains a "safety radius" around the spherical obstacle, applying a penalty term that adapts based on the predicted trajectory of the object.
* **Trajectory Prediction:** The obstacle's future positions are predicted by comparing its last 5 position and velocity measurements and fitting them to the most likely kinematic model (static, linear, or parabolic).
* **Solver:** The optimization problem is solved in real-time using the PANOC non-convex solver through the OpEn software framework.

## Implementation Details
* **Environment:** Initially prototyped in Python, the NMPC was subsequently deployed in the Gazebo simulator to evaluate the controller under more realistic physical conditions.
* **ROS 2 Integration:** The entire system is built on ROS 2. A single node was developed to handle the control logic for both the drone and the obstacle.
* **Drone Model:** We utilized a simplified quadcopter model (available at [simple_quadcopter_gazebo](https://github.com/PIC4SeR/simple_quadcopter_gazebo)) which is controlled by publishing linear velocity commands to the `/cmd_vel` topic. The obstacle is represented by a spherical object controlled via `/cmd_vel_sphere`.
* **State Estimation:** For this simulation setup, real-time positions and velocities are retrieved directly by subscribing to the `/odom` and `/odom_sphere` topics.

## Inside ROS2 Workspace
* **Controller package:** `Robotics_project_delivery/ros2_quadcopter_ws/src/controller`
* **Server, Solver generator, Main node (`open_node`):** `Robotics_project_delivery/ros2_quadcopter_ws/src/controller/controller_pkg/controller_pkg`
* **Gazebo package:** `Robotics_project_delivery/ros2_quadcopter_ws/src/simple_quadcopter_gazebo`

## Results and Limitations
Simulations demonstrate that the NMPC successfully and consistently generates smooth, collision-free paths around moving obstacles in various configurations. However, performance degrades when the obstacle moves at very high velocities; in such extreme cases, the drone may fail to react in time due to its physical response limitations and latency.

## Future Works
* Integration of onboard sensors (e.g., cameras) for real-time obstacle detection to account for measurement noise and real-world perception uncertainties.
* Expansion of the NMPC constraints and parameter tuning to support multiple-object avoidance.

---

## How to Run (ROS 2 Instructions)

**1. Enter the workspace:**
```bash
cd ~/ros2_quadcopter_ws
```

**2. Build the workspace:**
In one terminal, compile the packages:
```bash
colcon build --symlink-install 
```

**3. Compile the solver:**
Before running the nodes, generate the OpEn solver:
```bash
cd ~/ros2_quadcopter_ws/src/controller/controller_pkg/controller_pkg/
python3 solver_generator_ros.py
```

**4. Source the environment:**
In *each* new terminal you open, remember to source the setup file:
```bash
source install/setup.bash
```

**5. Launch the Simulation:**
Open three separate terminals and run the following commands.

*Terminal 1 - Launch Gazebo:*
```bash
cd ~/ros2_quadcopter_ws
ros2 launch simple_quad simple.launch.py
```

*Terminal 2 - Start the Server:*
```bash
cd ~/ros2_quadcopter_ws/src/controller/controller_pkg/controller_pkg/
python3 server.py
```

*Terminal 3 - Start the NMPC Node:*
```bash
cd ~/ros2_quadcopter_ws/src/controller/controller_pkg/controller_pkg/
ros2 run controller_pkg open_node 
```