# Custom DWA Local Planner for ROS2

## Overview
This project implements a custom **Dynamic Window Approach (DWA) local planner** for a **TurtleBot** in **ROS2 Humble**. The planner is written from scratch without using `nav2_dwb_controller` and aims to generate velocity commands (`/cmd_vel`) for obstacle avoidance and goal navigation.

The implementation is based on the concept of the DWA planner, but it is still under development. Future improvements will focus on optimizing trajectory evaluation, improving cost functions, and refining obstacle avoidance behavior.

## Features
- Samples velocity commands within dynamic constraints.
- Predicts trajectories based on sampled velocities.
- Evaluates trajectories using a cost function (goal distance, obstacle avoidance, and smoothness).
- Selects the best trajectory and publishes velocity commands (`/cmd_vel`).
- Subscribes to **Odometry (`/odom`)** and **LaserScan (`/scan`)**.
- Uses **RViz Markers** to visualize sampled trajectories.
- Works in **Gazebo** with obstacles for real-world testing.

## Installation & Setup
### 1. Install ROS2 Humble and TurtleBot3 Simulation
Ensure you have **ROS2 Humble** installed:
```sh
sudo apt update
sudo apt install ros-humble-desktop
```

Install TurtleBot3 and Gazebo simulation:
```sh
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git -b humble-devel

git clone https://github.com/ROBOTIS-GIT/turtlebot3.git -b humble-devel

git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git -b humble-devel

git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git -b humble-devel
```

### 2. Clone the Repository
Navigate to your ROS2 workspace and clone the repository:
```sh
cd ~/ros2_ws/src
git clone https://github.com/abdu7rahman/Custom-DWA-Local-Planner.git
```

### 3. Build the Package
```sh
cd ~/ros2_ws
colcon build --packages-select custom_dwa_planner
source install/setup.bash
```

### 4. Launch the Simulation
Start the Gazebo environment with TurtleBot3:
```sh
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

Run the custom DWA local planner:
```sh
ros2 run custom_dwa_planner dwa_planner
```

## Usage
- The robot will attempt to navigate to a goal while avoiding obstacles.
- Debugging logs will show velocity sampling, trajectory scores, and decision-making steps.
- **RViz** will display sampled trajectories and the selected path.
- The code prompts for user input, and one of the only few integer values of **XY coordinates** that works is **(2,1)**, so it is highly recommended to use that.
- **RViz Visualization:**
  - Visualize planned trajectories using **RViz Markers**.
  - Launch RViz by running:
    ```sh
    rviz2
    ```
  - In **RViz**, click **Add → By Topic** and select **visual_paths** to view the planned trajectories.


## Known Issues & Future Improvements
- The planner does not yet guarantee smooth navigation in all scenarios.
- Obstacle avoidance needs further optimization.
- Tuning of cost function parameters is ongoing.
