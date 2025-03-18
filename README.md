# VFF-Avoidance-Ali-Larissa
vff_avoidance is a ROS 2 package that implements an obstacle avoidance strategy for mobile robots using a Virtual Force Field (VFF) algorithm. The node computes attractive and repulsive vectors based on LaserScan data to safely steer the robot away from obstacles while moving forward.
Features

    Virtual Force Field (VFF) Algorithm: Combines attractive (forward) and repulsive (obstacle avoidance) forces.
    Parameterizable: Tune gains, speed limits, and sensor parameters via a YAML configuration file.
    RViz Debug Markers: Visualize attractive, repulsive, and resultant vectors.
    Gazebo Integration: Easily run the node in simulation with TurtleBot3 in Gazebo.

Dependencies

    ROS 2 
    rclcpp
    sensor_msgs
    geometry_msgs
    visualization_msgs
    turtlebot3_gazebo (for simulation)

Installation

    Clone the Repository
    Place the repository in the src directory of your ROS 2 workspace:

cd ~/ros2_ws/src
git clone <repository_url> vff_avoidance

Build the Package
From the root of your workspace:

cd ~/ros2_ws
colcon build --packages-select vff_avoidance

Source the Workspace

    source install/setup.bash

Usage

To run the node in simulation along with Gazebo and TurtleBot3, use the provided launch file:

ros2 launch vff_avoidance avoidance_vff_gazebo.launch.py

Authors:
Ali Rammal 
Larissa Azar
