Project Overview

Objective: Build a ROS2-based backend for a drone fleet dashboard that demonstrates dynamic path planning with obstacle avoidance in a 3D NYC grid, simulating 1–2 drones navigating Midtown Manhattan. Visualize drone paths and LiDAR data in RViz, with an optional React web frontend for extra polish.

Scope: Focus on Nav2 for path planning, Gazebo for simulation, RViz for visualization, and a small NYC grid (1km x 1km around 34th St). Use standard ROS2 drone models and NYC’s open 3D building data.

Tools:

ROS2 Humble Hawksbill: Backend for navigation and control.

Nav2: Dynamic path planning and obstacle avoidance.

Gazebo Harmonic: 3D simulation of drones and NYC grid.

RViz: Visualization of drone paths and LiDAR point clouds.

PCL (Point Cloud Library): Process LiDAR data for obstacle detection.

ROSbridge (optional): Stream ROS2 data to a web frontend.

React + Three.js (optional): Web-based 3D visualization.

Docker: Run ROS2 and tools on macOS.

3DCityDB: Source NYC 3D building models.

Hardware: M4 MacBook Pro (16GB RAM, 512GB SSD, macOS Sonoma).

Timeline: Hackathon-ready in 1–2 weeks, assuming 10–20 hours of work.

Step-by-Step Implementation Plan

Step 1: Set Up Your M4 MacBook Pro for ROS2

Since ROS2 Humble is optimized for Ubuntu 22.04, we’ll use Docker to run it on macOS, which is the fastest and most reliable approach for your M4. This avoids the complexity of native builds or VMs like Parallels.

Tools Needed

Docker Desktop: Runs ROS2, Gazebo, and RViz in a container.

Visual Studio Code: For editing ROS2 nodes and configs.

Terminal: macOS Terminal or iTerm2 for Docker commands.

Steps

Install Docker Desktop:

Download from docker.com.

Install and enable Rosetta 2 for ARM compatibility: softwareupdate --install-rosetta.

Verify: docker --version (should output Docker version 24.x or later).

Pull ROS2 Humble Image:

Run: docker pull osrf/ros:humble-desktop

This includes ROS2 Humble, Gazebo Harmonic, RViz, and Nav2.

Set Up Docker for GUI Apps:

Install XQuartz for X11 forwarding: brew install xquartz (install Homebrew first if needed: /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)").

Configure Docker to allow GUI apps:

xhost +local:docker
docker run -it --rm -e DISPLAY=host.docker.internal:0 -v /tmp/.X11-unix:/tmp/.X11-unix osrf/ros:humble-desktop
Create a Workspace:

On your Mac: mkdir -p ~/ros2_ws/src

Mount it in Docker: docker run -it --rm -v ~/ros2_ws:/ws -e DISPLAY=host.docker.internal:0 osrf/ros:humble-desktop

Inside the container, initialize: cd /ws && colcon build

Install VS Code:

Download from code.visualstudio.com.

Install Docker and ROS extensions for easier development.

Resources

Docker Desktop for Mac

ROS2 Docker Tutorial

Running GUI Apps in Docker on macOS

Notes

If you prefer a VM, install Parallels Desktop ($99 or trial) and Ubuntu 22.04, allocating 8GB RAM and 4 cores. Install ROS2 Humble natively: sudo apt install ros-humble-desktop.

Your M4’s 16GB RAM is sufficient for Docker with 1–2 drones. If you have 24GB or 32GB, you can scale up the simulation later.

Step 2: Acquire NYC 3D Building Data

We’ll use the 3DCityDB NYC CityGML model for 3D building geometries and NYC Open Data’s LiDAR point cloud for realistic LiDAR simulation.

Sources

3DCityDB NYC CityGML Model:

Description: Open-source dataset with LoD2 building models for all NYC boroughs, including heights and footprints.

Location: 3DCityDB GitHub or TUM Geoinformatics.

File Size: ~2GB for Manhattan (subset to ~500MB for Midtown).

Format: CityGML (convert to COLLADA for Gazebo).

NYC Open Data LiDAR:

Description: 2014 post-Sandy LiDAR point cloud with building and terrain data.

Location: NYC Open Data Portal or Discover GIS Data NY.

File Size: ~1GB for Midtown (downsampled to ~200MB).

Format: LAS (convert to PointCloud2 with PCL).

Steps

Download 3DCityDB NYC Model:

Clone: git clone https://github.com/3dcitydb/3dcitydb.git

Navigate to the NYC dataset folder or download the Manhattan CityGML file from TUM’s site.

Extract Midtown Manhattan (34th St to 42nd St, 5th Ave to 8th Ave) using QGIS or 3DCityDB’s Importer/Exporter.

Download NYC LiDAR:

Visit NYC Open Data.

Select tiles covering Midtown (use the map viewer to identify tiles, e.g., Manhattan grid 1000–1200).

Download LAS files (~1GB).

Convert CityGML to COLLADA:

Install 3DCityDB Importer/Exporter: docker run -v ~/ros2_ws:/ws 3dcitydb/3dcitydb-tools.

Run: 3dcitydb-exporter --input nyc_midtown.gml --output nyc_midtown.dae --format collada.

Output is a Gazebo-compatible 3D mesh.

Process LiDAR with PCL:

In Docker: apt install ros-humble-pcl-ros.

Use PCL to downsample: ros2 run pcl_ros las2pcd input.las output.pcd --voxel-size 0.1.

This creates a PointCloud2-compatible file for Gazebo LiDAR simulation.

Resources

3DCityDB Documentation

NYC Open Data LiDAR Guide

PCL ROS Tutorial

Step 3: Select and Configure a Standard ROS2 Drone Model

We’ll use a pre-existing ROS2-compatible drone model to avoid custom design, with minor tweaks for LiDAR and Nav2 integration.

Drone Model

Choice: PX4 SITL (Software-in-the-Loop) Quadcopter

Why: Widely used, ROS2-compatible, includes LiDAR and GPS plugins, and integrates with Gazebo and Nav2.

Source: PX4-Autopilot GitHub

Modifications: Add a Velodyne VLP-16 LiDAR plugin and configure for Nav2.

Steps

Clone PX4 Repository:

In Docker: cd /ws/src && git clone https://github.com/PX4/PX4-Autopilot.git --recursive.

Install Dependencies:

Run: apt install ros-humble-px4-ros2-interface ros-humble-px4-msgs.

Configure Drone URDF:

Copy the default quadcopter URDF: cp PX4-Autopilot/Tools/sitl_gazebo/models/quadrotor/quadrotor.urdf /ws/src/quadrotor.urdf.

Add a LiDAR plugin (Velodyne VLP-16):


  
    /lidar/points
    lidar_link
    0.1
    100.0
    360
    0.5
  


  
    
      
    
  


  base_link
  lidar_link
  

Build: cd /ws && colcon build.

Resources

PX4 ROS2 Integration

Gazebo ROS Sensors

PX4 SITL Tutorial

Step 4: Set Up the Gazebo NYC World

Create a Gazebo world with the NYC Midtown grid and the PX4 drone.

Steps

Create a World File:

In /ws/src, create nyc.world:



  
    
      model://sun
    
    
      model://ground_plane
    
    
      file://nyc_midtown.dae
      0 0 0 0 0 0
    
    
      
        model://quadrotor
        100 100 10 0 0 0
      
    
  

Place nyc_midtown.dae in /ws/src/models/.

Launch Gazebo:

Run: ros2 launch gazebo_ros gazebo.launch.py world:=/ws/src/nyc.world.

Verify: Ensure the drone spawns above the NYC grid (e.g., at UTM coordinates for Times Square).

Resources

Gazebo World Creation

ROS2 Gazebo Launch

Step 5: Configure Nav2 for Path Planning

Set up Nav2 to navigate the drone around NYC buildings using LiDAR.

Steps

Install Nav2:

In Docker: apt install ros-humble-navigation2 ros-humble-nav2-bringup.

Create Nav2 Config:

In /ws/src, create nav2_params.yaml:

planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "nav2_dwb_controller/DWBController"
global_costmap:
  ros__parameters:
    plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
    obstacle_layer:
      observation_sources: scan
      scan:
        data_type: PointCloud2
        topic: /lidar/points
        marking: true
        clearing: true
    static_layer:
      map_topic: /map
local_costmap:
  ros__parameters:
    plugins: ["obstacle_layer", "inflation_layer"]
    obstacle_layer:
      observation_sources: scan
      scan:
        data_type: PointCloud2
        topic: /lidar/points
Generate Occupancy Grid:

Use QGIS to convert nyc_midtown.gml to a 2D .pgm map (buildings as obstacles).

Create a .yaml map file:

image: nyc_midtown.pgm
resolution: 0.5
origin: [0.0, 0.0, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
Launch Nav2:

Run: ros2 launch nav2_bringup navigation_launch.py params_file:=/ws/src/nav2_params.yaml.

Send Goal Pose:

Use RViz to set a goal (e.g., from Times Square to Empire State Building) or a Python node:

import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped

rclpy.init()
navigator = BasicNavigator()
goal_pose = PoseStamped()
goal_pose.header.frame_id = 'map'
goal_pose.pose.position.x = 200.0  # Empire State Building
goal_pose.pose.position.y = 150.0
goal_pose.pose.position.z = 10.0
navigator.goToPose(goal_pose)
rclpy.spin(navigator)
Resources

Nav2 Getting Started

Nav2 Simple Commander

QGIS Map Creation

Step 6: Visualize in RViz

Use RViz to display drone paths, LiDAR point clouds, and the NYC grid.

Steps

Launch RViz:

Run: ros2 run rviz2 rviz2.

Configure Displays:

Add Map (topic: /map).

Add PointCloud2 (topic: /lidar/points).

Add Path (topic: /plan).

Add Pose (topic: /tf, frame: base_link).

Save Config: Save as nyc.rviz for quick loading.

Resources

RViz User Guide

Step 7: (Optional) Build a Simple Web Frontend

If time allows, add a React + Three.js frontend to visualize the drone paths, connected via ROSbridge.

Steps

Install ROSbridge:

In Docker: apt install ros-humble-rosbridge-suite.

Launch: ros2 launch rosbridge_server rosbridge_websocket_launch.xml.

Set Up React Project:

On macOS: npx create-react-app drone-dashboard && cd drone-dashboard && npm install three roslib.

Create 3D Visualization:

Use Three.js to render the NYC mesh (load nyc_midtown.dae).

Use roslibjs to subscribe to /lidar/points and /plan.

Example:

import ROSLIB from 'roslib';
import * as THREE from 'three';

const ros = new ROSLIB.Ros({ url: 'ws://localhost:9090' });
const pathTopic = new ROSLIB.Topic({
  ros: ros,
  name: '/plan',
  messageType: 'nav_msgs/Path'
});
pathTopic.subscribe((message) => {
  // Render path in Three.js
});
Run Locally: npm start.

Resources

roslibjs Tutorial

Three.js Getting Started

Step 8: Test and Debug

Run Full Stack:

Gazebo: ros2 launch gazebo_ros gazebo.launch.py world:=/ws/src/nyc.world.

Nav2: ros2 launch nav2_bringup navigation_launch.py params_file:=/ws/src/nav2_params.yaml.

RViz: ros2 run rviz2 rviz2 -d /ws/src/nyc.rviz.

(Optional) ROSbridge: ros2 launch rosbridge_server rosbridge_websocket_launch.xml.

Test Navigation:

Send a goal pose to navigate from Times Square (x: 100, y: 100, z: 10) to Empire State Building (x: 200, y: 150, z: 10).

Verify the drone avoids buildings using LiDAR data.

Optimize:

Downsample LiDAR if Gazebo lags (e.g., reduce samples in URDF).

Simplify the NYC mesh if rendering slows (use Blender to reduce polygons).

Open-Source Repositories to Start From

Instead of starting from an empty repo, fork these to save time:

PX4-Autopilot:

GitHub

Use the ros2 branch for SITL quadcopter with ROS2 integration.

Nav2 Examples:

GitHub

Copy the nav2_bringup and nav2_simple_commander examples.

ROS2 Gazebo Demos:

GitHub

Use the gazebo_ros_demos for world and sensor configs.

3DCityDB NYC:

GitHub

Fork and extract the NYC dataset.

Online Resources

ROS2 Documentation: docs.ros.org/en/humble

Nav2 Tutorials: navigation.ros.org

Gazebo Tutorials: classic.gazebosim.org

PX4 ROS2 Guide: docs.px4.io/main/en/ros/

3DCityDB Guide: 3dcitydb-docs.readthedocs.io

NYC Open Data: data.cityofnewyork.us

PCL ROS: pointclouds.org

Notes for Your M4 MacBook Pro

Performance: Your M4 (16GB RAM) handles 1–2 drones and a 1km x 1km Midtown grid. If you have 24GB/32GB, you can scale to 3 drones or a larger grid.

Storage: The NYC dataset (~2GB), ROS2 (~5GB), and workspace (~5GB) fit in 512GB SSD.

Docker Tips: Run docker system prune if disk space fills up. Use -v ~/ros2_ws:/ws to persist code outside containers.

Battery: Plug in during development, as Gazebo+Docker drains battery in ~3–4 hours.

Final Checklist

Week 1:

Set up Docker and ROS2 Humble.

Download NYC 3DCityDB and LiDAR data.

Convert to COLLADA and PointCloud2.

Clone PX4 and configure drone URDF.

Week 2:

Build Gazebo world with NYC grid.

Configure Nav2 for path planning.

Set up RViz for visualization.

(Optional) Start React frontend.

Hackathon Day:

Demo 1 drone navigating from Times Square to Empire State Building, avoiding buildings, with LiDAR point clouds in RViz.

