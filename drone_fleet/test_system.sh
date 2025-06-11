#!/bin/bash

echo "🚁 Drone Fleet System Test Script"
echo "================================="
echo ""
echo "This script demonstrates how to test the drone fleet system."
echo ""

# Source ROS2 if not already sourced
if [ -z "$ROS_DISTRO" ]; then
    echo "Sourcing ROS2 Humble..."
    source /opt/ros/humble/setup.bash
fi

# Source workspace if built
if [ -d "/workspace/install" ]; then
    echo "Sourcing workspace..."
    source /workspace/install/setup.bash
fi

echo ""
echo "1. First, launch the simulation in one terminal:"
echo "   ros2 launch drone_fleet_core drone_fleet_launch.py"
echo ""
echo "2. In another terminal, check available topics:"
echo "   ros2 topic list"
echo ""
echo "3. Monitor fleet status:"
echo "   ros2 topic echo /fleet/status"
echo ""
echo "4. Send a test mission (Times Square tour):"
echo "   ros2 topic pub -1 /mission/request std_msgs/String '{data: \"{\\\"drone\\\": \\\"drone1\\\", \\\"mission\\\": \\\"tour_midtown\\\"}\"}'"
echo ""
echo "5. Watch LiDAR data:"
echo "   ros2 topic hz /drone1/lidar/points"
echo ""
echo "6. Send drone to specific location:"
echo "   ros2 topic pub -1 /drone1/goal_pose geometry_msgs/PoseStamped '{header: {frame_id: \"map\"}, pose: {position: {x: 200.0, y: 150.0, z: 20.0}, orientation: {w: 1.0}}}'"
echo ""
echo "7. Emergency stop all drones:"
echo "   ros2 topic pub -1 /emergency_stop std_msgs/Bool '{data: true}'"
echo ""

# Function to run a demo
demo_mission() {
    echo "Running demo mission..."
    echo "Sending drone1 on a tour of Midtown..."
    
    ros2 topic pub -1 /mission/request std_msgs/String \
        '{data: "{\"drone\": \"drone1\", \"mission\": \"tour_midtown\"}"}'
    
    echo "Mission sent! Monitor progress with:"
    echo "  ros2 topic echo /mission/status"
}

# Check if user wants to run demo
if [ "$1" == "demo" ]; then
    demo_mission
fi