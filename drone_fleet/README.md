# ROS2 Drone Fleet Dashboard Backend

A ROS2-based backend system for managing and navigating a fleet of drones in a simulated NYC Midtown Manhattan environment.

## Features

- **Multi-drone Fleet Management**: Coordinate 1-2 drones simultaneously
- **3D Path Planning**: Navigate around NYC buildings using Nav2
- **LiDAR-based Obstacle Avoidance**: Real-time obstacle detection using Velodyne VLP-16
- **NYC Environment Simulation**: Gazebo simulation with Midtown Manhattan buildings
- **Web API**: ROSbridge WebSocket interface for frontend integration
- **Mission Planning**: Predefined missions for touring, delivery, and patrol routes

## System Architecture

```
Gazebo Simulation → Drone Sensors → Nav2 Planning → Path Execution → RViz Visualization
                                                                    ↓
                                                            ROSbridge → Frontend
```

## Prerequisites

- macOS with M4 chip (or any Apple Silicon)
- Docker Desktop
- XQuartz (for GUI forwarding)
- 16GB+ RAM
- 10GB+ free disk space

## Quick Start

1. **Clone and Setup**
   ```bash
   cd /workspace/drone_fleet
   chmod +x setup.sh
   ./setup.sh
   ```

2. **Start Docker Container**
   ```bash
   docker-compose run --rm ros2-drone-fleet
   ```

3. **Build Workspace** (inside container)
   ```bash
   cd /workspace
   colcon build --symlink-install
   source install/setup.bash
   ```

4. **Launch Simulation**
   ```bash
   ros2 launch drone_fleet_core drone_fleet_launch.py
   ```

## Project Structure

```
drone_fleet/
├── src/
│   ├── drone_fleet_core/         # Core C++ nodes
│   │   ├── src/
│   │   │   └── fleet_manager.cpp # Fleet coordination
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   ├── drone_fleet_navigation/   # Python navigation nodes
│   │   ├── drone_fleet_navigation/
│   │   │   ├── __init__.py
│   │   │   └── mission_planner.py
│   │   ├── setup.py
│   │   └── package.xml
├── config/
│   └── nav2_params.yaml         # Nav2 configuration
├── launch/
│   └── drone_fleet_launch.py    # Main launch file
├── models/
│   └── quadrotor_lidar.urdf.xacro # Drone model with LiDAR
├── worlds/
│   └── nyc_midtown.world         # NYC Gazebo world
├── docker-compose.yml
├── Dockerfile
└── setup.sh
```

## Key Components

### Fleet Manager (C++)
- Monitors drone states (position, battery, status)
- Publishes fleet status as JSON to `/fleet/status`
- Manages emergency stops and safety systems

### Mission Planner (Python)
- Handles waypoint navigation missions
- Predefined NYC locations (Times Square, Empire State, etc.)
- Publishes mission updates to `/mission/status`

### Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/drone1/lidar/points` | PointCloud2 | LiDAR point cloud data |
| `/drone1/odom` | Odometry | Drone odometry |
| `/drone1/cmd_vel` | Twist | Velocity commands |
| `/fleet/status` | String (JSON) | Fleet status updates |
| `/mission/request` | String (JSON) | Mission requests |
| `/mission/status` | String (JSON) | Mission status updates |

### Services

- Navigation services (Nav2)
- Emergency stop (to be implemented)
- Mission assignment (to be implemented)

## Mission Examples

Send a mission request:
```bash
ros2 topic pub -1 /mission/request std_msgs/String '{data: "{\"drone\": \"drone1\", \"mission\": \"tour_midtown\"}"}'
```

Available missions:
- `tour_midtown`: Visit major landmarks
- `delivery_route`: Times Square → Empire State → Times Square
- `patrol_route`: Circular patrol pattern

## Development

### Adding New Nodes

1. **C++ Node**: Add to `src/drone_fleet_core/src/` and update `CMakeLists.txt`
2. **Python Node**: Add to `src/drone_fleet_navigation/drone_fleet_navigation/` and update `setup.py`

### Extending Missions

Edit `mission_planner.py` to add new waypoints or mission types:
```python
self.nyc_waypoints['new_location'] = {'x': 250.0, 'y': 200.0, 'z': 25.0}
self.demo_missions['new_mission'] = ['location1', 'location2', ...]
```

## Troubleshooting

### XQuartz Issues
```bash
# Reset XQuartz permissions
xhost +local:docker
defaults write org.xquartz.X11 enable_iglx -bool true
```

### Build Errors
```bash
# Clean and rebuild
cd /workspace
rm -rf build install log
colcon build --symlink-install
```

### Performance Issues
- Reduce LiDAR samples in URDF
- Lower Nav2 update frequencies in config
- Use fewer particles in AMCL

## Next Steps

1. **NYC Data Integration**
   - Download and convert 3DCityDB models
   - Process NYC LiDAR data
   - Generate accurate occupancy maps

2. **Advanced Features**
   - Multi-drone collision avoidance
   - Dynamic obstacle handling
   - Battery management system
   - Weather simulation effects

3. **Frontend Integration**
   - Connect React dashboard via ROSbridge
   - Real-time 3D visualization
   - Mission control interface

## Resources

- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Nav2 Documentation](https://navigation.ros.org/)
- [Gazebo Tutorials](http://gazebosim.org/tutorials)
- [3DCityDB NYC Data](https://www.3dcitydb.org/3dcitydb/3dwebclientdemo/nycopen/)

## License

Apache 2.0