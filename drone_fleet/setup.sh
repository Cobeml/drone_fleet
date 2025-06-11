#!/bin/bash

echo "🚁 Drone Fleet ROS2 Backend Setup"
echo "================================="

# Check if running on macOS
if [[ "$OSTYPE" == "darwin"* ]]; then
    echo "✓ macOS detected"
    
    # Check for Docker
    if ! command -v docker &> /dev/null; then
        echo "❌ Docker not found. Please install Docker Desktop from https://docker.com"
        exit 1
    fi
    echo "✓ Docker installed"
    
    # Check for XQuartz
    if ! command -v xquartz &> /dev/null && ! [ -d /Applications/Utilities/XQuartz.app ]; then
        echo "❌ XQuartz not found. Installing via Homebrew..."
        if ! command -v brew &> /dev/null; then
            echo "Installing Homebrew first..."
            /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
        fi
        brew install --cask xquartz
        echo "⚠️  XQuartz installed. Please logout and login again, then run this script again."
        exit 0
    fi
    echo "✓ XQuartz installed"
    
    # Configure XQuartz for Docker
    echo "Configuring XQuartz for Docker..."
    defaults write org.xquartz.X11 enable_iglx -bool true
    defaults write org.xquartz.X11 nolisten_tcp -bool false
    
    # Start XQuartz if not running
    if ! pgrep -x "XQuartz" > /dev/null; then
        echo "Starting XQuartz..."
        open -a XQuartz
        sleep 2
    fi
    
    # Allow connections from Docker
    xhost +local:docker
    echo "✓ XQuartz configured for Docker"
fi

# Build Docker image
echo ""
echo "Building Docker image..."
docker-compose build

# Create ROS2 package structure
echo ""
echo "Creating ROS2 package structure..."
mkdir -p src/drone_fleet_{core,navigation,simulation,communication}

# Create package.xml for main package
cat > src/drone_fleet_core/package.xml << EOF
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>drone_fleet_core</name>
  <version>0.1.0</version>
  <description>Core functionality for drone fleet management</description>
  <maintainer email="your@email.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  
  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>tf2</depend>
  <depend>tf2_ros</depend>
  
  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
EOF

# Create CMakeLists.txt
cat > src/drone_fleet_core/CMakeLists.txt << EOF
cmake_minimum_required(VERSION 3.8)
project(drone_fleet_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
EOF

echo "✓ ROS2 package structure created"

echo ""
echo "🎉 Setup complete!"
echo ""
echo "To start the development environment:"
echo "  docker-compose run --rm ros2-drone-fleet"
echo ""
echo "Once inside the container, build the workspace with:"
echo "  cd /workspace && colcon build"
echo ""