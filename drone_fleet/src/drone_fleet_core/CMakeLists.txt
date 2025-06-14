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

# Fleet Manager executable
add_executable(fleet_manager src/fleet_manager.cpp)
ament_target_dependencies(fleet_manager 
  rclcpp 
  std_msgs 
  geometry_msgs 
  sensor_msgs 
  nav_msgs
  tf2
  tf2_ros
)

# Install executables
install(TARGETS
  fleet_manager
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  ${CMAKE_CURRENT_SOURCE_DIR}/../../launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  ${CMAKE_CURRENT_SOURCE_DIR}/../../config
  DESTINATION share/${PROJECT_NAME}
)

# Install world files
install(DIRECTORY
  ${CMAKE_CURRENT_SOURCE_DIR}/../../worlds
  DESTINATION share/${PROJECT_NAME}
)

# Install model files
install(DIRECTORY
  ${CMAKE_CURRENT_SOURCE_DIR}/../../models
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()