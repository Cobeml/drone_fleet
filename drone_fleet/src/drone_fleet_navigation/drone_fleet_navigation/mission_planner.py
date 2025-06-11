#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import String
from nav2_simple_commander.robot_navigator import BasicNavigator
import json
import threading

class MissionPlanner(Node):
    def __init__(self):
        super().__init__('mission_planner')
        
        # Parameters
        self.declare_parameter('mission_file', '')
        self.declare_parameter('loop_missions', False)
        
        # Initialize navigators for each drone
        self.navigators = {}
        self.current_missions = {}
        
        # Publishers
        self.mission_status_pub = self.create_publisher(
            String, 'mission/status', 10)
        
        # Subscribers
        self.mission_request_sub = self.create_subscription(
            String, 'mission/request', self.handle_mission_request, 10)
        
        # Predefined waypoints for NYC demo
        self.nyc_waypoints = {
            'times_square': {'x': 100.0, 'y': 100.0, 'z': 15.0},
            'empire_state': {'x': 200.0, 'y': 150.0, 'z': 20.0},
            'madison_square': {'x': 150.0, 'y': 100.0, 'z': 15.0},
            'bryant_park': {'x': 180.0, 'y': 120.0, 'z': 18.0},
            'herald_square': {'x': 120.0, 'y': 80.0, 'z': 15.0}
        }
        
        # Demo missions
        self.demo_missions = {
            'tour_midtown': [
                'times_square', 'herald_square', 'madison_square', 
                'bryant_park', 'empire_state'
            ],
            'delivery_route': [
                'times_square', 'empire_state', 'times_square'
            ],
            'patrol_route': [
                'madison_square', 'bryant_park', 'herald_square', 'madison_square'
            ]
        }
        
        self.get_logger().info('Mission Planner initialized')
        
    def handle_mission_request(self, msg):
        """Handle incoming mission requests"""
        try:
            request = json.loads(msg.data)
            drone_name = request.get('drone', 'drone1')
            mission_type = request.get('mission', 'tour_midtown')
            
            if mission_type in self.demo_missions:
                self.execute_mission(drone_name, mission_type)
            else:
                self.get_logger().warn(f'Unknown mission type: {mission_type}')
                
        except json.JSONDecodeError:
            self.get_logger().error('Invalid mission request format')
    
    def execute_mission(self, drone_name, mission_type):
        """Execute a predefined mission for a specific drone"""
        if drone_name not in self.navigators:
            self.navigators[drone_name] = BasicNavigator(namespace=drone_name)
        
        navigator = self.navigators[drone_name]
        waypoint_names = self.demo_missions[mission_type]
        
        # Create path from waypoints
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        
        for waypoint_name in waypoint_names:
            if waypoint_name in self.nyc_waypoints:
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.header.stamp = path.header.stamp
                
                waypoint = self.nyc_waypoints[waypoint_name]
                pose.pose.position.x = waypoint['x']
                pose.pose.position.y = waypoint['y']
                pose.pose.position.z = waypoint['z']
                
                # Default orientation (facing forward)
                pose.pose.orientation.w = 1.0
                
                path.poses.append(pose)
        
        # Start navigation in a separate thread
        nav_thread = threading.Thread(
            target=self._navigate_through_poses,
            args=(navigator, drone_name, mission_type, path.poses)
        )
        nav_thread.start()
        
        # Update mission status
        self.current_missions[drone_name] = mission_type
        self.publish_mission_status(drone_name, 'started', mission_type)
    
    def _navigate_through_poses(self, navigator, drone_name, mission_type, poses):
        """Navigate through a series of poses"""
        self.get_logger().info(
            f'{drone_name} starting mission: {mission_type} with {len(poses)} waypoints')
        
        # Wait for Nav2 to be ready
        navigator.waitUntilNav2Active()
        
        # Navigate through poses
        navigator.goThroughPoses(poses)
        
        # Monitor progress
        i = 0
        while not navigator.isTaskComplete():
            # Get feedback
            i += 1
            feedback = navigator.getFeedback()
            
            if feedback and i % 20 == 0:  # Log every 2 seconds at 10Hz
                self.get_logger().info(
                    f'{drone_name}: Distance to goal: '
                    f'{feedback.distance_to_goal:.2f}m')
                
            # Check for cancel or error
            if navigator.isTaskComplete():
                break
                
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # Get result
        result = navigator.getResult()
        if result == BasicNavigator.TaskResult.SUCCEEDED:
            self.get_logger().info(f'{drone_name} completed mission: {mission_type}')
            self.publish_mission_status(drone_name, 'completed', mission_type)
        elif result == BasicNavigator.TaskResult.CANCELED:
            self.get_logger().info(f'{drone_name} mission canceled: {mission_type}')
            self.publish_mission_status(drone_name, 'canceled', mission_type)
        else:
            self.get_logger().error(f'{drone_name} mission failed: {mission_type}')
            self.publish_mission_status(drone_name, 'failed', mission_type)
        
        # Remove from current missions
        if drone_name in self.current_missions:
            del self.current_missions[drone_name]
    
    def publish_mission_status(self, drone_name, status, mission_type):
        """Publish mission status update"""
        status_msg = String()
        status_data = {
            'timestamp': self.get_clock().now().seconds_nanoseconds()[0],
            'drone': drone_name,
            'mission': mission_type,
            'status': status
        }
        status_msg.data = json.dumps(status_data)
        self.mission_status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    
    mission_planner = MissionPlanner()
    
    try:
        rclpy.spin(mission_planner)
    except KeyboardInterrupt:
        pass
    finally:
        mission_planner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()