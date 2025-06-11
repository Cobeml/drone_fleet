#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <memory>
#include <string>
#include <vector>
#include <map>

namespace drone_fleet
{

struct DroneState
{
    std::string name;
    geometry_msgs::msg::PoseStamped current_pose;
    double battery_percentage;
    bool is_active;
    std::string current_mission;
};

class FleetManager : public rclcpp::Node
{
public:
    FleetManager() : Node("fleet_manager")
    {
        // Initialize TF2
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        // Parameters
        this->declare_parameter("fleet_size", 2);
        this->declare_parameter("update_rate", 10.0);
        
        fleet_size_ = this->get_parameter("fleet_size").as_int();
        double update_rate = this->get_parameter("update_rate").as_double();
        
        // Initialize fleet
        for (int i = 1; i <= fleet_size_; ++i)
        {
            std::string drone_name = "drone" + std::to_string(i);
            drones_[drone_name] = DroneState{
                drone_name,
                geometry_msgs::msg::PoseStamped(),
                100.0,
                true,
                "idle"
            };
            
            // Subscribe to individual drone topics
            auto pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/" + drone_name + "/pose",
                10,
                [this, drone_name](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                    this->updateDronePose(drone_name, msg);
                });
            pose_subscribers_.push_back(pose_sub);
            
            auto battery_sub = this->create_subscription<sensor_msgs::msg::BatteryState>(
                "/" + drone_name + "/battery_state",
                10,
                [this, drone_name](const sensor_msgs::msg::BatteryState::SharedPtr msg) {
                    this->updateDroneBattery(drone_name, msg);
                });
            battery_subscribers_.push_back(battery_sub);
        }
        
        // Publishers
        fleet_status_pub_ = this->create_publisher<std_msgs::msg::String>("fleet/status", 10);
        
        // Services
        // TODO: Add mission assignment service
        // TODO: Add emergency stop service
        
        // Timer for periodic updates
        update_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / update_rate),
            std::bind(&FleetManager::updateFleetStatus, this));
        
        RCLCPP_INFO(this->get_logger(), "Fleet Manager initialized with %d drones", fleet_size_);
    }
    
private:
    void updateDronePose(const std::string& drone_name, 
                        const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (drones_.find(drone_name) != drones_.end())
        {
            drones_[drone_name].current_pose = *msg;
        }
    }
    
    void updateDroneBattery(const std::string& drone_name,
                           const sensor_msgs::msg::BatteryState::SharedPtr msg)
    {
        if (drones_.find(drone_name) != drones_.end())
        {
            drones_[drone_name].battery_percentage = msg->percentage * 100.0;
            
            // Check for low battery
            if (msg->percentage < 0.2)
            {
                RCLCPP_WARN(this->get_logger(), 
                           "%s battery low: %.1f%%", 
                           drone_name.c_str(), 
                           msg->percentage * 100.0);
            }
        }
    }
    
    void updateFleetStatus()
    {
        // Create status message
        std_msgs::msg::String status_msg;
        std::stringstream ss;
        
        ss << "{\n";
        ss << "  \"timestamp\": " << this->now().seconds() << ",\n";
        ss << "  \"drones\": [\n";
        
        bool first = true;
        for (const auto& [name, state] : drones_)
        {
            if (!first) ss << ",\n";
            first = false;
            
            ss << "    {\n";
            ss << "      \"name\": \"" << name << "\",\n";
            ss << "      \"position\": {\n";
            ss << "        \"x\": " << state.current_pose.pose.position.x << ",\n";
            ss << "        \"y\": " << state.current_pose.pose.position.y << ",\n";
            ss << "        \"z\": " << state.current_pose.pose.position.z << "\n";
            ss << "      },\n";
            ss << "      \"battery\": " << state.battery_percentage << ",\n";
            ss << "      \"active\": " << (state.is_active ? "true" : "false") << ",\n";
            ss << "      \"mission\": \"" << state.current_mission << "\"\n";
            ss << "    }";
        }
        
        ss << "\n  ]\n";
        ss << "}";
        
        status_msg.data = ss.str();
        fleet_status_pub_->publish(status_msg);
    }
    
    // Member variables
    int fleet_size_;
    std::map<std::string, DroneState> drones_;
    
    // TF2
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // Publishers
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fleet_status_pub_;
    
    // Subscribers
    std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> pose_subscribers_;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr> battery_subscribers_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr update_timer_;
};

} // namespace drone_fleet

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<drone_fleet::FleetManager>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}