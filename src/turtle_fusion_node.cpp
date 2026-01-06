#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"

class Position2DEstimator : rclcpp::Node
{
    public:
        Position2DEstimator():Node("Position2DEstimatorNode")
        {

        }   
        
    private:
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Position2DEstimator>());
    rclcpp::shutdown();
}