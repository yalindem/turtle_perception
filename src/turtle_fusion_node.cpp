#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class Position2DEstimator : public rclcpp::Node
{
    public:
        Position2DEstimator() : Node("Position2DEstimatorNode")
        {
            imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("/imu", rclcpp::SensorDataQoS(), std::bind(&Position2DEstimator::imuCallback, this, std::placeholders::_1));
            odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", rclcpp::SensorDataQoS(), std::bind(&Position2DEstimator::odomCallback, this, std::placeholders::_1));
        }
    
    private:
        void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
        {
            double x = msg->orientation.x;
            double y = msg->orientation.y;
            double z = msg->orientation.z;
            double w = msg->orientation.w;

            tf2::Quaternion q(x,y,z,w);

            tf2::Matrix3x3 m(q);
            double roll, yaw, pitch;
            m.getRPY(roll, pitch, yaw);

            double yaw_rate = msg->angular_velocity.z;
            
            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "IMU | yaw: %.3f rad | yaw_rate: %.3f rad/s",
                yaw,
                yaw_rate
            );

        }

        void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
        {
            double x = msg->pose.pose.position.x;
            double y = msg->pose.pose.position.y;

            double v = msg->twist.twist.linear.x;

            RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "ODOM | x: %.3f m | y: %.3f m | v: %.3f m/s",
            x,
            y,
            v
            );
        }

        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Position2DEstimator>());
    rclcpp::shutdown();
}