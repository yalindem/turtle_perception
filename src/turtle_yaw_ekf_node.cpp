#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <Eigen/Dense>

using std::placeholders::_1;

class YawEKFNode : public rclcpp::Node
{
public:
  YawEKFNode() : Node("yawEkfNode")
  {
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", rclcpp::SensorDataQoS(),
      std::bind(&YawEKFNode::imuCallback, this, _1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&YawEKFNode::odomCallback, this, _1));

    // Initial state
    x_ = Eigen::Vector2d::Zero();

    // Covariance
    P_ = Eigen::Matrix2d::Identity() * 0.1;

    // Process noise
    Q_ = Eigen::Matrix2d::Identity() * 0.01;

    // Measurement noise
    R_imu_ = Eigen::Matrix<double, 1, 1>::Identity() * 0.05;
    R_odom_ = Eigen::Matrix<double, 1, 1>::Identity() * 0.1;

    last_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  }

private:
  // ---------- Callbacks ----------

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    rclcpp::Time current_time(msg->header.stamp);

    if (last_time_.nanoseconds() == 0) {
      last_time_ = current_time;
      return;
    }

    double dt = (current_time - last_time_).seconds();
    last_time_ = current_time;

    if (dt <= 0.0 || dt > 0.1) return;

    // ---------- Prediction ----------
    double gyro_z = msg->angular_velocity.z;

    x_(0) += x_(1) * dt;
    x_(1) = gyro_z;

    Eigen::Matrix2d F;
    F << 1.0, dt,
         0.0, 1.0;

    P_ = F * P_ * F.transpose() + Q_;

    // ---------- IMU Yaw Measurement ----------
    double imu_yaw = this->quaternionToYaw(msg->orientation);
    updateYawMeasurement(imu_yaw, R_imu_);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Yaw estimate: %.3f rad", x_(0));
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    double odom_yaw = this->quaternionToYaw(msg->pose.pose.orientation);
    updateYawMeasurement(odom_yaw, R_odom_);
  }

  // ---------- EKF Update ----------
  void updateYawMeasurement(double yaw_meas, const Eigen::Matrix<double,1,1>& R)
  {
    Eigen::Matrix<double,1,2> H;
    H << 1.0, 0.0;

    Eigen::Matrix<double,1,1> z;
    z << yaw_meas;

    Eigen::Matrix<double,1,1> y = z - H * x_;
    Eigen::Matrix<double,1,1> S = H * P_ * H.transpose() + R;
    Eigen::Matrix<double,2,1> K = P_ * H.transpose() * S.inverse();

    x_ = x_ + K * y;
    P_ = (Eigen::Matrix2d::Identity() - K * H) * P_;
  }

  double quaternionToYaw(const geometry_msgs::msg::Quaternion& q_msg)
  {
    tf2::Quaternion q(q_msg.x, q_msg.y, q_msg.z, q_msg.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
  }

  // ---------- Members ----------
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  Eigen::Vector2d x_;
  Eigen::Matrix2d P_, Q_;
  Eigen::Matrix<double,1,1> R_imu_, R_odom_;

  rclcpp::Time last_time_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<YawEKFNode>());
  rclcpp::shutdown();
  return 0;
}
