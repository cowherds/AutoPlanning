#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <quadrotor_msgs/msg/so3_command.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>

#include "so3_control/SO3Control.h"

class NetworkControlRos2 final : public rclcpp::Node {
public:
  NetworkControlRos2() : Node("network_ctrl_node_ros2") {
    is_simulation_ = this->declare_parameter<bool>("is_simulation", true);
    hover_thrust_ = this->declare_parameter<double>("hover_thrust", 0.375);
    kx_xy_ = this->declare_parameter<double>("kx_xy", 5.7);
    kx_z_ = this->declare_parameter<double>("kx_z", 6.2);
    kv_xy_ = this->declare_parameter<double>("kv_xy", 3.4);
    kv_z_ = this->declare_parameter<double>("kv_z", 4.0);

    so3_controller_.setMass(mass_);

    using std::placeholders::_1;
    so3_command_pub_ = this->create_publisher<quadrotor_msgs::msg::SO3Command>("so3_cmd", 10);
    position_cmd_sub_ = this->create_subscription<quadrotor_msgs::msg::PositionCommand>(
        "position_cmd", 10, std::bind(&NetworkControlRos2::positionCmdCallback, this, _1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&NetworkControlRos2::odomCallback, this, _1));
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu", 50, std::bind(&NetworkControlRos2::imuCallback, this, _1));

    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(control_dt_),
        std::bind(&NetworkControlRos2::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "ROS2 network controller is ready.");
  }

private:
  void positionCmdCallback(const quadrotor_msgs::msg::PositionCommand::SharedPtr msg) {
    if (!state_init_) return;
    position_cmd_init_ = true;

    des_pos_ = Eigen::Vector3d(msg->position.x, msg->position.y, msg->position.z);
    des_vel_ = Eigen::Vector3d(msg->velocity.x, msg->velocity.y, msg->velocity.z);
    des_acc_ = Eigen::Vector3d(msg->acceleration.x, msg->acceleration.y, msg->acceleration.z);
    des_yaw_ = msg->yaw;
    des_yaw_dot_ = msg->yaw_dot;

    Eigen::Vector3d att_acc;
    if (msg->trajectory_flag == quadrotor_msgs::msg::PositionCommand::TRAJECTORY_STATUS_READY) {
      att_acc = des_acc_;
      publishSO3FromAcc(att_acc, des_yaw_);
    } else {
      att_acc = publishHoverSO3Command(des_pos_, des_vel_, des_acc_, des_yaw_, des_yaw_dot_);
    }
    last_des_acc_ = att_acc;
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom) {
    tf2::Quaternion q(
        odom->pose.pose.orientation.x, odom->pose.pose.orientation.y,
        odom->pose.pose.orientation.z, odom->pose.pose.orientation.w);
    cur_yaw_ = tf2::getYaw(q);

    cur_pos_ = Eigen::Vector3d(
        odom->pose.pose.position.x, odom->pose.pose.position.y, odom->pose.pose.position.z);
    cur_vel_ = Eigen::Vector3d(
        odom->twist.twist.linear.x, odom->twist.twist.linear.y, odom->twist.twist.linear.z);
    cur_att_.w() = odom->pose.pose.orientation.w;
    cur_att_.x() = odom->pose.pose.orientation.x;
    cur_att_.y() = odom->pose.pose.orientation.y;
    cur_att_.z() = odom->pose.pose.orientation.z;

    so3_controller_.setPosition(cur_pos_);
    so3_controller_.setVelocity(cur_vel_);

    if (!state_init_) {
      state_init_ = true;
      des_pos_ = cur_pos_;
      des_yaw_ = cur_yaw_;
      RCLCPP_INFO(this->get_logger(), "Received odom, controller activated.");
    }
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr imu) {
    Eigen::Vector3d acc(imu->linear_acceleration.x, imu->linear_acceleration.y, imu->linear_acceleration.z);
    if (is_simulation_) {
      cur_acc_ = acc;
    } else {
      Eigen::Vector3d acc_world = cur_att_ * acc;
      acc_world(2) -= one_g_;
      cur_acc_ = acc_world;
    }
  }

  void timerCallback() {
    if (!state_init_ || position_cmd_init_) return;
    publishHoverSO3Command(des_pos_, des_vel_, des_acc_, des_yaw_, des_yaw_dot_);
  }

  Eigen::Vector3d publishHoverSO3Command(
      const Eigen::Vector3d &des_pos, const Eigen::Vector3d &des_vel,
      const Eigen::Vector3d &des_acc, double des_yaw, double des_yaw_dot) {
    (void)des_yaw_dot;
    Eigen::Vector3d kx(kx_xy_, kx_xy_, kx_z_);
    Eigen::Vector3d kv(kv_xy_, kv_xy_, kv_z_);
    so3_controller_.calculateControl(des_pos, des_vel, des_acc, des_yaw, 0.0, kx, kv);

    const auto force = so3_controller_.getComputedForce();
    const auto orientation = so3_controller_.getComputedOrientation();
    publishSO3(force, orientation, cur_yaw_);

    double thrust = force.norm() / mass_;
    Eigen::Matrix3d c_bn;
    c_bn = orientation.toRotationMatrix();
    Eigen::Vector3d att_acc = c_bn * Eigen::Vector3d(0, 0, thrust);
    att_acc(2) -= one_g_;
    return att_acc;
  }

  void publishSO3FromAcc(const Eigen::Vector3d &ref_acc, double ref_yaw) {
    Eigen::Vector3d force = mass_ * one_g_ * Eigen::Vector3d(0, 0, 1) + mass_ * ref_acc;
    Eigen::Vector3d b1d(cos(ref_yaw), sin(ref_yaw), 0);
    Eigen::Vector3d b3c = force.norm() > 1e-6 ? force.normalized() : Eigen::Vector3d(0, 0, 1);
    Eigen::Vector3d b2c = b3c.cross(b1d).normalized();
    Eigen::Vector3d b1c = b2c.cross(b3c).normalized();
    Eigen::Matrix3d rot;
    rot << b1c, b2c, b3c;
    publishSO3(force, Eigen::Quaterniond(rot), cur_yaw_);
  }

  void publishSO3(const Eigen::Vector3d &force, const Eigen::Quaterniond &quat, double cur_yaw) {
    quadrotor_msgs::msg::SO3Command cmd;
    cmd.header.stamp = this->now();
    cmd.force.x = force(0);
    cmd.force.y = force(1);
    cmd.force.z = force(2);
    cmd.orientation.w = quat.w();
    cmd.orientation.x = quat.x();
    cmd.orientation.y = quat.y();
    cmd.orientation.z = quat.z();
    cmd.k_r = {1.5, 1.5, 1.0};
    cmd.k_om = {0.13, 0.13, 0.1};
    cmd.aux.current_yaw = cur_yaw;
    cmd.aux.enable_motors = true;
    cmd.aux.use_external_yaw = false;
    cmd.aux.kf_correction = 0.0;
    cmd.aux.angle_corrections = {0.0, 0.0};
    so3_command_pub_->publish(cmd);
  }

private:
  rclcpp::Publisher<quadrotor_msgs::msg::SO3Command>::SharedPtr so3_command_pub_;
  rclcpp::Subscription<quadrotor_msgs::msg::PositionCommand>::SharedPtr position_cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  SO3Control so3_controller_;

  double mass_{0.98};
  double control_dt_{0.02};
  double hover_thrust_{0.375};
  double kx_xy_{5.7}, kx_z_{6.2}, kv_xy_{3.4}, kv_z_{4.0};
  const double one_g_{9.81};

  bool is_simulation_{true};
  bool state_init_{false};
  bool position_cmd_init_{false};

  double cur_yaw_{0.0};
  Eigen::Vector3d cur_pos_{0, 0, 0};
  Eigen::Vector3d cur_vel_{0, 0, 0};
  Eigen::Vector3d cur_acc_{0, 0, 0};
  Eigen::Quaterniond cur_att_{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d last_des_acc_{0, 0, 0};

  Eigen::Vector3d des_pos_{0, 0, 2.0};
  Eigen::Vector3d des_vel_{0, 0, 0};
  Eigen::Vector3d des_acc_{0, 0, 0};
  double des_yaw_{0.0};
  double des_yaw_dot_{0.0};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NetworkControlRos2>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
