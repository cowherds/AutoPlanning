#include <cmath>
#include <string>
#include <array>
#include <memory>

#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <quadrotor_msgs/msg/so3_command.hpp>
#include <quadrotor_simulator/Quadrotor.h>

struct Control {
  double rpm[4]{0.0, 0.0, 0.0, 0.0};
};

struct Command {
  float force[3]{0.0f, 0.0f, 0.0f};
  float qx{0}, qy{0}, qz{0}, qw{1};
  float k_r[3]{1.5f, 1.5f, 1.0f};
  float k_om[3]{0.13f, 0.13f, 0.1f};
  float corrections[3]{0.0f, 0.0f, 0.0f};
  float current_yaw{0.0f};
  bool use_external_yaw{false};
};

struct Disturbance {
  Eigen::Vector3d f{Eigen::Vector3d::Zero()};
  Eigen::Vector3d m{Eigen::Vector3d::Zero()};
};

class QuadrotorSimulatorRos2 final : public rclcpp::Node {
public:
  QuadrotorSimulatorRos2() : Node("quadrotor_simulator_so3_ros2") {
    init_x_ = this->declare_parameter<double>("simulator.init_state_x", 0.0);
    init_y_ = this->declare_parameter<double>("simulator.init_state_y", 0.0);
    init_z_ = this->declare_parameter<double>("simulator.init_state_z", 2.0);
    simulation_rate_ = this->declare_parameter<double>("rate.simulation", 1000.0);
    odom_rate_ = this->declare_parameter<double>("rate.odom", 100.0);
    quad_name_ = this->declare_parameter<std::string>("quadrotor_name", "quadrotor");

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 100);
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
    mesh_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("uav", 1);

    using std::placeholders::_1;
    cmd_sub_ = this->create_subscription<quadrotor_msgs::msg::SO3Command>(
        "cmd", 100, std::bind(&QuadrotorSimulatorRos2::cmdCallback, this, _1));
    f_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
        "force_disturbance", 20, std::bind(&QuadrotorSimulatorRos2::forceDisturbanceCallback, this, _1));
    m_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
        "moment_disturbance", 20, std::bind(&QuadrotorSimulatorRos2::momentDisturbanceCallback, this, _1));

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    mesh_resource_ = "file://" + ament_index_cpp::get_package_share_directory("so3_quadrotor_simulator") + "/config/uav.dae";

    quad_.setStatePos(Eigen::Vector3d(init_x_, init_y_, init_z_));

    dt_ = 1.0 / simulation_rate_;
    odom_pub_interval_ = 1.0 / odom_rate_;
    last_odom_pub_time_ = this->now();

    sim_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(dt_), std::bind(&QuadrotorSimulatorRos2::simLoop, this));

    RCLCPP_INFO(this->get_logger(), "ROS2 quadrotor simulator ready.");
  }

private:
  static void stateToOdomMsg(
      const QuadrotorSimulator::Quadrotor::State &state,
      nav_msgs::msg::Odometry &odom) {
    odom.pose.pose.position.x = state.x(0);
    odom.pose.pose.position.y = state.x(1);
    odom.pose.pose.position.z = state.x(2);

    Eigen::Quaterniond q(state.R);
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    odom.twist.twist.linear.x = state.v(0);
    odom.twist.twist.linear.y = state.v(1);
    odom.twist.twist.linear.z = state.v(2);

    odom.twist.twist.angular.x = state.omega(0);
    odom.twist.twist.angular.y = state.omega(1);
    odom.twist.twist.angular.z = state.omega(2);
  }

  static void quadToImuMsg(const QuadrotorSimulator::Quadrotor &quad, sensor_msgs::msg::Imu &imu) {
    auto state = quad.getState();
    Eigen::Quaterniond q(state.R);
    imu.orientation.x = q.x();
    imu.orientation.y = q.y();
    imu.orientation.z = q.z();
    imu.orientation.w = q.w();
    imu.angular_velocity.x = state.omega(0);
    imu.angular_velocity.y = state.omega(1);
    imu.angular_velocity.z = state.omega(2);
    imu.linear_acceleration.x = quad.getAcc()[0];
    imu.linear_acceleration.y = quad.getAcc()[1];
    imu.linear_acceleration.z = quad.getAcc()[2];
  }

  Control getControl(const QuadrotorSimulator::Quadrotor &quad, const Command &cmd) {
    const double _kf = quad.getPropellerThrustCoefficient();
    const double _km = quad.getPropellerMomentCoefficient();
    const double kf = _kf - cmd.corrections[0];
    const double km = _km / _kf * kf;
    const double d = quad.getArmLength();

    const Eigen::Matrix3f J = quad.getInertia().cast<float>();
    const float I[3][3] = {{J(0, 0), J(0, 1), J(0, 2)}, {J(1, 0), J(1, 1), J(1, 2)}, {J(2, 0), J(2, 1), J(2, 2)}};
    const auto state = quad.getState();

    Eigen::Vector3d ypr;
    ypr[0] = std::atan2(state.R(1, 0), state.R(0, 0));
    ypr[1] = std::asin(-state.R(2, 0));
    ypr[2] = std::atan2(state.R(2, 1), state.R(2, 2));
    if (cmd.use_external_yaw) ypr[0] = cmd.current_yaw;

    Eigen::Matrix3d R;
    R = Eigen::AngleAxisd(ypr[0], Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(ypr[1], Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(ypr[2], Eigen::Vector3d::UnitX());

    const float R11 = R(0, 0), R12 = R(0, 1), R13 = R(0, 2);
    const float R21 = R(1, 0), R22 = R(1, 1), R23 = R(1, 2);
    const float R31 = R(2, 0), R32 = R(2, 1), R33 = R(2, 2);
    const float Om1 = state.omega(0), Om2 = state.omega(1), Om3 = state.omega(2);

    const float Rd11 = cmd.qw * cmd.qw + cmd.qx * cmd.qx - cmd.qy * cmd.qy - cmd.qz * cmd.qz;
    const float Rd12 = 2 * (cmd.qx * cmd.qy - cmd.qw * cmd.qz);
    const float Rd13 = 2 * (cmd.qx * cmd.qz + cmd.qw * cmd.qy);
    const float Rd21 = 2 * (cmd.qx * cmd.qy + cmd.qw * cmd.qz);
    const float Rd22 = cmd.qw * cmd.qw - cmd.qx * cmd.qx + cmd.qy * cmd.qy - cmd.qz * cmd.qz;
    const float Rd23 = 2 * (cmd.qy * cmd.qz - cmd.qw * cmd.qx);
    const float Rd31 = 2 * (cmd.qx * cmd.qz - cmd.qw * cmd.qy);
    const float Rd32 = 2 * (cmd.qy * cmd.qz + cmd.qw * cmd.qx);
    const float Rd33 = cmd.qw * cmd.qw - cmd.qx * cmd.qx - cmd.qy * cmd.qy + cmd.qz * cmd.qz;

    const float psi = 0.5f * (3.0f - (Rd11 * R11 + Rd21 * R21 + Rd31 * R31 +
                                      Rd12 * R12 + Rd22 * R22 + Rd32 * R32 +
                                      Rd13 * R13 + Rd23 * R23 + Rd33 * R33));
    float force = 0;
    if (psi < 1.0f) force = cmd.force[0] * R13 + cmd.force[1] * R23 + cmd.force[2] * R33;

    const float eR1 = 0.5f * (R12 * Rd13 - R13 * Rd12 + R22 * Rd23 - R23 * Rd22 + R32 * Rd33 - R33 * Rd32);
    const float eR2 = 0.5f * (R13 * Rd11 - R11 * Rd13 - R21 * Rd23 + R23 * Rd21 - R31 * Rd33 + R33 * Rd31);
    const float eR3 = 0.5f * (R11 * Rd12 - R12 * Rd11 + R21 * Rd22 - R22 * Rd21 + R31 * Rd32 - R32 * Rd31);

    const float in1 = Om2 * (I[2][0] * Om1 + I[2][1] * Om2 + I[2][2] * Om3) -
                      Om3 * (I[1][0] * Om1 + I[1][1] * Om2 + I[1][2] * Om3);
    const float in2 = Om3 * (I[0][0] * Om1 + I[0][1] * Om2 + I[0][2] * Om3) -
                      Om1 * (I[2][0] * Om1 + I[2][1] * Om2 + I[2][2] * Om3);
    const float in3 = Om1 * (I[1][0] * Om1 + I[1][1] * Om2 + I[1][2] * Om3) -
                      Om2 * (I[0][0] * Om1 + I[0][1] * Om2 + I[0][2] * Om3);

    const float M1 = -cmd.k_r[0] * eR1 - cmd.k_om[0] * Om1 + in1;
    const float M2 = -cmd.k_r[1] * eR2 - cmd.k_om[1] * Om2 + in2;
    const float M3 = -cmd.k_r[2] * eR3 - cmd.k_om[2] * Om3 + in3;

    float w_sq[4];
    w_sq[0] = force / (4 * kf) - M2 / (2 * d * kf) + M3 / (4 * km);
    w_sq[1] = force / (4 * kf) + M2 / (2 * d * kf) + M3 / (4 * km);
    w_sq[2] = force / (4 * kf) + M1 / (2 * d * kf) - M3 / (4 * km);
    w_sq[3] = force / (4 * kf) - M1 / (2 * d * kf) - M3 / (4 * km);

    Control control;
    for (int i = 0; i < 4; i++) {
      if (w_sq[i] < 0) w_sq[i] = 0;
      control.rpm[i] = std::sqrt(w_sq[i]);
    }
    return control;
  }

  void cmdCallback(const quadrotor_msgs::msg::SO3Command::SharedPtr cmd) {
    command_.force[0] = static_cast<float>(cmd->force.x);
    command_.force[1] = static_cast<float>(cmd->force.y);
    command_.force[2] = static_cast<float>(cmd->force.z);
    command_.qx = static_cast<float>(cmd->orientation.x);
    command_.qy = static_cast<float>(cmd->orientation.y);
    command_.qz = static_cast<float>(cmd->orientation.z);
    command_.qw = static_cast<float>(cmd->orientation.w);
    command_.k_r[0] = static_cast<float>(cmd->k_r[0]);
    command_.k_r[1] = static_cast<float>(cmd->k_r[1]);
    command_.k_r[2] = static_cast<float>(cmd->k_r[2]);
    command_.k_om[0] = static_cast<float>(cmd->k_om[0]);
    command_.k_om[1] = static_cast<float>(cmd->k_om[1]);
    command_.k_om[2] = static_cast<float>(cmd->k_om[2]);
    command_.corrections[0] = static_cast<float>(cmd->aux.kf_correction);
    command_.corrections[1] = static_cast<float>(cmd->aux.angle_corrections[0]);
    command_.corrections[2] = static_cast<float>(cmd->aux.angle_corrections[1]);
    command_.current_yaw = static_cast<float>(cmd->aux.current_yaw);
    command_.use_external_yaw = cmd->aux.use_external_yaw;
  }

  void forceDisturbanceCallback(const geometry_msgs::msg::Vector3::SharedPtr f) {
    disturbance_.f = Eigen::Vector3d(f->x, f->y, f->z);
  }

  void momentDisturbanceCallback(const geometry_msgs::msg::Vector3::SharedPtr m) {
    disturbance_.m = Eigen::Vector3d(m->x, m->y, m->z);
  }

  void simLoop() {
    const auto t_loop_start = this->now();
    auto last = control_;
    control_ = getControl(quad_, command_);
    for (int i = 0; i < 4; ++i) {
      if (std::isnan(control_.rpm[i])) control_.rpm[i] = last.rpm[i];
    }
    quad_.setInput(control_.rpm[0], control_.rpm[1], control_.rpm[2], control_.rpm[3]);
    quad_.setExternalForce(disturbance_.f);
    quad_.setExternalMoment(disturbance_.m);
    quad_.step(dt_);

    const auto now = this->now();
    if ((now - last_odom_pub_time_).seconds() >= odom_pub_interval_) {
      publishOdomAndImu(now);
      last_odom_pub_time_ = now;
    }

    const double loop_time = (this->now() - t_loop_start).seconds();
    if (loop_time > dt_) {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "Simulator loop overrun: %.2f ms > expected %.2f ms", loop_time * 1000.0, dt_ * 1000.0);
    }
  }

  void publishOdomAndImu(const rclcpp::Time &tnow) {
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = tnow;
    odom_msg.header.frame_id = "world";
    odom_msg.child_frame_id = quad_name_;
    stateToOdomMsg(quad_.getState(), odom_msg);
    odom_pub_->publish(odom_msg);

    sensor_msgs::msg::Imu imu;
    imu.header.stamp = tnow;
    imu.header.frame_id = "world";
    quadToImuMsg(quad_, imu);
    imu_pub_->publish(imu);

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = tnow;
    tf.header.frame_id = "world";
    tf.child_frame_id = quad_name_;
    tf.transform.translation.x = odom_msg.pose.pose.position.x;
    tf.transform.translation.y = odom_msg.pose.pose.position.y;
    tf.transform.translation.z = odom_msg.pose.pose.position.z;
    tf.transform.rotation = odom_msg.pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf);

    if (mesh_pub_->get_subscription_count() > 0) {
      visualization_msgs::msg::Marker mesh;
      mesh.mesh_resource = mesh_resource_;
      mesh.mesh_use_embedded_materials = true;
      mesh.header = odom_msg.header;
      mesh.header.frame_id = "world";
      mesh.ns = "mesh";
      mesh.id = 0;
      mesh.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
      mesh.action = visualization_msgs::msg::Marker::ADD;
      mesh.pose = odom_msg.pose.pose;
      mesh.scale.x = 2.0;
      mesh.scale.y = 2.0;
      mesh.scale.z = 2.0;
      mesh.color.r = 1.0;
      mesh.color.g = 1.0;
      mesh.color.b = 1.0;
      mesh.color.a = 1.0;
      mesh_pub_->publish(mesh);
    }
  }

private:
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr mesh_pub_;
  rclcpp::Subscription<quadrotor_msgs::msg::SO3Command>::SharedPtr cmd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr f_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr m_sub_;
  rclcpp::TimerBase::SharedPtr sim_timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  QuadrotorSimulator::Quadrotor quad_;
  Command command_;
  Disturbance disturbance_;
  Control control_;

  double init_x_{0.0}, init_y_{0.0}, init_z_{2.0};
  double simulation_rate_{1000.0}, odom_rate_{100.0}, dt_{0.001}, odom_pub_interval_{0.01};
  std::string quad_name_{"quadrotor"};
  std::string mesh_resource_;
  rclcpp::Time last_odom_pub_time_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<QuadrotorSimulatorRos2>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
