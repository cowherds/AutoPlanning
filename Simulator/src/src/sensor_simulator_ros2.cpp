#include <chrono>
#include <iostream>
#include <filesystem>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/common/common.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "sensor_simulator.cuh"
#include "maps.hpp"

using namespace raycast;

class SensorSimulatorRos2 final : public rclcpp::Node {
public:
  SensorSimulatorRos2() : Node("sensor_simulator_node_ros2") {
    YAML::Node config = YAML::LoadFile(CONFIG_FILE_PATH);
    const std::filesystem::path config_path(CONFIG_FILE_PATH);
    const std::filesystem::path package_root = config_path.parent_path().parent_path();
    const std::filesystem::path workspace_root = package_root.parent_path();
    if (config["tree_file"]) {
      std::filesystem::path tree_file_path(config["tree_file"].as<std::string>());
      if (tree_file_path.is_relative()) {
        const auto candidate_pkg = package_root / tree_file_path;
        const auto candidate_ws = workspace_root / tree_file_path;
        if (std::filesystem::exists(candidate_pkg)) {
          config["tree_file"] = candidate_pkg.string();
        } else {
          config["tree_file"] = candidate_ws.string();
        }
      }
    }

    lidar_ = std::make_unique<LidarParams>();
    lidar_->vertical_lines = config["lidar"]["vertical_lines"].as<int>();
    lidar_->vertical_angle_start = config["lidar"]["vertical_angle_start"].as<float>();
    lidar_->vertical_angle_end = config["lidar"]["vertical_angle_end"].as<float>();
    lidar_->horizontal_num = config["lidar"]["horizontal_num"].as<int>();
    lidar_->horizontal_resolution = config["lidar"]["horizontal_resolution"].as<float>();
    lidar_->max_lidar_dist = config["lidar"]["max_lidar_dist"].as<float>();

    lidar_pub_interval_ = 1.0 / config["lidar_fps"].as<double>();

    const std::string ply_file = config["ply_file"].as<std::string>();
    const std::string odom_topic = config["odom_topic"].as<std::string>();
    const std::string lidar_topic = config["lidar_topic"].as<std::string>();
    const std::string lidar_world_topic =
        config["lidar_world_topic"] ? config["lidar_world_topic"].as<std::string>() : "/lidar_points_world";

    const bool use_random_map = config["random_map"].as<bool>();
    float resolution = config["resolution"].as<float>();
    int occupy_threshold = config["occupy_threshold"].as<int>();

    pcl_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("mock_map", 1);

    int seed = config["seed"].as<int>();
    int sizeX = static_cast<int>(config["x_length"].as<int>() / resolution);
    int sizeY = static_cast<int>(config["y_length"].as<int>() / resolution);
    int sizeZ = static_cast<int>(config["z_length"].as<int>() / resolution);
    int type = config["maze_type"].as<int>();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    if (use_random_map) {
      mocka::Maps::BasicInfo info;
      info.sizeX = sizeX;
      info.sizeY = sizeY;
      info.sizeZ = sizeZ;
      info.seed = seed;
      info.scale = 1.0 / resolution;
      info.cloud = cloud;
      mocka::Maps map;
      map.setParam(config);
      map.setInfo(info);
      map.generate(type);
      RCLCPP_INFO(this->get_logger(), "Generated random map.");
    } else {
      if (pcl::io::loadPLYFile(ply_file, *cloud) == -1) {
        throw std::runtime_error("Failed to read point cloud file: " + ply_file);
      }
      RCLCPP_INFO(this->get_logger(), "Loaded map from %s", ply_file.c_str());
    }

    pcl::toROSMsg(*cloud, map_msg_);
    map_msg_.header.frame_id = "world";
    grid_map_ = std::make_unique<GridMap>(cloud, resolution, occupy_threshold);

    point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(lidar_topic, 1);
    point_cloud_world_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(lidar_world_topic, 1);
    using std::placeholders::_1;
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, 20, std::bind(&SensorSimulatorRos2::odomCallback, this, _1));

    timer_map_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&SensorSimulatorRos2::timerMapCallback, this));
    last_lidar_pub_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "ROS2 LiDAR sensor simulator ready.");
  }

private:
  void timerMapCallback() {
    if (pcl_pub_->get_subscription_count() > 0) {
      map_msg_.header.stamp = this->now();
      pcl_pub_->publish(map_msg_);
    }
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    quat_.x() = msg->pose.pose.orientation.x;
    quat_.y() = msg->pose.pose.orientation.y;
    quat_.z() = msg->pose.pose.orientation.z;
    quat_.w() = msg->pose.pose.orientation.w;

    pos_.x() = msg->pose.pose.position.x;
    pos_.y() = msg->pose.pose.position.y;
    pos_.z() = msg->pose.pose.position.z;
    if (!msg->child_frame_id.empty()) {
      lidar_frame_id_ = msg->child_frame_id;
      if (!lidar_frame_id_.empty() && lidar_frame_id_.front() == '/') {
        lidar_frame_id_.erase(0, 1);
      }
    }

    auto now = this->now();
    if ((now - last_lidar_pub_time_).seconds() >= lidar_pub_interval_) {
      last_lidar_pub_time_ = now;
      renderLidar(msg->header.stamp);
    }
  }

  void renderLidar(const builtin_interfaces::msg::Time &stamp) {
    cudaMat::SE3<float> T_wc(quat_.w(), quat_.x(), quat_.y(), quat_.z(), pos_.x(), pos_.y(), pos_.z());
    pcl::PointCloud<pcl::PointXYZ> lidar_points;
    renderLidarPointcloud(grid_map_.get(), lidar_.get(), T_wc, lidar_points);

    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(lidar_points, output);
    output.header.stamp = stamp;
    // renderLidarPointcloud returns points in the LiDAR/body frame.
    // Use odom child_frame_id from the latest odometry message.
    output.header.frame_id = lidar_frame_id_;
    point_cloud_pub_->publish(output);

    if (point_cloud_world_pub_->get_subscription_count() > 0) {
      pcl::PointCloud<pcl::PointXYZ> lidar_points_world;
      const Eigen::Affine3f tf_wc =
          Eigen::Translation3f(pos_) * quat_.normalized().toRotationMatrix();
      pcl::transformPointCloud(lidar_points, lidar_points_world, tf_wc);

      sensor_msgs::msg::PointCloud2 output_world;
      pcl::toROSMsg(lidar_points_world, output_world);
      output_world.header.stamp = stamp;
      output_world.header.frame_id = "world";
      point_cloud_world_pub_->publish(output_world);
    }
  }

private:
  std::unique_ptr<LidarParams> lidar_;
  std::unique_ptr<GridMap> grid_map_;

  double lidar_pub_interval_{0.1};

  Eigen::Quaternionf quat_{Eigen::Quaternionf::Identity()};
  Eigen::Vector3f pos_{Eigen::Vector3f::Zero()};
  std::string lidar_frame_id_{"body"};

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_world_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_map_;

  sensor_msgs::msg::PointCloud2 map_msg_;
  rclcpp::Time last_lidar_pub_time_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SensorSimulatorRos2>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
