#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <algorithm>
#include <random>
#include <cctype>
#include "sensor_simulator.cuh"
#include "maps.hpp"

using namespace raycast;
namespace fs = std::filesystem;

void prepareSavePath(const std::string &path, bool print=false)
{
    if (fs::exists(path))
    {
        if (print)
            std::cout << "Directory exists. Removing: " << path << std::endl;
        fs::remove_all(path);
    }
    fs::create_directories(path);
    if (print)
        std::cout << "Created new dataset directory: " << path << std::endl;
}

void savePointCloudAsPLY(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const std::string &path)
{
    if (pcl::io::savePLYFileBinary(path, *cloud) == -1)
        std::cerr << "Failed to save ply file to " << path << std::endl;
}

void saveLidarPointsAsBinary(const pcl::PointCloud<pcl::PointXYZ> &cloud, const std::string &filepath)
{
    std::ofstream out(filepath, std::ios::binary);
    if (!out.is_open())
    {
        std::cerr << "Failed to save lidar binary to " << filepath << std::endl;
        return;
    }
    for (const auto &pt : cloud.points)
    {
        float data[3] = {pt.x, pt.y, pt.z};
        out.write(reinterpret_cast<const char *>(data), 3 * sizeof(float));
    }
    out.close();
}

Eigen::Quaternionf RPY2Quat(float roll_deg, float pitch_deg, float yaw_deg)
{
    float roll = roll_deg * M_PI / 180.0f;
    float pitch = pitch_deg * M_PI / 180.0f;
    float yaw = yaw_deg * M_PI / 180.0f;
    Eigen::AngleAxisf rollAngle(roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf pitchAngle(pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf yawAngle(yaw, Eigen::Vector3f::UnitZ());
    return yawAngle * pitchAngle * rollAngle;
}

void printProgressBar(int current, int total, int bar_width = 50)
{
    float progress = static_cast<float>(current) / total;
    int pos = static_cast<int>(bar_width * progress);
    std::cout << "\r[";
    for (int i = 0; i < bar_width; ++i)
    {
        if (i < pos)
            std::cout << "=";
        else if (i == pos)
            std::cout << ">";
        else
            std::cout << " ";
    }
    std::cout << "] " << int(progress * 100.0f) << "%";
    std::cout.flush();
}

std::string resolveSavePath(const std::string &raw_path, const std::filesystem::path &workspace_root)
{
    std::filesystem::path p(raw_path);
    if (p.is_relative())
    {
        p = workspace_root / p;
    }
    p = p.lexically_normal();
    return p.string();
}

int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;
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

    LidarParams lidar;
    lidar.vertical_lines = config["lidar"]["vertical_lines"] ? config["lidar"]["vertical_lines"].as<int>() : 64;
    lidar.vertical_angle_start = config["lidar"]["vertical_angle_start"] ? config["lidar"]["vertical_angle_start"].as<float>() : -38.7f;
    lidar.vertical_angle_end = config["lidar"]["vertical_angle_end"] ? config["lidar"]["vertical_angle_end"].as<float>() : 38.7f;
    lidar.horizontal_num = config["lidar"]["horizontal_num"] ? config["lidar"]["horizontal_num"].as<int>() : 360;
    lidar.horizontal_resolution = config["lidar"]["horizontal_resolution"] ? config["lidar"]["horizontal_resolution"].as<float>() : 1.0f;
    lidar.max_lidar_dist = config["lidar"]["max_lidar_dist"] ? config["lidar"]["max_lidar_dist"].as<float>() : 20.0f;
    std::cout << "LiDAR-only dataset generation. Lines=" << lidar.vertical_lines
              << ", horizontal=" << lidar.horizontal_num
              << ", FOV=[" << lidar.vertical_angle_start << ", " << lidar.vertical_angle_end
              << "] deg, max_dist=" << lidar.max_lidar_dist << "m" << std::endl;

    float resolution = config["resolution"].as<float>();
    int occupy_threshold = config["occupy_threshold"].as<int>();
    int seed = config["seed"].as<int>();
    int sizeX = config["x_length"].as<int>();
    int sizeY = config["y_length"].as<int>();
    int sizeZ = config["z_length"].as<int>();
    double scale = 1 / resolution;
    sizeX *= scale;
    sizeY *= scale;
    sizeZ *= scale;

    std::string raw_save_path = config["save_path"].as<std::string>();
    std::string save_path = resolveSavePath(raw_save_path, workspace_root);
    if (!save_path.empty() && save_path.back() != '/')
    {
        save_path += "/";
    }
    std::cout << "Raw save_path: " << raw_save_path << std::endl;
    std::cout << "Resolved save_path: " << save_path << std::endl;
    std::cout << "Dataset format: lidar_<i>.bin" << std::endl;

    int env_num = config["env_num"].as<int>();
    int image_num = config["image_num"].as<int>();
    float roll_range = config["roll_range"].as<float>();
    float pitch_range = config["pitch_range"].as<float>();
    float x_range = config["x_range"].as<float>();
    float y_range = config["y_range"].as<float>();
    float z_min = config["z_range"][0].as<float>();
    float z_max = config["z_range"][1].as<float>();
    float safe_dist = config["safe_dist"].as<float>();
    float ply_res = config["ply_res"].as<float>();

    int dataset_num = env_num * image_num;
    float x_min = -x_range / 2.0f;
    float y_min = -y_range / 2.0f;

    std::cout << "Map range (m): "
              << "X: [" << -sizeX * resolution / 2.0 << ", " << sizeX * resolution / 2.0 << "], "
              << "Y: [" << -sizeY * resolution / 2.0 << ", " << sizeY * resolution / 2.0 << "], "
              << "Z: [" << 0 << ", " << sizeZ * resolution << "]" << std::endl;
    std::cout << "Sampling range (m): "
              << "X: [" << x_min << ", " << x_min + x_range << "], "
              << "Y: [" << y_min << ", " << y_min + y_range << "], "
              << "Z: [" << z_min << ", " << z_max << "]" << std::endl;
    std::cout << "Angle range (deg): "
              << "Roll: [" << -roll_range << ", " << roll_range << "], "
              << "Pitch: [" << -pitch_range << ", " << pitch_range << "], "
              << "Yaw: [0, 360]" << std::endl;

    std::default_random_engine generator(std::random_device{}());
    std::normal_distribution<float> normal_distribution(0.0f, 1.0f);
    std::uniform_real_distribution<float> uniform_uniform(0.0f, 1.0f);
    prepareSavePath(save_path, true);

    for (int map_i = 0; map_i < env_num; ++map_i)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        mocka::Maps::BasicInfo info;
        info.sizeX = sizeX;
        info.sizeY = sizeY;
        info.sizeZ = sizeZ;
        info.seed = seed + map_i;
        info.scale = scale;
        info.cloud = cloud;

        mocka::Maps map;
        map.setParam(config);
        map.setInfo(info);
        map.generate(config["maze_type"].as<int>());

        GridMap grid_map(cloud, resolution, occupy_threshold);

        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud(cloud);
        sor.setLeafSize(ply_res, ply_res, ply_res);
        sor.filter(*filtered_cloud);
        pcl::PointXYZ min_pt, max_pt;
        pcl::getMinMax3D(*filtered_cloud, min_pt, max_pt);

        std::string data_path = save_path + std::to_string(map_i) + "/";
        prepareSavePath(data_path);

        savePointCloudAsPLY(filtered_cloud, save_path + "pointcloud-" + std::to_string(map_i) + ".ply");

        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(filtered_cloud);

        std::ofstream pose_file(save_path + "pose-" + std::to_string(map_i) + ".csv");
        pose_file << "px,py,pz,qw,qx,qy,qz\n";

        for (int image_i = 0; image_i < image_num; ++image_i)
        {
            Eigen::Vector3f pos;
            float dist;
            do {
                pos.x() = x_min + uniform_uniform(generator) * x_range;
                pos.y() = y_min + uniform_uniform(generator) * y_range;
                pos.z() = z_min + uniform_uniform(generator) * (z_max - z_min);
                pcl::PointXYZ searchPoint(pos.x(), pos.y(), pos.z());
                std::vector<int> pointIdxNKNSearch(1);
                std::vector<float> pointNKNSquaredDistance(1);
                kdtree.nearestKSearch(searchPoint, 1, pointIdxNKNSearch, pointNKNSquaredDistance);
                dist = sqrt(pointNKNSquaredDistance[0]);
            } while (dist < safe_dist);

            float roll = normal_distribution(generator) * roll_range / 3.0f;
            float pitch_angle = normal_distribution(generator) * pitch_range / 3.0f;
            float yaw = uniform_uniform(generator) * 360.0f;

            Eigen::Quaternionf quat = RPY2Quat(roll, pitch_angle, yaw);

            cudaMat::SE3<float> T_wl(quat.w(), quat.x(), quat.y(), quat.z(),
                                      pos.x(), pos.y(), pos.z());

            pcl::PointCloud<pcl::PointXYZ> lidar_points;
            renderLidarPointcloud(&grid_map, &lidar, T_wl, lidar_points);

            std::string filename = data_path + "lidar_" + std::to_string(image_i) + ".bin";
            saveLidarPointsAsBinary(lidar_points, filename);

            pose_file << std::fixed << std::setprecision(6)
                      << pos.x() << "," << pos.y() << "," << pos.z() << ","
                      << quat.w() << "," << quat.x() << ","
                      << quat.y() << "," << quat.z() << "\n";

            printProgressBar(map_i * image_num + image_i + 1, dataset_num);
        }
        pose_file.close();
        grid_map.freeGridMap();
    }

    std::cout << "\nDataset generation completed!" << std::endl;
    return 0;
}
