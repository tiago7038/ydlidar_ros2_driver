/*
 *  YDLIDAR SYSTEM
 *  YDLIDAR ROS 2 Node
 *
 *  Copyright 2017 - 2020 EAI TEAM
 *  http://www.eaibot.com
 *
 */

#ifdef _MSC_VER
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#endif

#include "src/CYdLidar.h"
#include "core/common/ydlidar_help.h"
#include <math.h>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <thread>

#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time_source.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_srvs/srv/empty.hpp"
#include <vector>
#include <iostream>
#include <string>
#include <signal.h>

#define ROS2Verision "1.0.1"

namespace {

void sdkLogCallback(ydlidar::core::common::LogLevel level,
                    const char *message, void *user_data)
{
  auto *node = static_cast<rclcpp::Node *>(user_data);
  std::istringstream lines(message ? message : "");
  std::string line;

  while (std::getline(lines, line)) {
    if (line.empty()) {
      continue;
    }

    switch (level) {
      case ydlidar::core::common::LogLevelDebug:
        RCLCPP_DEBUG(node->get_logger(), "%s", line.c_str());
        break;
      case ydlidar::core::common::LogLevelWarn:
        RCLCPP_WARN(node->get_logger(), "%s", line.c_str());
        break;
      case ydlidar::core::common::LogLevelError:
        RCLCPP_ERROR(node->get_logger(), "%s", line.c_str());
        break;
      case ydlidar::core::common::LogLevelInfo:
      default:
        RCLCPP_INFO(node->get_logger(), "%s", line.c_str());
        break;
    }
  }
}

sensor_msgs::msg::PointCloud2 makePointCloud(
  const LaserScan & scan, const std::string & frame_id)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.stamp.sec = RCL_NS_TO_S(scan.stamp);
  cloud.header.stamp.nanosec = scan.stamp - RCL_S_TO_NS(cloud.header.stamp.sec);
  cloud.header.frame_id = frame_id;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2Fields(
    4,
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32,
    "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);

  size_t valid_point_count = 0;
  for (const auto & point : scan.points) {
    if (std::isfinite(point.range) &&
      point.range >= scan.config.min_range &&
      point.range <= scan.config.max_range)
    {
      ++valid_point_count;
    }
  }
  modifier.resize(valid_point_count);

  sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
  sensor_msgs::PointCloud2Iterator<float> intensity(cloud, "intensity");
  for (const auto & point : scan.points) {
    if (!std::isfinite(point.range) ||
      point.range < scan.config.min_range ||
      point.range > scan.config.max_range)
    {
      continue;
    }
    *x = point.range * std::cos(point.angle);
    *y = point.range * std::sin(point.angle);
    *z = 0.0f;
    *intensity = point.intensity;
    ++x;
    ++y;
    ++z;
    ++intensity;
  }
  cloud.is_dense = true;
  return cloud;
}

}  // namespace


int main(int argc, char *argv[]) {
#ifdef _WIN32
  _putenv_s("RCUTILS_COLORIZED_OUTPUT", "0");
#else
  setenv("RCUTILS_COLORIZED_OUTPUT", "0", 1);
#endif
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("ydlidar_ros2_driver_node");
  ydlidar::core::common::setLogCallback(sdkLogCallback, node.get());

  RCLCPP_INFO(node->get_logger(), "Current ROS driver version: %s", ROS2Verision);

  CYdLidar laser;
  std::string str_optvalue = "/dev/ydlidar";
  str_optvalue = node->declare_parameter("port", str_optvalue);
  ///lidar port
  laser.setlidaropt(LidarPropSerialPort, str_optvalue.c_str(), str_optvalue.size());

  ///ignore array
  str_optvalue = "";
  str_optvalue = node->declare_parameter("ignore_array", str_optvalue);
  laser.setlidaropt(LidarPropIgnoreArray, str_optvalue.c_str(), str_optvalue.size());

  std::string frame_id = "laser_frame";
  frame_id = node->declare_parameter("frame_id", frame_id);

  //////////////////////int property/////////////////
  /// lidar baudrate
  int optval = 230400;
  optval = node->declare_parameter("baudrate", optval);
  laser.setlidaropt(LidarPropSerialBaudrate, &optval, sizeof(int));
  /// tof lidar
  optval = TYPE_TRIANGLE;
  optval = node->declare_parameter("lidar_type", optval);
  laser.setlidaropt(LidarPropLidarType, &optval, sizeof(int));
  /// device type
  optval = YDLIDAR_TYPE_SERIAL;
  optval = node->declare_parameter("device_type", optval);
  laser.setlidaropt(LidarPropDeviceType, &optval, sizeof(int));
  /// sample rate
  optval = 9;
  optval = node->declare_parameter("sample_rate", optval);
  laser.setlidaropt(LidarPropSampleRate, &optval, sizeof(int));
  /// abnormal count
  optval = 4;
  optval = node->declare_parameter("abnormal_check_count", optval);
  laser.setlidaropt(LidarPropAbnormalCheckCount, &optval, sizeof(int));

  /// Intenstiy bit count
  optval = 0;
  optval = node->declare_parameter("intensity_bit", optval);
  laser.setlidaropt(LidarPropIntenstiyBit, &optval, sizeof(int));
     
  //////////////////////bool property/////////////////
  /// fixed angle resolution
  bool b_optvalue = false;
  b_optvalue = node->declare_parameter("fixed_resolution", b_optvalue);
  laser.setlidaropt(LidarPropFixedResolution, &b_optvalue, sizeof(bool));
  /// rotate 180
  b_optvalue = true;
  b_optvalue = node->declare_parameter("reversion", b_optvalue);
  laser.setlidaropt(LidarPropReversion, &b_optvalue, sizeof(bool));
  /// Counterclockwise
  b_optvalue = true;
  b_optvalue = node->declare_parameter("inverted", b_optvalue);
  laser.setlidaropt(LidarPropInverted, &b_optvalue, sizeof(bool));
  b_optvalue = true;
  b_optvalue = node->declare_parameter("auto_reconnect", b_optvalue);
  laser.setlidaropt(LidarPropAutoReconnect, &b_optvalue, sizeof(bool));
  /// one-way communication
  b_optvalue = false;
  b_optvalue = node->declare_parameter("isSingleChannel", b_optvalue);
  laser.setlidaropt(LidarPropSingleChannel, &b_optvalue, sizeof(bool));
  /// intensity
  b_optvalue = false;
  b_optvalue = node->declare_parameter("intensity", b_optvalue);
  laser.setlidaropt(LidarPropIntenstiy, &b_optvalue, sizeof(bool));
  /// Motor DTR
  b_optvalue = false;
  b_optvalue = node->declare_parameter("support_motor_dtr", b_optvalue);
  laser.setlidaropt(LidarPropSupportMotorDtrCtrl, &b_optvalue, sizeof(bool));
  //是否启用调试
  b_optvalue = false;
  b_optvalue = node->declare_parameter("debug", b_optvalue);
  laser.setEnableDebug(b_optvalue);

  //////////////////////float property/////////////////
  /// unit: °
  float f_optvalue = 180.0f;
  f_optvalue = static_cast<float>(
    node->declare_parameter<double>("angle_max", f_optvalue));
  laser.setlidaropt(LidarPropMaxAngle, &f_optvalue, sizeof(float));
  f_optvalue = -180.0f;
  f_optvalue = static_cast<float>(
    node->declare_parameter<double>("angle_min", f_optvalue));
  laser.setlidaropt(LidarPropMinAngle, &f_optvalue, sizeof(float));
  /// unit: m
  f_optvalue = 64.f;
  f_optvalue = static_cast<float>(
    node->declare_parameter<double>("range_max", f_optvalue));
  laser.setlidaropt(LidarPropMaxRange, &f_optvalue, sizeof(float));
  f_optvalue = 0.1f;
  f_optvalue = static_cast<float>(
    node->declare_parameter<double>("range_min", f_optvalue));
  laser.setlidaropt(LidarPropMinRange, &f_optvalue, sizeof(float));
  /// unit: Hz
  f_optvalue = 10.f;
  f_optvalue = static_cast<float>(
    node->declare_parameter<double>("frequency", f_optvalue));
  laser.setlidaropt(LidarPropScanFrequency, &f_optvalue, sizeof(float));

  bool invalid_range_is_inf = false;
  invalid_range_is_inf = node->declare_parameter("invalid_range_is_inf", invalid_range_is_inf);

  const bool publish_laserscan =
    node->declare_parameter("publish_laserscan", true);
  const bool publish_pointcloud =
    node->declare_parameter("publish_pointcloud", true);
  const std::string pointcloud_topic =
    node->declare_parameter<std::string>("pointcloud_topic", "pointcloud");

  //初始化
  bool ret = laser.initialize();
  if (ret) 
  {
    //设置GS工作模式（非GS雷达请无视该代码）
    int i_v = 0;
    i_v = node->declare_parameter("m1_mode", i_v);
    laser.setWorkMode(i_v, 0x01);
    i_v = 0;
    i_v = node->declare_parameter("m2_mode", i_v);
    laser.setWorkMode(i_v, 0x02);
    i_v = 1;
    i_v = node->declare_parameter("m3_mode", i_v);
    laser.setWorkMode(i_v, 0x04);
    //启动扫描
    ret = laser.turnOn();
  } 
  else 
  {
    RCLCPP_ERROR(node->get_logger(), "%s", laser.DescribeError());
  }
  
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub;
  if (publish_laserscan) {
    laser_pub = node->create_publisher<sensor_msgs::msg::LaserScan>(
      "scan", rclcpp::SensorDataQoS());
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub;
  if (publish_pointcloud) {
    pointcloud_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>(
      pointcloud_topic, rclcpp::SensorDataQoS());
  }

  auto stop_scan_service =
    [&laser](const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<std_srvs::srv::Empty::Request> req,
  std::shared_ptr<std_srvs::srv::Empty::Response> response) -> bool
  {
    return laser.turnOff();
  };

  auto stop_service = node->create_service<std_srvs::srv::Empty>("stop_scan",stop_scan_service);

  auto start_scan_service =
    [&laser](const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<std_srvs::srv::Empty::Request> req,
  std::shared_ptr<std_srvs::srv::Empty::Response> response) -> bool
  {
    return laser.turnOn();
  };

  auto start_service = node->create_service<std_srvs::srv::Empty>("start_scan",start_scan_service);

  rclcpp::WallRate loop_rate(20);

  //std::ofstream file("pointcloud_data.txt"); // 打开文件流
  while (ret && rclcpp::ok()) 
  {
    LaserScan scan;
    if (laser.doProcessSimple(scan)) 
    {
      if (publish_laserscan) {
        auto scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>();
        scan_msg->header.stamp.sec = RCL_NS_TO_S(scan.stamp);
        scan_msg->header.stamp.nanosec =
          scan.stamp - RCL_S_TO_NS(scan_msg->header.stamp.sec);
        scan_msg->header.frame_id = frame_id;
        scan_msg->angle_min = scan.config.min_angle;
        scan_msg->angle_max = scan.config.max_angle;
        scan_msg->angle_increment = scan.config.angle_increment;
        scan_msg->scan_time = scan.config.scan_time;
        scan_msg->time_increment = scan.config.time_increment;
        scan_msg->range_min = scan.config.min_range;
        scan_msg->range_max = scan.config.max_range;

        const int size =
          (scan.config.max_angle - scan.config.min_angle) /
          scan.config.angle_increment + 1;
        scan_msg->ranges.resize(size);
        scan_msg->intensities.resize(size);
        for (const auto & point : scan.points) {
          const int index = std::ceil(
            (point.angle - scan.config.min_angle) / scan.config.angle_increment);
          if (index >= 0 && index < size) {
            scan_msg->ranges[index] = point.range;
            scan_msg->intensities[index] = point.intensity;
          }
        }
        laser_pub->publish(std::move(scan_msg));
      }

      if (publish_pointcloud) {
        pointcloud_pub->publish(makePointCloud(scan, frame_id));
      }
    } 
    else 
    {
      RCLCPP_ERROR(node->get_logger(), "Failed to get scan");
    }
    if(!rclcpp::ok()) 
    {
      break;
    }
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  RCLCPP_INFO(node->get_logger(), "YDLIDAR is stopping");

  // A failed USB device can leave the SDK's serial reader blocked in the
  // kernel. Bound shutdown so ros2 launch never has to wait for SIGTERM and
  // SIGKILL. Normal cleanup completes well before this timeout.
  std::mutex shutdown_mutex;
  std::condition_variable shutdown_condition;
  bool shutdown_complete = false;
  std::thread shutdown_watchdog([&]() {
    std::unique_lock<std::mutex> lock(shutdown_mutex);
    if (!shutdown_condition.wait_for(
          lock, std::chrono::seconds(3), [&]() { return shutdown_complete; })) {
      RCLCPP_ERROR(
        node->get_logger(),
        "YDLIDAR shutdown timed out; forcing this driver process to exit");
      std::_Exit(EXIT_SUCCESS);
    }
  });

  laser.turnOff();
  ydlidar::core::common::setLogCallback(NULL);

  {
    std::lock_guard<std::mutex> lock(shutdown_mutex);
    shutdown_complete = true;
  }
  shutdown_condition.notify_one();
  shutdown_watchdog.join();
  rclcpp::shutdown();

  // Some USB-serial adapters can block indefinitely in closePort() after a
  // communication fault. This executable owns the lidar and its descriptor,
  // so exiting the process is the safest cleanup: the kernel closes the file
  // descriptor without running the SDK's blocking disconnect/destructor path.
  std::_Exit(EXIT_SUCCESS);
}
