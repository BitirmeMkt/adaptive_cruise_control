#ifndef MY_PACKAGE__MY_NODE_HPP_
#define MY_PACKAGE__MY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <laser_geometry/laser_geometry.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <Eigen/Dense>

namespace adaptive_cruise_control
{

class AdaptiveCruiseControl : public rclcpp::Node
{
public:

  explicit AdaptiveCruiseControl(const rclcpp::NodeOptions & options);

private:
  // Buraya publisher, subscriber, timer vb. eklersin
  inline bool read_parameters();

  void extrinsic_calculation();


  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  void camera_info_topic_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscriber_;

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscriber_;

  laser_geometry::LaserProjection projector_;

  sensor_msgs::msg::PointCloud2 m_pointcloud_msg;

  std::vector<double> translation_vector_;

  std::vector<double> rotation_matrix_;

  std::string laser_topic_;

  std::string camera_topic_;

  std::string camera_info_topic_;

  // 4x4 extrinsic matrix [row-major]
  std::array<std::array<double, 4>, 4> extrinsic{};

  // eigen
  Eigen::Matrix<double, 3, 4> extrinsic_matrix_;

  double u_;
  
  double v_;

  bool received_once_;

  double fx_;

  double fy_;

  double cx_;

  double cy_;
};

}  // namespace my_package

#endif  // MY_PACKAGE__MY_NODE_HPP_
