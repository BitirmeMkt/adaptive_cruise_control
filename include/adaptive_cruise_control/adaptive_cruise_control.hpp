#ifndef MY_PACKAGE__MY_NODE_HPP_
#define MY_PACKAGE__MY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <laser_geometry/laser_geometry.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <cmath>
#include <unistd.h>
#include <stdio.h>
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"

namespace adaptive_cruise_control
{

class AdaptiveCruiseControl : public rclcpp::Node
{
public:

  explicit AdaptiveCruiseControl(const rclcpp::NodeOptions & options);

private:
  // Buraya publisher, subscriber, timer vb. eklersin
  void read_parameters();

  void extrinsic_calculation();


  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  void camera_info_topic_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

  void detection_callback(const vision_msgs::msg::Detection2DArray::SharedPtr msg);

  void timer_callback();

  int adaptive_cruise_controller(double relative_lead_vehicle_speed);

  int cruise_controller();

  int normalize_output(double output);

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscriber_;

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscriber_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;

  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detection_subscriber_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr adaptive_cruise_controller_flag_subscriber_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr throttle_publisher_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr adaptive_cruise_controller_flag_publisher_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr sudden_brake_distance_publisher_;

  laser_geometry::LaserProjection projector_;

  sensor_msgs::msg::PointCloud2 m_pointcloud_msg;

  std::vector<double> translation_vector_;

  std::vector<double> rotation_matrix_;

  std::string laser_topic_;

  std::string camera_topic_;

  std::string camera_info_topic_;

  std::string detection_topic_;

  std::string throttle_publisher_topic_;

  std::string serial_port_;

  int serial_baudrate_;

  double bbox_center_x_;

  double bbox_center_y_;

  double bbox_size_x_;

  double bbox_size_y_;

  // 4x4 extrinsic matrix [row-major]
  std::array<std::array<double, 4>, 4> extrinsic{};

  // eigen
  Eigen::Matrix<double, 3, 4> extrinsic_matrix_;

  double u_;
  
  double v_;

  bool received_once_;

  bool bbox_received_;

  bool relative_lead_vehicle_speed_flag_;

  bool old_distance_initialized_;

  double fx_;

  double fy_;

  double cx_;

  double cy_;

  std::vector<cv::Point2d> image_points_;

  struct LidarCameraPoint
  {
    float x;
    float y;
    float z;
    int u;
    int v;
  };

  std::vector<LidarCameraPoint> bbox_points;

  std::vector<double> distance_values;

  std::vector<double> sudden_brake_distance_values;

  rclcpp::TimerBase::SharedPtr timer_;

  double old_distance_;

  double timer_value_;

  double kp_;

  double ki_;

  double kd_;

  double pid_max_;

  double pid_min_;

  bool adaptive_cruise_controller_flag_;

  std::string adaptive_cruite_controller_topic_;

  double sudden_brake_box_min_x_;

  double sudden_brake_box_max_x_;

  double sudden_brake_box_min_y_;

  double sudden_brake_box_max_y_;

  std::string sudden_brake_distance_topic_;

  double distance_threshold_;

  int throttle;

  double reference_speed_ = 12.0; //Cruise controller speed

  double vehicle_speed_ = 10.0; // From stm32
  
  double prev_error_adaptive_;

  double prev_error_;

  double relative_lead_vehicle_speed;

  double integral_adaptive_ = 0.0;

  double integral_ = 0.0;
};

}  // namespace my_package

#endif  // MY_PACKAGE__MY_NODE_HPP_
