#include "adaptive_cruise_control/adaptive_cruise_control.hpp"

namespace adaptive_cruise_control
{

AdaptiveCruiseControl::AdaptiveCruiseControl(const rclcpp::NodeOptions & options)
: Node("adaptive_cruise_control_node", options)
{
  // Geri kalan başlatma işlemleri burada
  read_parameters();

  extrinsic_calculation();

  camera_info_subscriber_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    camera_info_topic_, 10, std::bind(&AdaptiveCruiseControl::camera_info_topic_callback, this, std::placeholders::_1));

  laser_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    laser_topic_, 10, std::bind(&AdaptiveCruiseControl::scan_callback, this, std::placeholders::_1));

  image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
    camera_topic_, 10, std::bind(&AdaptiveCruiseControl::image_callback, this, std::placeholders::_1));
  
  detection_subscriber_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
    detection_topic_, 10, std::bind(&AdaptiveCruiseControl::detection_callback, this, std::placeholders::_1));

  throttle_publisher_ = this->create_publisher<std_msgs::msg::Float32>(
    throttle_publisher_topic_.c_str(), 10);

  adaptive_cruise_controller_flag_publisher_ = this->create_publisher<std_msgs::msg::Bool>(
    adaptive_cruite_controller_topic_ .c_str(), 10);

  // Timer callback
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(timer_value_)), std::bind(&AdaptiveCruiseControl::timer_callback, this));
}

void AdaptiveCruiseControl::read_parameters()
{
    // this->declare_parameter<string>("asdf","default");
    this->declare_parameter<std::vector<double>>("translation_vector",{0.0,0.0,0.0});
    this->declare_parameter<std::vector<double>>("rotation_matrix",std::vector<double>(9, 0.0));
    this->declare_parameter<std::string>("laser_topic","/default_laser_topic");
    this->declare_parameter<std::string>("camera_topic","/default_camera_topic");
    this->declare_parameter<std::string>("camera_info_topic","/default_camera_info_topic");
    this->declare_parameter<std::string>("detection_topic", "/default_detection_topic");
    this->declare_parameter<std::string>("throttle_publisher_topic", "/default_throttle_topic");    
    this->declare_parameter<std::string>("serial_port", "/dev/default");
    this->declare_parameter<int>("serial_baudrate", 0);
    this->declare_parameter<double>("timer_value", 100.0);
    this->declare_parameter<int>("kp_value", 0);
    this->declare_parameter<int>("ki_value", 0);
    this->declare_parameter<int>("kd_value", 0);
    this->declare_parameter<double>("pid_max", 0.0);
    this->declare_parameter<double>("pid_min", 0.0);
    this->declare_parameter<bool>("adaptive_cruise_controller_flag", false);
    this->declare_parameter<std::string>("adaptive_cruite_controller_topic", "default_adaptive_cruise_controller_flag");

    translation_vector_ = this->get_parameter("translation_vector").as_double_array();
    rotation_matrix_ = this->get_parameter("rotation_matrix").as_double_array();
    laser_topic_ = this->get_parameter("laser_topic").as_string();
    camera_topic_ = this->get_parameter("camera_topic").as_string();
    camera_info_topic_ = this->get_parameter("camera_info_topic").as_string();
    detection_topic_ = this->get_parameter("detection_topic").as_string();
    throttle_publisher_topic_ = this->get_parameter("throttle_publisher_topic").as_string();
    serial_port_ = this->get_parameter("serial_port").as_string();
    serial_baudrate_ = this->get_parameter("serial_baudrate").as_int();
    timer_value_ = this->get_parameter("timer_value").as_double();
    kp_ = static_cast<double>(this->get_parameter("kp_value").as_int());
    ki_ = static_cast<double>(this->get_parameter("ki_value").as_int());
    kd_ = static_cast<double>(this->get_parameter("kd_value").as_int());
    pid_max_ = this->get_parameter("pid_max").as_double();
    pid_min_ = this->get_parameter("pid_min").as_double();
    adaptive_cruise_controller_flag_ = this->get_parameter("adaptive_cruise_controller_flag").as_bool();
    adaptive_cruite_controller_topic_ = this->get_parameter("adaptive_cruite_controller_topic").as_string();

    RCLCPP_INFO(this->get_logger(), "translation_vector: [%f, %f, %f]", translation_vector_[0], translation_vector_[1], translation_vector_[2]);
    RCLCPP_INFO(this->get_logger(), "rotation_matrix: [%f, %f, %f, %f, %f, %f, %f, %f, %f]", rotation_matrix_[0], rotation_matrix_[1],
     rotation_matrix_[2], rotation_matrix_[3], rotation_matrix_[4], rotation_matrix_[5], rotation_matrix_[6], rotation_matrix_[7], rotation_matrix_[8]);
    RCLCPP_INFO(this->get_logger(), "laser_topic: %s", laser_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "camera_topic: %s", camera_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "camera_info_topic: %s", camera_info_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "detection_topic: %s", detection_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "throttle_publisher_topic: %s", detection_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "serial_port: %s", serial_port_.c_str());
    RCLCPP_INFO(this->get_logger(), "serial_baudrate: %d", serial_baudrate_);
    RCLCPP_INFO(this->get_logger(), "timer_value: %f", timer_value_);
    RCLCPP_INFO(this->get_logger(), "kp: %f", kp_);
    RCLCPP_INFO(this->get_logger(), "ki: %f", ki_);
    RCLCPP_INFO(this->get_logger(), "kd: %f", kd_);
    RCLCPP_INFO(this->get_logger(), "pid_max: %f", pid_max_);
    RCLCPP_INFO(this->get_logger(), "pid_min: %f", pid_min_);
    RCLCPP_INFO(this->get_logger(), "adaptive_cruise_controller_flag: %s", adaptive_cruise_controller_flag_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "adaptive_cruite_controller_topic: %s", adaptive_cruite_controller_topic_.c_str());
}

void AdaptiveCruiseControl::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  image_points_.clear();
  bbox_points.clear();
  distance_values.clear();
  // RCLCPP_INFO(this->get_logger(), "LaserScan mesaji alindi! Mesafe: %f", msg->ranges[0]);
  projector_.projectLaser(*msg,m_pointcloud_msg);
  m_pointcloud_msg.header.frame_id = msg->header.frame_id;
  m_pointcloud_msg.header.stamp = msg->header.stamp;
  // RCLCPP_INFO(this->get_logger(), "PointCloud2 mesaji olusturuldu!");
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(m_pointcloud_msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(m_pointcloud_msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(m_pointcloud_msg, "z");

  for (size_t i = 0; i < m_pointcloud_msg.width * m_pointcloud_msg.height; ++i, ++iter_x, ++iter_y, ++iter_z) {
    // RCLCPP_INFO(this->get_logger(), "Point %zu: x=%f, y=%f, z=%f", i, *iter_x, *iter_y, *iter_z);

    Eigen::Vector3d camera_matrix_;

    camera_matrix_ = extrinsic_matrix_ * Eigen::Vector4d(-(*iter_y), -(*iter_z), *iter_x, 1.0);
    // RCLCPP_INFO(this->get_logger(), "Point %zu: x=%f, y=%f, z=%f", i, camera_matrix_(0), camera_matrix_(1), camera_matrix_(2));

    u_ = fx_ * camera_matrix_(0) / camera_matrix_(2) + cx_;
    v_ = fy_ * camera_matrix_(1) / camera_matrix_(2) + cy_;
    if(bbox_received_ == true)
    {
      if(bbox_center_x_ + bbox_size_x_ / 2 > u_ && bbox_center_x_ - bbox_size_x_ / 2 < u_ &&
         bbox_center_y_ + bbox_size_y_ / 2 > v_ && bbox_center_y_ - bbox_size_y_ / 2 < v_)
      {
        // RCLCPP_INFO(this->get_logger(), "Uygun nokta bulundu! u: %f, v: %f", u_, v_);
        LidarCameraPoint bbox_point;
        bbox_point.x = camera_matrix_(0);
        bbox_point.y = camera_matrix_(1);
        bbox_point.z = camera_matrix_(2);
        bbox_point.u = u_;
        bbox_point.v = v_;
        bbox_points.push_back(bbox_point);
        // Find distance to the object
        double distance = std::sqrt(std::pow(bbox_point.x, 2) + std::pow(bbox_point.y, 2) + std::pow(bbox_point.z, 2));
        // RCLCPP_INFO(this->get_logger(), "Uygun noktanin mesafesi: %f", distance);
        distance_values.push_back(distance);
      }
      else
      {
        // RCLCPP_INFO(this->get_logger(), "Uygun nokta bulunamadi!");
      }
      image_points_.push_back(cv::Point2d(u_,v_));
      // RCLCPP_INFO(this->get_logger(), "Pixel coordinates for point [%zu]: u=%f, v=%f",i, u_, v_);
    }

  }
}

void AdaptiveCruiseControl::extrinsic_calculation()
{  
  // 2. 3x3 rotasyon matrisini doldur
  for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j)
          extrinsic_matrix_(i, j) = rotation_matrix_[i * 3 + j];
  
  // 3. 3x1 translation vektörünü son sütuna koy
  for (int i = 0; i < 3; ++i)
      extrinsic_matrix_(i, 3) = translation_vector_[i];
  
  // 4. Yazdır (RCLCPP_INFO ile)
  for (int i = 0; i < 3; ++i) {
      std::stringstream row;
      for (int j = 0; j < 4; ++j)
          row << extrinsic_matrix_(i, j) << " ";
      RCLCPP_INFO(this->get_logger(), "%s", row.str().c_str());
  }
}

void AdaptiveCruiseControl::camera_info_topic_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  if(!received_once_)
  {
    RCLCPP_INFO(this->get_logger(), "CameraInfo mesaji alindi!");
    for (size_t i = 0; i < 9; ++i) {
      RCLCPP_INFO(this->get_logger(), "K[%zu]: %f", i, msg->k[i]);
      fx_ = msg->k[0];
      fy_ = msg->k[4];
      cx_ = msg->k[2];
      cy_ = msg->k[5];
      received_once_ = true;
    }
  }
}

void AdaptiveCruiseControl::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  try
  {
  // RCLCPP_INFO(this->get_logger(), "Image mesaji alindi!" );
  cv::Mat image = cv_bridge::toCvCopy(msg, "bgr8")->image;
  for (const auto& point : image_points_)
  {
    cv::Point point_int(static_cast<int>(point.x), static_cast<int>(point.y));
    cv::circle(image, point_int, 1, cv::Scalar(0, 255, 0), -1);
  }
  cv::imshow("Görüntü", image);
  cv::waitKey(1);
  }
  catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge hatasi: %s", e.what());
    }
}

void AdaptiveCruiseControl::detection_callback(const vision_msgs::msg::Detection2DArray::SharedPtr msg)
{
  try
  {
    if (msg->detections.empty())
    {
      RCLCPP_INFO(this->get_logger(), "Detection mesaji bos!");
      integral_adaptive_ = 0.0;
      prev_error_adaptive_ = 0.0;
      bbox_received_ =false;
      return;
    }else
    {
      // RCLCPP_INFO(this->get_logger(), "Detection mesaji alindi!");
      // RCLCPP_INFO(this->get_logger(), "Detection id : %s",msg->detections[0].results[0].hypothesis.class_id.c_str());
      bbox_center_x_ = msg->detections[0].bbox.center.position.x;
      bbox_center_y_ = msg->detections[0].bbox.center.position.y;
      bbox_size_x_ = msg->detections[0].bbox.size_x;
      bbox_size_y_ = msg->detections[0].bbox.size_y;
      bbox_received_ = true;
    }

  }
  catch (std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Detection hatasi: %s", e.what());
    }
}

void AdaptiveCruiseControl::timer_callback()
{
  std_msgs::msg::Bool controller_flag_msg;
  controller_flag_msg.data = adaptive_cruise_controller_flag_;
  adaptive_cruise_controller_flag_publisher_->publish(controller_flag_msg);
  RCLCPP_INFO(this->get_logger(), "Adaptive cruise controller flag: %s", adaptive_cruise_controller_flag_ ? "true" : "false");
  // Distance calculation
  if(bbox_received_ == true && !distance_values.empty() && adaptive_cruise_controller_flag_ == true)
  {
    double sum = 0.0;
    for (const auto& distances : distance_values)
    {
      // bütün distance değerlerinin ortalamasını al
      sum += distances;
    }
    double average_distance = sum / distance_values.size();
    RCLCPP_INFO(this->get_logger(), "Mesafe: %f", average_distance);
    if (old_distance_initialized_)
    {
      relative_lead_vehicle_speed = (average_distance - old_distance_) / (timer_value_/1000);
      RCLCPP_INFO(this->get_logger(), "Lead vehicle speed: %f", relative_lead_vehicle_speed);
      relative_lead_vehicle_speed_flag_ = true; 
    }
    
    old_distance_ = average_distance;
    old_distance_initialized_ = true;
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "Mesafe hesaplanamadi! bbox bilgisi yok veya lidar nokta bulutu dusmuyor.");
    integral_adaptive_ = 0.0;
    prev_error_adaptive_ = 0.0;
    integral_ = 0.0;
    prev_error_ = 0.0;
    relative_lead_vehicle_speed_flag_ = false;
    relative_lead_vehicle_speed = 0.0;

  }

  if(adaptive_cruise_controller_flag_ == true  && relative_lead_vehicle_speed_flag_ == true && old_distance_ >= 5.0)
  {
    int throttle = adaptive_cruise_controller(relative_lead_vehicle_speed);
    std_msgs::msg::Float32 throttle_msg;
    throttle_msg.data = static_cast<float>(throttle);
    throttle_publisher_->publish(throttle_msg);
    RCLCPP_INFO(this->get_logger(), "Throttle published: %f", throttle_msg.data);
  }else
  {
    int throttle = 0.0;
    std_msgs::msg::Float32 throttle_msg;
    throttle_msg.data = static_cast<float>(throttle);
    throttle_publisher_->publish(throttle_msg);
    RCLCPP_INFO(this->get_logger(), "Throttle published: %f", throttle_msg.data);

  }
}

int AdaptiveCruiseControl::adaptive_cruise_controller(double relative_lead_vehicle_speed)
{
  RCLCPP_INFO(this->get_logger(), "Adaptive cruise controller calisiyor!");
  double dt = timer_value_ / 1000.0;
  double error = relative_lead_vehicle_speed;
  integral_adaptive_ += error * dt;
  double derivative = (dt > 0.0) ? (error - prev_error_adaptive_) / dt : 0.0;
  prev_error_adaptive_ = error;
  double output = kp_ * error + ki_ * integral_adaptive_ + kd_ * derivative;
  throttle = normalize_output(output);
  RCLCPP_INFO(this->get_logger(), "Adaptive throttle: %d", throttle);
  return throttle;
}

int AdaptiveCruiseControl::cruise_controller()
{
  RCLCPP_INFO(this->get_logger(), "Cruise controller calisiyor!");
  double dt = timer_value_ / 1000.0;
  double error = reference_speed_ - vehicle_speed_;
  RCLCPP_INFO(this->get_logger(), "error : %f", error);
  integral_ += error * dt;
  double derivative = (dt > 0.0) ? (error - prev_error_) / dt : 0.0;
  prev_error_ = error;
  RCLCPP_INFO(this->get_logger(), "kp * error : %f", kp_ * error);
  RCLCPP_INFO(this->get_logger(), "ki_ * integral_ : %f", ki_ * integral_);
  RCLCPP_INFO(this->get_logger(), "kd_ * derivative : %f", kd_ * derivative);
  double output = kp_ * error + ki_ * integral_ + kd_ * derivative;
  RCLCPP_INFO(this->get_logger(), "output : %f", output);
  throttle = normalize_output(output);
  RCLCPP_INFO(this->get_logger(), "Throttle: %d", throttle);
  return throttle;
}

int AdaptiveCruiseControl::normalize_output(double output)
{
  if (output > pid_max_) 
  {
    output = pid_max_;
  }
  if (output < pid_min_)
  {
    output = pid_min_;
  }

  return static_cast<int>((output - pid_min_) / (pid_max_ - pid_min_) * 100.0);
}

}


#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(adaptive_cruise_control::AdaptiveCruiseControl)
