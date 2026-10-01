#ifndef ROBOTEQ_ROS2_DRIVER__ROBOTEQ_ROS2_DRIVER_HPP_
#define ROBOTEQ_ROS2_DRIVER__ROBOTEQ_ROS2_DRIVER_HPP_

#include <math.h>
#include <unistd.h>

#include <cstdio>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <deque>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/header.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/bool.hpp"


#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <thread>

namespace Roboteq
{
class Roboteq : public rclcpp::Node
{
  public:
  explicit Roboteq(); //(nodeOptions options?)
  ~Roboteq();

  private:
  // class atributes
  //rclcpp::Node::SharedPtr nh{};
  std::unique_ptr<tf2_ros::TransformBroadcaster> odom_baselink_transform_;
  std::thread odom_thread_;

  std::atomic_bool running_{false};
  std::mutex odom_mutex_;
  std::mutex command_mutex_;

  uint32_t starttime{};
  uint32_t hstimer{};
  uint32_t mstimer{};
  uint32_t lstimer{};
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr param_update_timer;

  // buffer for reading encoder counts
  unsigned int odom_idx{};
  char odom_buf[24]{};

  // toss out initial encoder readings
  char odom_encoder_toss{};

  int32_t odom_encoder_left{};
  int32_t odom_encoder_right{};

  struct EncoderSample
  {
    std::chrono::steady_clock::time_point stamp;
    int32_t right_count;
    int32_t left_count;
  };

  bool encoder_initialized_{};
  int32_t previous_encoder_left_{};
  int32_t previous_encoder_right_{};
  std::deque<EncoderSample> encoder_history_;
  std::chrono::steady_clock::time_point last_encoder_receive_time_{};
  double cumulative_wheel_revolutions_left_{};
  double cumulative_wheel_revolutions_right_{};
  double estimated_wheel_rpm_left_{};
  double estimated_wheel_rpm_right_{};
  double estimated_linear_velocity_{};
  double estimated_angular_velocity_{};
  double velocity_window_seconds_{};
  bool velocity_estimate_valid_{};

  float odom_x{};
  float odom_y{};
  float odom_yaw{};
  float odom_last_x{};
  float odom_last_y{};
  float odom_last_yaw{};

  float motor_param{};
  bool straight{};

  int pulse{};
  float right_rpm_command{};
  float left_rpm_command{};
  float linear_x{};
  float angular_z{};
  float max_speed{};

  uint32_t odom_last_time{};


  // settings
  bool pub_odom_tf{};
  std::string odom_frame{};
  std::string base_frame{};
  std::string cmdvel_topic{};
  std::string odom_topic{};
  std::string port{};
  int baud{};
  bool open_loop{};
  double wheel_circumference{};
  double track_width{};
  int encoder_ppr{};
  int encoder_cpr{};
  double max_amps{};
  int max_rpm{};
  // odom stream interval (milliseconds) configurable so users can tune
  // how frequently the Roboteq controller sends encoder data (default 50ms -> 20Hz)
  int odom_stream_interval_ms{};
  // last interval value sent to device — used to re-send stream command when
  // parameter changes at runtime
  int last_sent_odom_stream_ms{};
  // publish rate for /odom (Hz). This is the maximum publish frequency; the
  // actual rate will also depend on the controller stream.
  double odom_publish_hz{};
  // user-tunable scale correction applied to odometry distances — useful for
  // calibrating mismatch between encoder counts and physical distance.
  double odom_scale{};
  // speed scale factor to calibrate commanded speed to match actual robot speed
  double speed_scale{};
  float gear_ratio{};
  
  // ソフトウェアP制御用のゲイン
  double kp_soft{};
  double min_speed_threshold{};
  double encoder_sign_r{};
  double encoder_sign_l{};

  // Test different odom msg memory
  //nav_msgs::msg::Odometry odom_msg{};
  nav_msgs::msg::Odometry odom_msg{};
  //geometry_msgs::msg::Twist twist_msg{};



    //
  // cmd_vel subscriber
  //
  void cmdvel_callback(const geometry_msgs::msg::Twist::SharedPtr twist_msg);
  void cmdvel_setup();
  void cmdvel_loop();
  void cmdvel_run();

  void bumper_callback(const std_msgs::msg::Bool::SharedPtr stop_msg);

  //
  // odom publisher
  //
  void odom_setup();
  void odom_stream(); 
  void odom_loop();
  void process_encoder_sample(
    int32_t right_count,
    int32_t left_count,
    std::chrono::steady_clock::time_point stamp);
  static int64_t encoder_count_delta(int32_t current, int32_t previous);
  //void odom_hs_run();
  void odom_ms_run();
  void odom_ls_run();
  void odom_publish();
  void connect();
  bool safe_serial_write(const std::string &cmd);

  void update_parameters();
  int run();

  //subscriber
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdvel_sub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_sub;

  //publisher
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;



};

}





#endif
