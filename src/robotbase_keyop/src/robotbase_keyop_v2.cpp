#include <chrono>
#include <cmath>
#include <cstdio>
#include <functional>
#include <memory>
#include <stdexcept>
#include <thread>

#include <termios.h>
#include <unistd.h>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"

class RobotbaseKeyopV2 : public rclcpp::Node
{
public:
  RobotbaseKeyopV2()
  : Node("robotbase_keyop_v2"),
    last_input_time_(0, 0, RCL_ROS_TIME),
    last_release_time_(0, 0, RCL_ROS_TIME)
  {
    linear_step_ = declare_parameter("linear_step", 0.1);
    angular_step_ = declare_parameter("angular_step", 0.1);
    max_linear_velocity_ = declare_parameter("max_linear_velocity", 1.0);
    max_angular_velocity_ = declare_parameter("max_angular_velocity", 1.5);
    manual_override_grace_period_ =
      declare_parameter("manual_override_grace_period", 1.0);
    stop_release_duration_ = declare_parameter("stop_release_duration", 0.5);
    const auto reset_topic = declare_parameter("reset_topic", "/robotbase/reset");
    const auto pause_waypoint_topic =
      declare_parameter("pause_waypoint_topic", "/waypoint_record/pause");

    velocity_publisher_ =
      create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_teleop", 10);
    stop_publisher_ = create_publisher<std_msgs::msg::Bool>("/stop", 10);
    reset_publisher_ = create_publisher<std_msgs::msg::Empty>(reset_topic, 10);
    pause_waypoint_publisher_ =
      create_publisher<std_msgs::msg::Empty>(pause_waypoint_topic, 10);
    initial_pose_publisher_ =
      create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);

    last_input_time_ = now();
    last_release_time_ = now();
    timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&RobotbaseKeyopV2::timer_callback, this));

    RCLCPP_INFO(get_logger(), "Koko-chan keyboard teleoperation started");
    RCLCPP_INFO(get_logger(), "w/x: linear +/-   a/d: angular +/-");
    RCLCPP_INFO(get_logger(), "s: zero velocity and yield to Nav2");
    RCLCPP_INFO(get_logger(), "q/e: emergency stop / release");
    RCLCPP_INFO(get_logger(), "p: save 10-second pause waypoint");
    RCLCPP_INFO(get_logger(), "r: reset pose   Ctrl-C: quit");
  }

  void run()
  {
    while (rclcpp::ok()) {
      const char key = get_key();
      switch (key) {
        case 'q':
          stopped_ = true;
          linear_velocity_ = 0.0;
          angular_velocity_ = 0.0;
          last_input_time_ = now();
          std::printf("\n>>> EMERGENCY STOP (LOCKED) <<<\n");
          break;
        case 'e':
          stopped_ = false;
          last_input_time_ = now();
          std::printf("\n>>> STOP RELEASED (UNLOCKED) <<<\n");
          break;
        case 'r':
          reset_system();
          break;
        case 'p':
          pause_waypoint_publisher_->publish(std_msgs::msg::Empty());
          if (pause_waypoint_publisher_->get_subscription_count() == 0) {
            std::printf("\n>>> WAYPOINT RECORDER IS NOT RUNNING <<<\n");
          } else {
            std::printf("\n>>> 10-SECOND PAUSE WAYPOINT REQUESTED <<<\n");
          }
          break;
        case 'w':
          linear_velocity_ += linear_step_;
          last_input_time_ = now();
          print_status();
          break;
        case 'x':
          linear_velocity_ -= linear_step_;
          last_input_time_ = now();
          print_status();
          break;
        case 'a':
          angular_velocity_ += angular_step_;
          last_input_time_ = now();
          print_status();
          break;
        case 'd':
          angular_velocity_ -= angular_step_;
          last_input_time_ = now();
          print_status();
          break;
        case 's':
          linear_velocity_ = 0.0;
          angular_velocity_ = 0.0;
          last_input_time_ = now();
          print_status();
          break;
        default:
          break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

private:
  void reset_system()
  {
    reset_publisher_->publish(std_msgs::msg::Empty());
    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = now();
    pose.pose.pose.orientation.w = 1.0;
    pose.pose.covariance[0] = 0.25;
    pose.pose.covariance[7] = 0.25;
    pose.pose.covariance[35] = 0.06;
    initial_pose_publisher_->publish(pose);

    linear_velocity_ = 0.0;
    angular_velocity_ = 0.0;
    stopped_ = false;
    last_input_time_ = now();
    std::printf("\n>>> SYSTEM RESET <<<\n");
  }

  void timer_callback()
  {
    const auto current_time = now();
    const bool has_manual_velocity =
      std::abs(linear_velocity_) > 0.01 || std::abs(angular_velocity_) > 0.01;
    bool within_grace_period = false;
    try {
      within_grace_period =
        (current_time - last_input_time_).seconds() < manual_override_grace_period_;
    } catch (const std::runtime_error &) {
      last_input_time_ = current_time;
    }

    if (has_manual_velocity || within_grace_period) {
      linear_velocity_ = std::clamp(
        linear_velocity_, -max_linear_velocity_, max_linear_velocity_);
      angular_velocity_ = std::clamp(
        angular_velocity_, -max_angular_velocity_, max_angular_velocity_);
      geometry_msgs::msg::Twist velocity;
      velocity.linear.x = linear_velocity_;
      velocity.angular.z = angular_velocity_;
      velocity_publisher_->publish(velocity);
    }

    if (stopped_) {
      std_msgs::msg::Bool stop;
      stop.data = true;
      stop_publisher_->publish(stop);
      last_stop_state_ = true;
    } else {
      if (last_stop_state_) {
        last_release_time_ = current_time;
        last_stop_state_ = false;
      }
      try {
        if ((current_time - last_release_time_).seconds() < stop_release_duration_) {
          std_msgs::msg::Bool release;
          release.data = false;
          stop_publisher_->publish(release);
        }
      } catch (const std::runtime_error &) {
        last_release_time_ = current_time;
      }
    }
  }

  void print_status() const
  {
    std::printf(
      "\rSmart Mode - Linear: %+.2f, Angular: %+.2f  [%s]  ",
      linear_velocity_, angular_velocity_, stopped_ ? "STOPPED" : "ACTIVE");
    std::fflush(stdout);
  }

  char get_key() const
  {
    termios old_settings {};
    if (tcgetattr(STDIN_FILENO, &old_settings) != 0) {
      return 0;
    }
    termios new_settings = old_settings;
    new_settings.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
    new_settings.c_cc[VMIN] = 0;
    new_settings.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &new_settings);
    char key = 0;
    (void)::read(STDIN_FILENO, &key, 1);
    tcsetattr(STDIN_FILENO, TCSANOW, &old_settings);
    return key;
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_publisher_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr reset_publisher_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pause_waypoint_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    initial_pose_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  double linear_step_ {0.1};
  double angular_step_ {0.1};
  double max_linear_velocity_ {1.0};
  double max_angular_velocity_ {1.5};
  double manual_override_grace_period_ {1.0};
  double stop_release_duration_ {0.5};
  double linear_velocity_ {0.0};
  double angular_velocity_ {0.0};
  bool stopped_ {false};
  bool last_stop_state_ {false};
  rclcpp::Time last_input_time_;
  rclcpp::Time last_release_time_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const auto node = std::make_shared<RobotbaseKeyopV2>();
  std::thread spin_thread([node]() {rclcpp::spin(node);});
  node->run();
  rclcpp::shutdown();
  spin_thread.join();
  return 0;
}

