#include "robotbase_costmap_plugins/hokuyo_obstacle_layer.hpp"

#include <algorithm>
#include <mutex>
#include "pluginlib/class_list_macros.hpp"

namespace robotbase_costmap_plugins
{
void HokuyoObstacleLayer::onInitialize()
{
  ObstacleLayer::onInitialize();
  auto node = node_.lock();
  // Volatile: a new costmap must never inherit an old disable command.
  disable_subscription_ = node->create_subscription<std_msgs::msg::Bool>(
    "/waypoint_follow/hokuyo_disable", rclcpp::QoS(1),
    [this](std_msgs::msg::Bool::ConstSharedPtr msg) {receiveDisable(msg->data);});
}

void HokuyoObstacleLayer::receiveDisable(bool disabled)
{
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> guard(*getMutex());
  disable_until_ = disabled ? leaseNow() + std::chrono::seconds(1) : Clock::time_point{};
}

void HokuyoObstacleLayer::updateBounds(double robot_x, double robot_y, double robot_yaw,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> guard(*getMutex());
  const bool suspend = leaseNow() < disable_until_;
  const bool changed = suspend != suspended_;
  suspended_ = suspend;
  if (changed) {
    // Remove only Hokuyo's stored cells, including marks behind the scanner.
    // Invalidate the entire window so the master and inflation also lose those marks.
    resetMaps();
    *min_x = std::min(*min_x, getOriginX());
    *min_y = std::min(*min_y, getOriginY());
    *max_x = std::max(*max_x, getOriginX() + getSizeInMetersX());
    *max_y = std::max(*max_y, getOriginY() + getSizeInMetersY());
    RCLCPP_INFO(logger_, "Hokuyo costmap: %s", suspend ? "DISABLED" : "ENABLED");
  }
  if (suspended_) {
    if (rolling_window_) {
      updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
    }
    current_ = true;
    return;
  }
  ObstacleLayer::updateBounds(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
}

void HokuyoObstacleLayer::updateCosts(nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> guard(*getMutex());
  if (!suspended_) {
    ObstacleLayer::updateCosts(master_grid, min_i, min_j, max_i, max_j);
  }
}
}  // namespace robotbase_costmap_plugins

PLUGINLIB_EXPORT_CLASS(robotbase_costmap_plugins::HokuyoObstacleLayer, nav2_costmap_2d::Layer)
