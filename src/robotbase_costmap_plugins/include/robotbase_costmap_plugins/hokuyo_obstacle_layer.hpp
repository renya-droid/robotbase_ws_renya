#pragma once

#include <chrono>
#include "nav2_costmap_2d/obstacle_layer.hpp"
#include "std_msgs/msg/bool.hpp"

namespace robotbase_costmap_plugins
{
class HokuyoObstacleLayer : public nav2_costmap_2d::ObstacleLayer
{
public:
  void onInitialize() override;
  void updateBounds(double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  void updateCosts(nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;

protected:
  using Clock = std::chrono::steady_clock;
  // Steady time ensures paused /clock or a changed system clock cannot prolong the lease.
  virtual Clock::time_point leaseNow() const {return Clock::now();}
  void receiveDisable(bool disabled);
  Clock::time_point disable_until_{};
  bool suspended_{false};
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr disable_subscription_;
};
}  // namespace robotbase_costmap_plugins
