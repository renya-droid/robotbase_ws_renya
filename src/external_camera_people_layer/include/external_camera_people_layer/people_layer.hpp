#ifndef EXTERNAL_CAMERA_PEOPLE_LAYER__PEOPLE_LAYER_HPP_
#define EXTERNAL_CAMERA_PEOPLE_LAYER__PEOPLE_LAYER_HPP_

#include <chrono>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_array.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"

namespace external_camera_people_layer
{

class PeopleLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  void onInitialize() override;
  void matchSize() override;
  void updateBounds(double, double, double, double *, double *, double *, double *) override;
  void updateCosts(nav2_costmap_2d::Costmap2D &, int, int, int, int) override;
  void reset() override;
  void activate() override;
  void deactivate() override;
  bool isClearable() override {return false;}  // Full reset() supported; no partial sensor clearing.

private:
  using Steady = std::chrono::steady_clock;
  struct Settings
  {
    bool enabled{true};
    std::string topic{"/external_camera/people"};
    double radius{1.0};
    int64_t max_cost{220};
    double timeout{1.5};
  };
  struct Bounds
  {
    bool valid{false};
    double min_x{}, min_y{}, max_x{}, max_y{};
    void include(double x0, double y0, double x1, double y1);
    void extend(double *, double *, double *, double *) const;
  };

  Settings changedSettings(const std::vector<rclcpp::Parameter> &) const;
  void subscribeLocked();  // state_mutex_ held; callbacks never acquire a costmap mutex.
  void receive(const geometry_msgs::msg::PoseArray &, uint64_t generation);
  void clearInputLocked();
  void changeActive(bool active);

  // Lock order: master costmap -> internal costmap -> state. Subscription/parameter
  // callbacks take only state; they never mutate raster cells or update bounds.
  std::mutex state_mutex_;
  Settings settings_;
  bool active_{true};
  uint64_t subscription_generation_{0};
  std::vector<std::pair<double, double>> people_;
  std::optional<Steady::time_point> received_;
  Bounds previous_;
  std::string global_frame_;
  rclcpp::Clock warning_clock_{RCL_STEADY_TIME};
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscription_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr validate_handle_;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr apply_handle_;
};

}  // namespace external_camera_people_layer
#endif
