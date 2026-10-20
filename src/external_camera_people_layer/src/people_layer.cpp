#include "external_camera_people_layer/people_layer.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/expand_topic_or_service_name.hpp"

namespace external_camera_people_layer
{
using nav2_costmap_2d::NO_INFORMATION;

void PeopleLayer::Bounds::include(double x0, double y0, double x1, double y1)
{
  if (!valid) {
    min_x = x0; min_y = y0; max_x = x1; max_y = y1;
    valid = true;
  } else {
    min_x = std::min(min_x, x0); min_y = std::min(min_y, y0);
    max_x = std::max(max_x, x1); max_y = std::max(max_y, y1);
  }
}

void PeopleLayer::Bounds::extend(double * x0, double * y0, double * x1, double * y1) const
{
  if (valid) {
    *x0 = std::min(*x0, min_x); *y0 = std::min(*y0, min_y);
    *x1 = std::max(*x1, max_x); *y1 = std::max(*y1, max_y);
  }
}

PeopleLayer::Settings PeopleLayer::changedSettings(
  const std::vector<rclcpp::Parameter> & parameters) const
{
  auto result = settings_;
  for (const auto & p : parameters) {
    if (p.get_name() == name_ + ".enabled") {result.enabled = p.as_bool();}
    if (p.get_name() == name_ + ".people_topic") {result.topic = p.as_string();}
    if (p.get_name() == name_ + ".radius") {result.radius = p.as_double();}
    if (p.get_name() == name_ + ".max_cost") {result.max_cost = p.as_int();}
    if (p.get_name() == name_ + ".timeout") {result.timeout = p.as_double();}
  }
  if (!std::isfinite(result.radius) || result.radius <= 0.0 ||
    !std::isfinite(result.timeout) || result.timeout <= 0.0 ||
    result.max_cost < 1 || result.max_cost > 252 || result.topic.empty())
  {
    throw std::invalid_argument("radius/timeout must be finite and > 0; max_cost 1..252; topic nonempty");
  }
  const auto node = node_.lock();
  if (!node) {throw std::runtime_error("Costmap node expired");}
  rclcpp::expand_topic_or_service_name(result.topic, node->get_name(), node->get_namespace(), false);
  return result;
}

void PeopleLayer::onInitialize()
{
  const auto node = node_.lock();
  if (!node) {throw std::runtime_error("Costmap node expired");}
  global_frame_ = layered_costmap_->getGlobalFrameID();
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("people_topic", rclcpp::ParameterValue(settings_.topic));
  declareParameter("radius", rclcpp::ParameterValue(settings_.radius));
  declareParameter("max_cost", rclcpp::ParameterValue(settings_.max_cost));
  declareParameter("timeout", rclcpp::ParameterValue(settings_.timeout));
  std::vector<rclcpp::Parameter> parameters;
  for (const auto * key : {"enabled", "people_topic", "radius", "max_cost", "timeout"}) {
    parameters.push_back(node->get_parameter(name_ + "." + key));
  }
  settings_ = changedSettings(parameters);
  enabled_ = settings_.enabled;
  current_ = true;  // Missing people data means no extra cost, never block the planner.
  setDefaultValue(NO_INFORMATION);
  matchSize();
  subscribeLocked();
  validate_handle_ = node->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & params) {
      rcl_interfaces::msg::SetParametersResult result;
      std::lock_guard<std::mutex> lock(state_mutex_);
      try {
        changedSettings(params);
        result.successful = true;
      } catch (const std::exception & e) {
        result.successful = false;
        result.reason = e.what();
      }
      return result;
    });
  // Apply only after the complete parameter transaction has succeeded.
  apply_handle_ = node->add_post_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & params) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      const auto next = changedSettings(params);
      const bool resubscribe = next.enabled != settings_.enabled || next.topic != settings_.topic;
      settings_ = next;
      if (resubscribe) {
        clearInputLocked();
        subscribeLocked();
      }
    });
  RCLCPP_INFO(logger_, "%s: full PoseArray snapshots on %s, frame=%s, finite maximum=%ld",
    name_.c_str(), settings_.topic.c_str(), global_frame_.c_str(), settings_.max_cost);
}

void PeopleLayer::clearInputLocked()
{
  people_.clear();
  received_.reset();
}

void PeopleLayer::subscribeLocked()
{
  const auto token = ++subscription_generation_;
  subscription_.reset();
  if (!active_ || !settings_.enabled) {return;}
  const auto node = node_.lock();
  if (!node) {return;}
  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;
  subscription_ = node->create_subscription<geometry_msgs::msg::PoseArray>(
    settings_.topic, rclcpp::QoS(1).reliable().durability_volatile(),
    [this, token](geometry_msgs::msg::PoseArray::ConstSharedPtr msg) {receive(*msg, token);}, options);
}

void PeopleLayer::receive(const geometry_msgs::msg::PoseArray & msg, uint64_t token)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (token != subscription_generation_ || !active_ || !settings_.enabled) {return;}
  if (msg.header.frame_id != global_frame_) {
    clearInputLocked();
    RCLCPP_WARN_THROTTLE(logger_, warning_clock_, 5000,
      "%s: reject frame '%s', expected '%s'; no implicit TF", name_.c_str(),
      msg.header.frame_id.c_str(), global_frame_.c_str());
    return;
  }
  people_.clear();
  for (const auto & pose : msg.poses) {
    const auto & p = pose.position;
    if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) {
      people_.emplace_back(p.x, p.y);
    } else {
      RCLCPP_WARN_THROTTLE(logger_, warning_clock_, 5000, "%s: ignore non-finite position", name_.c_str());
    }
  }
  received_ = Steady::now();  // Intentionally ignore msg.header.stamp and /clock.
}

void PeopleLayer::matchSize()
{
  std::lock_guard<Costmap2D::mutex_t> lock(*getMutex());
  CostmapLayer::matchSize();
  resetMaps();
  // Retain previous_ until updateBounds has reported the old world region.
}

void PeopleLayer::updateBounds(
  double, double, double, double * min_x, double * min_y, double * max_x, double * max_y)
{
  std::lock_guard<Costmap2D::mutex_t> lock(*getMutex());
  const auto master = layered_costmap_->getCostmap();
  if (getOriginX() != master->getOriginX() || getOriginY() != master->getOriginY()) {
    CostmapLayer::matchSize();
  }
  Settings settings;
  std::vector<std::pair<double, double>> people;
  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    settings = settings_;
    enabled_ = settings.enabled;
    if (received_ && std::chrono::duration<double>(Steady::now() - *received_).count() >= settings.timeout) {
      clearInputLocked();
    }
    if (active_ && settings.enabled) {people = people_;}
  }
  // A complete small global-layer reset is deliberate: no accumulated ghost cells.
  resetMaps();
  previous_.extend(min_x, min_y, max_x, max_y);
  Bounds next;
  if (getSizeInCellsX() == 0 || getSizeInCellsY() == 0) {previous_ = next; return;}
  const double ox = getOriginX(), oy = getOriginY(), resolution = getResolution();
  const double right = ox + getSizeInCellsX() * resolution;
  const double top = oy + getSizeInCellsY() * resolution;
  for (const auto & p : people) {
    // Check doubles BEFORE Costmap2D's integer conversion (also handles huge finite input).
    if (p.first < ox || p.first >= right || p.second < oy || p.second >= top) {
      RCLCPP_WARN_THROTTLE(logger_, warning_clock_, 5000, "%s: ignore position outside costmap", name_.c_str());
      continue;
    }
    const double x0 = std::max(ox, p.first - settings.radius);
    const double y0 = std::max(oy, p.second - settings.radius);
    const double x1 = std::min(right, p.first + settings.radius);
    const double y1 = std::min(top, p.second + settings.radius);
    next.include(x0, y0, x1, y1);
    int i0, j0, i1, j1;
    worldToMapEnforceBounds(x0, y0, i0, j0);
    worldToMapEnforceBounds(x1, y1, i1, j1);
    // Nav2's enforce-bounds conversion returns size at the exact upper edge.
    i0 = std::clamp(i0, 0, static_cast<int>(getSizeInCellsX()) - 1);
    i1 = std::clamp(i1, 0, static_cast<int>(getSizeInCellsX()) - 1);
    j0 = std::clamp(j0, 0, static_cast<int>(getSizeInCellsY()) - 1);
    j1 = std::clamp(j1, 0, static_cast<int>(getSizeInCellsY()) - 1);
    for (int j = j0; j <= j1; ++j) {
      for (int i = i0; i <= i1; ++i) {
        double x, y;
        mapToWorld(i, j, x, y);
        const double distance = std::hypot(x - p.first, y - p.second);
        if (distance >= settings.radius) {continue;}
        const auto cost = static_cast<unsigned char>(std::max(1.0,
          std::round(settings.max_cost * (1.0 - distance / settings.radius))));
        const auto old = getCost(i, j);
        setCost(i, j, old == NO_INFORMATION ? cost : std::max(old, cost));
      }
    }
  }
  next.extend(min_x, min_y, max_x, max_y);
  previous_ = next;
  current_ = true;
}

void PeopleLayer::updateCosts(nav2_costmap_2d::Costmap2D & master, int x0, int y0, int x1, int y1)
{
  std::lock_guard<Costmap2D::mutex_t> lock(*getMutex());
  // The master window is recomposed by Nav2, not erased here. Unknown stays unknown.
  updateWithMaxWithoutUnknownOverwrite(master, x0, y0, x1, y1);
}

void PeopleLayer::changeActive(bool active)
{
  std::lock_guard<Costmap2D::mutex_t> master_lock(*layered_costmap_->getCostmap()->getMutex());
  std::lock_guard<Costmap2D::mutex_t> raster_lock(*getMutex());
  std::lock_guard<std::mutex> state_lock(state_mutex_);
  active_ = active;
  clearInputLocked();
  resetMaps();
  subscribeLocked();
  // Do not erase previous_: the next composition must restore other layers there.
}

void PeopleLayer::activate() {changeActive(true);}
void PeopleLayer::deactivate() {changeActive(false);}
void PeopleLayer::reset()
{
  std::lock_guard<Costmap2D::mutex_t> master_lock(*layered_costmap_->getCostmap()->getMutex());
  std::lock_guard<Costmap2D::mutex_t> raster_lock(*getMutex());
  std::lock_guard<std::mutex> state_lock(state_mutex_);
  clearInputLocked();
  resetMaps();
  subscribeLocked();
  current_ = true;
}

}  // namespace external_camera_people_layer

PLUGINLIB_EXPORT_CLASS(external_camera_people_layer::PeopleLayer, nav2_costmap_2d::Layer)
