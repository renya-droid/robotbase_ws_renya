#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <thread>
#include <vector>

#include "external_camera_people_layer/people_layer.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_navfn_planner/navfn.hpp"

using namespace std::chrono_literals;
using nav2_costmap_2d::FREE_SPACE;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;
using external_camera_people_layer::PeopleLayer;

// This baseline only requests bounds once. Later restoration therefore depends
// on PeopleLayer reporting BOTH its previous and new rectangles.
class Baseline : public nav2_costmap_2d::Layer
{
public:
  bool first{true};
  void onInitialize() override {current_ = enabled_ = true;}
  void reset() override {first = true;}
  bool isClearable() override {return false;}
  void updateBounds(double, double, double, double * x0, double * y0, double * x1, double * y1) override
  {
    if (first) {
      *x0 = -2; *y0 = -2; *x1 = 8; *y1 = 6;
      first = false;
    }
  }
  void updateCosts(nav2_costmap_2d::Costmap2D & master, int x0, int y0, int x1, int y1) override
  {
    for (int j = y0; j < y1; ++j) {
      for (int i = x0; i < x1; ++i) {
        master.setCost(i, j, i == 50 && j == 50 ? LETHAL_OBSTACLE :
          (i == 80 && j == 60 ? NO_INFORMATION : FREE_SPACE));
      }
    }
  }
};

class PeopleLayerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
  void SetUp() override
  {
    node = std::make_shared<nav2_util::LifecycleNode>("people_layer_test");
    sender = std::make_shared<rclcpp::Node>("people_sender_test");
    layers = std::make_unique<nav2_costmap_2d::LayeredCostmap>("map", false, true);
    layers->resizeMap(100, 80, .1, -2., -2.);
    auto base = std::make_shared<Baseline>();
    base->initialize(layers.get(), "baseline", nullptr, node, nullptr);
    layers->addPlugin(base);
    layer = std::make_shared<PeopleLayer>();
    layer->initialize(layers.get(), "people", nullptr, node, nullptr);
    layers->addPlugin(layer);
    pub = sender->create_publisher<geometry_msgs::msg::PoseArray>("/external_camera/people", rclcpp::QoS(1).reliable());
    executor.add_node(node->get_node_base_interface());
    executor.add_node(sender);
    waitForSubscriber();
    update();
  }
  void TearDown() override
  {
    executor.remove_node(sender);
    executor.remove_node(node->get_node_base_interface());
    pub.reset();
    layer.reset();
    layers.reset();
    sender.reset();
    node.reset();
  }
  void pump(std::chrono::milliseconds duration = 40ms)
  {
    const auto until = std::chrono::steady_clock::now() + duration;
    do {executor.spin_some(); std::this_thread::sleep_for(1ms);} while (std::chrono::steady_clock::now() < until);
  }
  void waitForSubscriber()
  {
    for (int i = 0; i < 100 && pub->get_subscription_count() == 0; ++i) {pump(20ms);}
    ASSERT_GT(pub->get_subscription_count(), 0u);
  }
  void send(const std::vector<std::pair<double, double>> & people, std::string frame = "map")
  {
    geometry_msgs::msg::PoseArray msg;
    msg.header.frame_id = frame;
    msg.header.stamp.sec = 1750000000;  // Wall epoch deliberately unrelated to simulated /clock.
    for (const auto & p : people) {
      geometry_msgs::msg::Pose pose;
      pose.position.x = p.first; pose.position.y = p.second; pose.orientation.w = 1.;
      msg.poses.push_back(pose);
    }
    waitForSubscriber();
    pub->publish(msg);
    pump();
    update();
  }
  void update() {layers->updateMap(0., 0., 0.);}
  unsigned char cost(unsigned int x, unsigned int y) {return layers->getCostmap()->getCost(x, y);}
  void set(std::string key, const rclcpp::ParameterValue & value)
  {
    const auto result = node->set_parameter(rclcpp::Parameter("people."+key, value));
    ASSERT_TRUE(result.successful) << result.reason;
  }
  std::shared_ptr<nav2_util::LifecycleNode> node;
  rclcpp::Node::SharedPtr sender;
  std::unique_ptr<nav2_costmap_2d::LayeredCostmap> layers;
  std::shared_ptr<PeopleLayer> layer;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub;
  rclcpp::executors::SingleThreadedExecutor executor;
};

TEST_F(PeopleLayerTest, FiniteRadialMaxAndOverlap)
{
  send({{.05, .05}});
  EXPECT_EQ(cost(20, 20), 220);
  EXPECT_GT(cost(23, 20), cost(27, 20));
  EXPECT_GT(cost(27, 20), 0);
  EXPECT_EQ(cost(31, 20), 0);
  const int intermediate = cost(24, 20);
  send({{.05, .05}, {.85, .05}, {.05, .05}});
  EXPECT_EQ(cost(24, 20), intermediate);
  EXPECT_EQ(cost(20, 20), 220);
}

TEST_F(PeopleLayerTest, MovingAndEmptyRestoreOtherLayersAndInternalGrid)
{
  send({{.05, .05}, {3.05, 3.05}});
  EXPECT_EQ(cost(50, 50), LETHAL_OBSTACLE);
  send({{4.05, .05}});
  EXPECT_EQ(cost(20, 20), 0);
  EXPECT_EQ(layer->getCost(20, 20), NO_INFORMATION);
  EXPECT_EQ(cost(60, 20), 220);
  EXPECT_EQ(cost(50, 50), LETHAL_OBSTACLE);
  send({});
  EXPECT_EQ(cost(60, 20), 0);
  EXPECT_EQ(layer->getCost(60, 20), NO_INFORMATION);
  EXPECT_EQ(cost(50, 50), LETHAL_OBSTACLE);
}

TEST_F(PeopleLayerTest, UnknownIsNotMadeKnownAndBoundaryIsSafe)
{
  send({{6.05, 4.05}, {7.95, 5.95}, {-1.95, -1.95}});
  EXPECT_EQ(cost(80, 60), NO_INFORMATION);
  EXPECT_EQ(layer->getCost(80, 60), 220);
  EXPECT_GT(cost(99, 79), 0);
  EXPECT_GT(cost(0, 0), 0);
  send({});
  EXPECT_EQ(cost(99, 79), 0);
}

TEST_F(PeopleLayerTest, InvalidFrameAndNonfiniteAndOutsideAreRejected)
{
  send({{.05, .05}});
  send({{.05, .05}}, "camera");
  EXPECT_EQ(cost(20, 20), 0);
  send({{.05, .05}}, "");
  EXPECT_EQ(cost(20, 20), 0);
  send({{std::numeric_limits<double>::quiet_NaN(), 0},
      {std::numeric_limits<double>::infinity(), 0}, {1e300, 1e300},
      {8., 0}, {-2.01, 0}, {.05, .05}});
  EXPECT_EQ(cost(20, 20), 220);
  send({{1e300, -1e300}});
  EXPECT_EQ(cost(20, 20), 0);
}

TEST_F(PeopleLayerTest, SteadyTimeoutWhileRosClockIsPaused)
{
  ASSERT_TRUE(node->set_parameter(rclcpp::Parameter("use_sim_time", true)).successful);
  pump();  // TimeSource consumes the parameter event asynchronously.
  EXPECT_EQ(node->now().nanoseconds(), 0);
  set("timeout", rclcpp::ParameterValue(.2));
  send({{.05, .05}});
  EXPECT_GT(cost(20, 20), 0);
  std::this_thread::sleep_for(250ms);
  update();
  EXPECT_EQ(cost(20, 20), 0);
  EXPECT_EQ(node->now().nanoseconds(), 0);
  EXPECT_TRUE(layer->isCurrent());
}

TEST_F(PeopleLayerTest, DisableReenableResetAndLifecycleNeverReviveOldData)
{
  send({{.05, .05}});
  set("enabled", rclcpp::ParameterValue(false)); update();
  EXPECT_EQ(cost(20, 20), 0);
  set("enabled", rclcpp::ParameterValue(true)); pump(); update();
  EXPECT_EQ(cost(20, 20), 0);
  send({{.05, .05}});
  layer->reset(); update();
  EXPECT_EQ(cost(20, 20), 0);
  send({{.05, .05}});
  layer->deactivate(); update();
  EXPECT_EQ(cost(20, 20), 0);
  layer->activate(); pump(); update();
  EXPECT_EQ(cost(20, 20), 0);
  send({{.05, .05}});
  EXPECT_EQ(cost(20, 20), 220);
}

TEST_F(PeopleLayerTest, ParameterValidationAndKernelShrink)
{
  send({{.05, .05}});
  EXPECT_GT(cost(27, 20), 0);
  set("radius", rclcpp::ParameterValue(.3)); update();
  EXPECT_EQ(cost(27, 20), 0);
  set("max_cost", rclcpp::ParameterValue(int64_t(100))); update();
  EXPECT_EQ(cost(20, 20), 100);
  for (const auto & p : {rclcpp::Parameter("people.max_cost", 253),
      rclcpp::Parameter("people.radius", -1.0), rclcpp::Parameter("people.timeout", 0.),
      rclcpp::Parameter("people.people_topic", "bad topic")})
  {
    EXPECT_FALSE(node->set_parameter(p).successful);
  }
  update();
  EXPECT_EQ(cost(20, 20), 100);
}

TEST_F(PeopleLayerTest, NewSnapshotBetweenBoundsAndCostsWaitsForNextCycle)
{
  send({{.05, .05}});
  double x0 = 1e6, y0 = 1e6, x1 = -1e6, y1 = -1e6;
  layer->updateBounds(0, 0, 0, &x0, &y0, &x1, &y1);
  geometry_msgs::msg::PoseArray msg;
  msg.header.frame_id = "map";
  pub->publish(msg); pump();
  nav2_costmap_2d::Costmap2D snapshot(100, 80, .1, -2., -2., 0);
  layer->updateCosts(snapshot, 0, 0, 100, 80);
  EXPECT_EQ(snapshot.getCost(20, 20), 220);  // Bounds/costs share the prepared raster.
  update();
  EXPECT_EQ(cost(20, 20), 0);
}

TEST_F(PeopleLayerTest, TopicChangeClearsAndRejectsTheOldSource)
{
  send({{.05, .05}});
  set("people_topic", rclcpp::ParameterValue("/other_camera_people"));
  update();
  EXPECT_EQ(cost(20, 20), 0);
  geometry_msgs::msg::PoseArray old;
  old.header.frame_id = "map";
  geometry_msgs::msg::Pose p;
  p.position.x = .05; p.position.y = .05;
  old.poses.push_back(p);
  pub->publish(old); pump(); update();
  EXPECT_EQ(cost(20, 20), 0);
  pub = sender->create_publisher<geometry_msgs::msg::PoseArray>("/other_camera_people", rclcpp::QoS(1).reliable());
  send({{.05, .05}});
  EXPECT_EQ(cost(20, 20), 220);
}

TEST_F(PeopleLayerTest, ConcurrentReceivesAndCostmapUpdates)
{
  std::thread receiver([this]() {executor.spin();});
  for (int i = 0; i < 80; ++i) {
    geometry_msgs::msg::PoseArray msg;
    msg.header.frame_id = "map";
    geometry_msgs::msg::Pose p;
    p.position.x = .05 + .01 * i; p.position.y = .05;
    msg.poses.push_back(p);
    pub->publish(msg);
    update();
    if (i % 10 == 0) {layer->reset();}
    std::this_thread::sleep_for(1ms);
  }
  executor.cancel(); receiver.join();
  layer->reset(); update();
  EXPECT_EQ(cost(20, 20), 0);
  EXPECT_EQ(cost(50, 50), LETHAL_OBSTACLE);
}

TEST_F(PeopleLayerTest, ActualNavfnFiniteCostsEncourageDetourAndClearRestoresRoute)
{
  auto plan_distance = [this]() {
      nav2_navfn_planner::NavFn planner(100, 80);
      planner.setCostmap(layers->getCostmap()->getCharMap(), true, true);
      int start[2]{10, 20}, goal[2]{60, 20};
      planner.setStart(start); planner.setGoal(goal);
      EXPECT_TRUE(planner.calcNavFnAstar([]() {return false;}));
      EXPECT_GT(planner.calcPath(1000), 0);
      double minimum = 1e9;
      for (int i = 0; i < planner.getPathLen(); ++i) {
        minimum = std::min(minimum, std::hypot(planner.getPathX()[i] - 35., planner.getPathY()[i] - 20.) * .1);
      }
      return minimum;
    };
  const double baseline = plan_distance();
  send({{1.55, .05}});
  const double detour = plan_distance();
  EXPECT_GT(detour, baseline + .4);
  send({});
  EXPECT_NEAR(plan_distance(), baseline, .05);
}
