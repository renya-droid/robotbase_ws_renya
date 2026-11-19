#include <gtest/gtest.h>
#include <memory>
#include <thread>
#include "robotbase_costmap_plugins/hokuyo_obstacle_layer.hpp"
#include "nav2_costmap_2d/inflation_layer.hpp"
#include "pluginlib/class_loader.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

class TestLayer : public robotbase_costmap_plugins::HokuyoObstacleLayer
{
public:
  void command(bool disabled) {receiveDisable(disabled);}
  void elapse(int milliseconds) {now_ += std::chrono::milliseconds(milliseconds);}
  bool suspended() const {return suspended_;}
protected:
  Clock::time_point leaseNow() const override {return now_;}
  Clock::time_point now_{std::chrono::seconds(10)};
};

class HokuyoLayerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    node = std::make_shared<nav2_util::LifecycleNode>("hokuyo_layer_test");
    node->declare_parameter("track_unknown_space", false);
    node->declare_parameter("transform_tolerance", 0.3);
    node->declare_parameter("inflation.inflation_radius", 0.5);
    tf = std::make_unique<tf2_ros::Buffer>(node->get_clock());
    map = std::make_unique<nav2_costmap_2d::LayeredCostmap>("map", false, false);
    map->resizeMap(100, 100, 0.1, -5.0, -5.0);
    hokuyo = std::make_shared<TestLayer>();
    velodyne = std::make_shared<nav2_costmap_2d::ObstacleLayer>();
    inflation = std::make_shared<nav2_costmap_2d::InflationLayer>();
    for (const auto & entry : std::vector<std::pair<std::string,
      std::shared_ptr<nav2_costmap_2d::Layer>>>{
      {"velodyne", velodyne}, {"hokuyo", hokuyo}, {"inflation", inflation}})
    {
      map->addPlugin(entry.second);
      entry.second->initialize(map.get(), entry.first, tf.get(), node,
        node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive));
    }
  }

  void addPoint(nav2_costmap_2d::ObstacleLayer & layer, float x, float y)
  {
    sensor_msgs::msg::PointCloud2 cloud;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(1);
    *sensor_msgs::PointCloud2Iterator<float>(cloud, "x") = x;
    *sensor_msgs::PointCloud2Iterator<float>(cloud, "y") = y;
    *sensor_msgs::PointCloud2Iterator<float>(cloud, "z") = 0.1;
    geometry_msgs::msg::Point origin;
    nav2_costmap_2d::Observation observation(origin, cloud, 10.0, 0.0, 10.0, 0.0);
    layer.addStaticObservation(observation, true, false);
  }

  unsigned char cost(double x, double y)
  {
    unsigned int mx, my;
    EXPECT_TRUE(map->getCostmap()->worldToMap(x, y, mx, my));
    return map->getCostmap()->getCost(mx, my);
  }

  std::shared_ptr<nav2_util::LifecycleNode> node;
  std::unique_ptr<tf2_ros::Buffer> tf;
  std::unique_ptr<nav2_costmap_2d::LayeredCostmap> map;
  std::shared_ptr<TestLayer> hokuyo;
  std::shared_ptr<nav2_costmap_2d::ObstacleLayer> velodyne;
  std::shared_ptr<nav2_costmap_2d::InflationLayer> inflation;
};

TEST_F(HokuyoLayerTest, ClearsHokuyoAndInflationButPreservesVelodyne)
{
  addPoint(*hokuyo, 1.0, 1.0);
  addPoint(*velodyne, -1.0, -1.0);
  map->updateMap(0.0, 0.0, 0.0);
  EXPECT_EQ(cost(1.0, 1.0), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_GT(cost(1.2, 1.0), nav2_costmap_2d::FREE_SPACE);
  hokuyo->command(true);
  map->updateMap(0.0, 0.0, 0.0);
  EXPECT_EQ(cost(1.0, 1.0), nav2_costmap_2d::FREE_SPACE);
  EXPECT_EQ(cost(1.2, 1.0), nav2_costmap_2d::FREE_SPACE);
  EXPECT_EQ(cost(-1.0, -1.0), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_TRUE(hokuyo->isCurrent());
  hokuyo->command(false);
  map->updateMap(0.0, 0.0, 0.0);
  EXPECT_EQ(cost(1.0, 1.0), nav2_costmap_2d::LETHAL_OBSTACLE);
}

TEST_F(HokuyoLayerTest, RestoresAfterMissingHeartbeatAndRenewsWhileAlive)
{
  addPoint(*hokuyo, 1.0, 1.0);
  hokuyo->command(true);
  map->updateMap(0.0, 0.0, 0.0);
  hokuyo->elapse(900);
  hokuyo->command(true);
  hokuyo->elapse(900);
  map->updateMap(0.0, 0.0, 0.0);
  EXPECT_TRUE(hokuyo->suspended());
  hokuyo->elapse(100);
  map->updateMap(0.0, 0.0, 0.0);
  EXPECT_FALSE(hokuyo->suspended());
  EXPECT_EQ(cost(1.0, 1.0), nav2_costmap_2d::LETHAL_OBSTACLE);
}

TEST_F(HokuyoLayerTest, ReceivesVolatileHeartbeatOverROS)
{
  auto sender = std::make_shared<rclcpp::Node>("hokuyo_lease_test_sender");
  auto publisher = sender->create_publisher<std_msgs::msg::Bool>(
    "/waypoint_follow/hokuyo_disable", 1);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.add_node(sender);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (!hokuyo->suspended() && std::chrono::steady_clock::now() < deadline) {
    std_msgs::msg::Bool message;
    message.data = true;
    publisher->publish(message);
    executor.spin_some();
    map->updateMap(0.0, 0.0, 0.0);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_TRUE(hokuyo->suspended());
  hokuyo->elapse(1000);
  map->updateMap(0.0, 0.0, 0.0);
  EXPECT_FALSE(hokuyo->suspended());
}

TEST_F(HokuyoLayerTest, LoadsThroughNav2PluginRegistry)
{
  pluginlib::ClassLoader<nav2_costmap_2d::Layer> loader(
    "nav2_costmap_2d", "nav2_costmap_2d::Layer");
  auto instance = loader.createSharedInstance("robotbase_costmap_plugins::HokuyoObstacleLayer");
  EXPECT_NE(instance, nullptr);
}
