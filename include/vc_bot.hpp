#include <algorithm>
#include <cmath>
#include <cstdio>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

using std::placeholders::_1;

using LaserScanMsg = sensor_msgs::msg::LaserScan;
using TwistMsg = geometry_msgs::msg::Twist;

using namespace std::chrono_literals;

enum class StateType {
  MOVE = 0,
  STOP,
  ROTATE,
};

class WalkerBot : public rclcpp::Node {
 public:
  WalkerBot() : Node("vc_bot"), state(StateType::STOP) {
    auto cmdVelTopicName = "cmd_vel";
    publisher_ = this->create_publisher<TwistMsg>(cmdVelTopicName, 10);

    auto defaultQoS = rclcpp::QoS(rclcpp::SensorDataQoS());

    auto laserScanTopicName = "/scan";
    auto subCallback = std::bind(&WalkerBot::subscribeCallback, this, _1);
    subscriber_ = this->create_subscription<LaserScanMsg>(
        laserScanTopicName, defaultQoS, subCallback);

    auto timerCallback = std::bind(&WalkerBot::timerCallback, this);
    timer_ = this->create_wall_timer(100ms, timerCallback);
  }

 private:

  void subscribeCallback(const LaserScanMsg& msg);

  void timerCallback();

  bool detectObstacle();

  rclcpp::Subscription<LaserScanMsg>::SharedPtr subscriber_;
  rclcpp::Publisher<TwistMsg>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  LaserScanMsg currentScan;
  StateType state;
};


