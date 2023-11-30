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
    publisher_ = this->create_publisher<TwistMsg>("cmd_vel", 10);
    subscriber_ = this->create_subscription<LaserScanMsg>(
        "/scan", rclcpp::QoS(rclcpp::SensorDataQoS()), 
        std::bind(&WalkerBot::subscribeCallback, this, _1));
    timer_ = this->create_wall_timer(100ms, std::bind(&WalkerBot::timerCallback, this));
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


