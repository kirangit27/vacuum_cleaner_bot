/**
 * @file vc_bot.hpp
 * @author  Kiran S Patil (kpatil27@umd.edu)
 * @brief   Header file for the WalkerBot class.
 * @version 0.1
 * @date 2023-11-29
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#pragma once

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

/**
 * @brief Enum representing different states of the WalkerBot.
 */
enum class StateType {
  MOVE = 0, /**< Move state */
  STOP,     /**< Stop state */
  ROTATE    /**< Rotate state */
};

/**
 * @brief WalkerBot class represents a simple robot that can navigate avoiding obstacles.
 */
class WalkerBot : public rclcpp::Node {
 public:
  /**
   * @brief Constructor for the WalkerBot class.
   */
  WalkerBot();

 private:
  /**
   * @brief Callback function for the laser scan subscriber.
   * @param msg The laser scan message received from the sensor.
   */
  void subscribeCallback(const LaserScanMsg& msg);

  /**
   * @brief Timer callback function, called periodically to control the robot's behavior.
   */
  void timerCallback();

  /**
   * @brief Detects if there is an obstacle based on the current laser scan data.
   * @return True if an obstacle is detected, false otherwise.
   */
  bool detectObstacle();

  rclcpp::Subscription<LaserScanMsg>::SharedPtr subscriber_; /**< Laser scan subscriber */
  rclcpp::Publisher<TwistMsg>::SharedPtr publisher_;         /**< Twist message publisher */
  rclcpp::TimerBase::SharedPtr timer_;                        /**< Timer for periodic tasks */
  LaserScanMsg currentScan;                                    /**< Current laser scan data */
  StateType state;                                             /**< Current state of the robot */
};


