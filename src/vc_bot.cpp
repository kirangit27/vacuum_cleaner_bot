/**
 * @file vc_bot.cpp
 * @author Kiran S Patil (kpatil27@umd.edu)
 * @brief Implementation file for the WalkerBot class.
 * @version 0.1
 * @date 2023-11-29
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "../include/vc_bot.hpp"

/**
 * @brief Callback function for the laser scan subscriber.
 * @param msg The laser scan message received from the sensor.
 */
void WalkerBot::subscribeCallback(const LaserScanMsg& msg) {
    currentScan = msg;
}

/**
 * @brief Timer callback function, called periodically to control the robot's behavior.
 */
void WalkerBot::timerCallback() {
    if (currentScan.header.stamp.sec == 0) {
        return;
    }

    auto pubMsg = TwistMsg();

    switch (state) {
        case StateType::STOP:
            if (detectObstacle()) {
                state = StateType::ROTATE;
                pubMsg.angular.z = -0.3;
                publisher_->publish(pubMsg);
                RCLCPP_INFO_STREAM(this->get_logger(), "STOP state");
            } else {
                state = StateType::MOVE;
                pubMsg.linear.x = 0.3;
                publisher_->publish(pubMsg);
                RCLCPP_INFO_STREAM(this->get_logger(), "MOVE state");
            }
            break;

        case StateType::MOVE:
            if (detectObstacle()) {
                state = StateType::STOP;
                pubMsg.linear.x = 0;
                publisher_->publish(pubMsg);
                RCLCPP_INFO_STREAM(this->get_logger(), "MOVE state");
            }
            break;

        case StateType::ROTATE:
            if (!detectObstacle()) {
                state = StateType::MOVE;
                pubMsg.linear.x = 0.1;
                publisher_->publish(pubMsg);
                RCLCPP_INFO_STREAM(this->get_logger(), "ROTATE state");
            }
            break;
    }
}

/**
 * @brief Detects if there is an obstacle based on the current laser scan data.
 * @return True if an obstacle is detected, false otherwise.
 */
bool WalkerBot::detectObstacle() {
    for (size_t i = 0; i < currentScan.ranges.size(); ++i) {
        if (currentScan.ranges[i] > currentScan.range_min &&
            currentScan.ranges[i] < currentScan.range_max) {
            RCLCPP_INFO(this->get_logger(), "Distance: %f is valid",
                        currentScan.ranges[i]);
            if (currentScan.ranges[i] < 0.3) {
                RCLCPP_INFO(this->get_logger(), "Obstacle detected, rotating");
                return true;
            }
        }
    }
    return false;
}