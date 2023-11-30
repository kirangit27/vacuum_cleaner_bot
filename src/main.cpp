/**
 * @file main.cpp
 * @author  Kiran S Patil (kpatil27@umd.edu)
 * @brief  Main file for the WalkerBot ROS 2 node.
 * @version 0.1
 * @date 2023-11-29
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "../include/vc_bot.hpp"

/**
 * @brief Main function to run the WalkerBot ROS 2 node.
 * @param argc Number of command-line arguments.
 * @param argv Array of command-line argument strings.
 * @return 0 on successful execution.
 */
int main(int argc, char** argv) {
  // Initialize ROS 2
  rclcpp::init(argc, argv);

  // Create an instance of the WalkerBot node
  auto walkerBotNode = std::make_shared<WalkerBot>();

  // Spin the node to process callbacks
  rclcpp::spin(walkerBotNode);

  // Shutdown ROS 2
  rclcpp::shutdown();

  return 0;
}
