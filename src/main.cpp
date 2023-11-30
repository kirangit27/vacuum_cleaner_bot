/**
 * @file main.cpp
 * @author  Kiran S Patil (kpatil27@umd.edu)
 * @brief 
 * @version 0.1
 * @date 2023-11-29
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include "../include/vc_bot.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WalkerBot>());
  rclcpp::shutdown();
  return 0;
}
