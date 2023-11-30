# vacuum_cleaner_bot
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
---

### Dependencies:
- OS : Ubuntu 22.04 
- ROS2 Distro : ROS2 Humble
- Package build type : ```ament_cmake ```
- Package dependencies : ```rclcpp```, ```std_msgs``` 
- ROS2 Humble Installation : [link](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html)
- Turtlebot3 Simulation for ros2: [link](https://github.com/ROBOTIS-GIT/turtlebot3/tree/ros2)


### Instructions to build the ROS Package

- clone the repo in the ros2 workspace/src

```
# check for any dependencies
rosdep install -i --from-path src --rosdistro humble -y

# build the package
colcon build --packages-select vacuum_cleaner_bot
```

### Launch

```
ros2 launch vacuum_cleaner_bot vacuum_cleaner_bot.launch.py 
```

- The ros bag file will be saved in the results folder


### cppcheck
```
cppcheck --enable=all --std=c++17 ./src/*.cpp --suppress=missingIncludeSystem --suppress=unmatchedSuppression --suppress=unusedFunction --suppress=missingInclude --suppress=useInitializationList > results/cppcheck.txt
```

### cpplint
```
 cpplint --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order ./src/*.cpp ./include/*.hpp > ./results/cpplint.txt
```