cmake_minimum_required(VERSION 3.8)
project(my_robotic_arm)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(controller_manager REQUIRED)
find_package(std_msgs REQUIRED)

install(DIRECTORY launch urdf config rviz worlds
        DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/control_arm.py
                   scripts/instant_keyboard_control.py
                   scripts/keyboard_control.py
        DESTINATION lib/${PROJECT_NAME})

ament_package()
