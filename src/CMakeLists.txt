cmake_minimum_required(VERSION 3.5)
project(ds4_driver)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set the path to DynamixelSDK
set(dynamixel_sdk_DIR "/home/ubuntu/DynamixelSDK/ros/dynamixel_sdk")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ds4_driver_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(dynamixel_sdk REQUIRED PATHS ${dynamixel_sdk_DIR})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Python Package Install
ament_python_install_package(${PROJECT_NAME})

# Install configuration and launch files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY nodes
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python nodes
install(PROGRAMS
  nodes/demo.py
  nodes/ds4_driver_node.py
  nodes/ds4_twist_node.py
  nodes/ds4_test_node.py
  nodes/joint_control_node.py
  nodes/control_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()