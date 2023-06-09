cmake_minimum_required(VERSION 3.5)
project(all_sky_image_publisher)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rcl_interfaces REQUIRED)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
)

# Node library
add_executable(all_sky_image_publisher src/main.cpp)
ament_target_dependencies(all_sky_image_publisher rclcpp sensor_msgs cv_bridge OpenCV)

install(TARGETS
    all_sky_image_publisher
    DESTINATION lib/${PROJECT_NAME})

ament_package()
