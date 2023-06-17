cmake_minimum_required(VERSION 3.5)
project(sky360_tracking)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if (CMAKE_COMPILER_IS_GNUCXX)
    find_package(TBB REQUIRED)  # for <execution> stdlib
endif ()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sky360lib REQUIRED)
find_package(sky360_interfaces REQUIRED)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
)

# Frame Provider Node
add_executable(track_provider_node src/track_provider.cpp)
target_link_libraries(track_provider_node PUBLIC sky360lib)
ament_target_dependencies(track_provider_node PUBLIC rclcpp sensor_msgs vision_msgs cv_bridge message_filters OpenCV sky360_interfaces)
install(TARGETS track_provider_node DESTINATION lib/${PROJECT_NAME})

ament_package()
