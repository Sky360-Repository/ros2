cmake_minimum_required(VERSION 3.5)
project(background_subtractor)

# Default to C++14
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
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sky360lib REQUIRED)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
)

# Node library
add_executable(background_subtractor src/main.cpp)
target_link_libraries(background_subtractor PUBLIC sky360lib)

ament_target_dependencies(background_subtractor PUBLIC rclcpp sensor_msgs cv_bridge OpenCV)

install(TARGETS
    background_subtractor
    DESTINATION lib/${PROJECT_NAME})

ament_package()
