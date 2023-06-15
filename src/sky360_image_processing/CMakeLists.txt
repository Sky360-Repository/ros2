cmake_minimum_required(VERSION 3.5)
project(sky360_image_processing)

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
find_package(sky360_camera REQUIRED)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
)

# Frame Provider Node
add_executable(frame_provider_node src/frame_provider.cpp)
target_link_libraries(frame_provider_node PUBLIC sky360lib)
ament_target_dependencies(frame_provider_node PUBLIC rclcpp sensor_msgs cv_bridge OpenCV sky360_camera)
install(TARGETS frame_provider_node DESTINATION lib/${PROJECT_NAME})

# Frame Provider Node
add_executable(background_subtractor_node src/background_subtractor.cpp)
target_link_libraries(background_subtractor_node PUBLIC sky360lib)
ament_target_dependencies(background_subtractor_node PUBLIC rclcpp sensor_msgs cv_bridge OpenCV)
install(TARGETS background_subtractor_node DESTINATION lib/${PROJECT_NAME})

ament_package()
