cmake_minimum_required(VERSION 3.5)
project(all_sky_image_publisher)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
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
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sky360lib REQUIRED)
find_package(rosidl_default_generators REQUIRED)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
)

set(msg_files
  "msg/AreaLimits.msg"
  "msg/BayerFormat.msg"
  "msg/CameraInfo.msg"
  "msg/ChipInfo.msg"
  "msg/ImageInfo.msg"
  "msg/ParamLimits.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME} ${msg_files}
    DEPENDENCIES builtin_interfaces std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

add_executable(all_sky_image_publisher_node src/main.cpp)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(all_sky_image_publisher_node PUBLIC sky360lib "${cpp_typesupport_target}")
ament_target_dependencies(all_sky_image_publisher_node PUBLIC
    rclcpp 
    sensor_msgs 
    std_msgs
    cv_bridge
    rcl_interfaces 
    OpenCV)

install(TARGETS
    all_sky_image_publisher_node
    DESTINATION lib/${PROJECT_NAME})

ament_package()
