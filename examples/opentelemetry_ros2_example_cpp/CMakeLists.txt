cmake_minimum_required(VERSION 3.8)
project(opentelemetry_ros2_example_cpp)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(opentelemetry-cpp CONFIG REQUIRED)

add_subdirectory(otel_config)
add_subdirectory(simple_process_cpp)

ament_package()
