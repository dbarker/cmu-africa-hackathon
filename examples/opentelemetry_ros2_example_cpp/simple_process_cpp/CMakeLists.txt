
add_executable(simple_process_cpp src/simple_process.cpp)

target_include_directories(simple_process_cpp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(simple_process_cpp PRIVATE 
  rclcpp::rclcpp
  opentelemetry-cpp::api
  otel_config
  )

install(TARGETS simple_process_cpp
  DESTINATION lib/${PROJECT_NAME})
