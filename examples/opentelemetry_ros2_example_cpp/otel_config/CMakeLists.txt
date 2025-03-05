
add_library(otel_config src/otel_provider_context.cpp)

target_include_directories(otel_config PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(otel_config PRIVATE 
  opentelemetry-cpp::api
  opentelemetry-cpp::sdk
  opentelemetry-cpp::otlp_grpc_exporter
  opentelemetry-cpp::otlp_grpc_log_record_exporter
  opentelemetry-cpp::otlp_grpc_metrics_exporter
  )

install(TARGETS otel_config
  DESTINATION lib/${PROJECT_NAME})
