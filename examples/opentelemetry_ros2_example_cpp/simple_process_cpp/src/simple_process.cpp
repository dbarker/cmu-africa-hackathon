#include <opentelemetry/logs/log_record.h>
#include <opentelemetry/logs/logger_provider.h>
#include <opentelemetry/logs/provider.h>
#include <opentelemetry/logs/severity.h>
#include <opentelemetry/metrics/meter.h>
#include <opentelemetry/metrics/provider.h>
#include <opentelemetry/metrics/sync_instruments.h>
#include <opentelemetry/trace/provider.h>
#include <opentelemetry/trace/scope.h>
#include <opentelemetry/trace/span.h>
#include <opentelemetry/trace/tracer.h>

#include <iostream>
#include <rclcpp/rclcpp.hpp>

#include "otel_config/otel_provider_context.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  otel_example::OtelProviderContext otel_context("simple_process", "0.1.0", "otel-collector:4317");

  RCLCPP_INFO(rclcpp::get_logger("simple_process"), "Starting simple_process...");

  auto meter = opentelemetry::metrics::Provider::GetMeterProvider()->GetMeter("simple_process");
  auto tracer = opentelemetry::trace::Provider::GetTracerProvider()->GetTracer("simple_process");
  auto logger = opentelemetry::logs::Provider::GetLoggerProvider()->GetLogger("simple_process");

  auto counter = meter->CreateUInt64Counter("simple_process_counter", "a counter of loops", "num");

  std::uint64_t count = 0;

  while (rclcpp::ok())
  {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("simple_process"), "Loop count=" << count);

    auto span = tracer->StartSpan("loop_span");
    opentelemetry::trace::Scope scope{ span };
    counter->Add(1);

    span->AddEvent("count", { { "value", count } });

    const auto msg = std::string("Loop complete count=") + std::to_string(count);
    logger->Info(msg);

    count++;

    std::this_thread::sleep_for(std::chrono::seconds(2));
  }

  rclcpp::shutdown();
  return 0;
}
