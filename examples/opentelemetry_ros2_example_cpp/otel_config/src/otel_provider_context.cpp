#include "otel_config/otel_provider_context.hpp"

#include <opentelemetry/exporters/otlp/otlp_grpc_client_factory.h>
#include <opentelemetry/exporters/otlp/otlp_grpc_exporter_factory.h>
#include <opentelemetry/exporters/otlp/otlp_grpc_log_record_exporter_factory.h>
#include <opentelemetry/exporters/otlp/otlp_grpc_metric_exporter_factory.h>
#include <opentelemetry/logs/provider.h>
#include <opentelemetry/metrics/provider.h>
#include <opentelemetry/sdk/logs/batch_log_record_processor_factory.h>
#include <opentelemetry/sdk/logs/logger_provider_factory.h>
#include <opentelemetry/sdk/metrics/export/periodic_exporting_metric_reader_factory.h>
#include <opentelemetry/sdk/metrics/export/periodic_exporting_metric_reader_options.h>
#include <opentelemetry/sdk/metrics/meter_provider_factory.h>
#include <opentelemetry/sdk/resource/resource.h>
#include <opentelemetry/sdk/trace/batch_span_processor_factory.h>
#include <opentelemetry/sdk/trace/tracer_provider_factory.h>
#include <opentelemetry/trace/provider.h>

namespace otel_example
{

OtelProviderContext::OtelProviderContext(
    const std::string& service_name, const std::string& service_version, const std::string& endpoint)
{
  // The resource sets the process level attributes on telemetry signal data
  auto resource = opentelemetry::sdk::resource::Resource::Create(
      { { "service.name", "simple_process" }, { "service.version", "0.1.0" } });

  auto grpc_options = opentelemetry::exporter::otlp::OtlpGrpcExporterOptions();
  grpc_options.endpoint = "otel-collector:4317";

  auto grpc_client = opentelemetry::exporter::otlp::OtlpGrpcClientFactory::Create(grpc_options);

  {
    auto exporter = opentelemetry::exporter::otlp::OtlpGrpcExporterFactory::Create(grpc_options, grpc_client);
    auto options = opentelemetry::sdk::trace::BatchSpanProcessorOptions();
    auto processor = opentelemetry::sdk::trace::BatchSpanProcessorFactory::Create(std::move(exporter), options);
    auto tracer_provider = opentelemetry::sdk::trace::TracerProviderFactory::Create(std::move(processor), resource);
    opentelemetry::trace::Provider::SetTracerProvider(
        opentelemetry::nostd::shared_ptr<opentelemetry::trace::TracerProvider>(tracer_provider.release()));
  }

  {
    auto logs_grpc_options = opentelemetry::exporter::otlp::OtlpGrpcLogRecordExporterOptions();
    logs_grpc_options.endpoint = grpc_options.endpoint;
    auto exporter =
        opentelemetry::exporter::otlp::OtlpGrpcLogRecordExporterFactory::Create(logs_grpc_options, grpc_client);
    auto options = opentelemetry::sdk::logs::BatchLogRecordProcessorOptions();
    auto processor = opentelemetry::sdk::logs::BatchLogRecordProcessorFactory::Create(std::move(exporter), options);
    auto logs_provider = opentelemetry::sdk::logs::LoggerProviderFactory::Create(std::move(processor), resource);
    opentelemetry::logs::Provider::SetLoggerProvider(
        opentelemetry::nostd::shared_ptr<opentelemetry::logs::LoggerProvider>(logs_provider.release()));
  }

  {
    auto metrics_grpc_options = opentelemetry::exporter::otlp::OtlpGrpcMetricExporterOptions();
    metrics_grpc_options.endpoint = grpc_options.endpoint;
    auto exporter =
        opentelemetry::exporter::otlp::OtlpGrpcMetricExporterFactory::Create(metrics_grpc_options, grpc_client);
    auto options = opentelemetry::sdk::metrics::PeriodicExportingMetricReaderOptions();
    options.export_interval_millis = std::chrono::milliseconds(30000);
    auto reader =
        opentelemetry::sdk::metrics::PeriodicExportingMetricReaderFactory::Create(std::move(exporter), options);
    auto meter_provider = opentelemetry::sdk::metrics::MeterProviderFactory::Create(
        std::make_unique<opentelemetry::sdk::metrics::ViewRegistry>(), resource);
    meter_provider->AddMetricReader(std::move(reader));

    opentelemetry::metrics::Provider::SetMeterProvider(
        opentelemetry::nostd::shared_ptr<opentelemetry::metrics::MeterProvider>(meter_provider.release()));
  }
}

OtelProviderContext::~OtelProviderContext()
{
  // flush the exporters
  auto sdk_tracer_provider = static_cast<opentelemetry::sdk::trace::TracerProvider*>(
      opentelemetry::trace::Provider::GetTracerProvider().get());
  sdk_tracer_provider->ForceFlush();

  auto sdk_logs_provider =
      static_cast<opentelemetry::sdk::logs::LoggerProvider*>(opentelemetry::logs::Provider::GetLoggerProvider().get());
  sdk_logs_provider->ForceFlush();

  auto sdk_meter_provider = static_cast<opentelemetry::sdk::metrics::MeterProvider*>(
      opentelemetry::metrics::Provider::GetMeterProvider().get());
  sdk_meter_provider->ForceFlush();

  opentelemetry::trace::Provider::SetTracerProvider(
      opentelemetry::nostd::shared_ptr<opentelemetry::trace::TracerProvider>(nullptr));
  opentelemetry::logs::Provider::SetLoggerProvider(
      opentelemetry::nostd::shared_ptr<opentelemetry::logs::LoggerProvider>(nullptr));
  opentelemetry::metrics::Provider::SetMeterProvider(
      opentelemetry::nostd::shared_ptr<opentelemetry::metrics::MeterProvider>(nullptr));
}

}  // namespace otel_example
