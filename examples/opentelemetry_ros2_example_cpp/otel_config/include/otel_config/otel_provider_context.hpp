#pragma once

#include <string>

namespace otel_example
{
class OtelProviderContext
{
public:
  OtelProviderContext(const std::string& service_name, const std::string& service_version, const std::string& endpoint);
  ~OtelProviderContext();
};

}  // namespace otel_example