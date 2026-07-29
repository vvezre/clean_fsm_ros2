#include "cleanbot_http/vehicle_state_json.hpp"

#include <cmath>
#include <iomanip>
#include <locale>
#include <sstream>

namespace cleanbot {
namespace http {
namespace {

std::string escapeJson(const std::string& value) {
  std::ostringstream output;
  output.imbue(std::locale::classic());
  output << std::hex << std::setfill('0');
  for (const unsigned char character : value) {
    switch (character) {
      case '"':
        output << "\\\"";
        break;
      case '\\':
        output << "\\\\";
        break;
      case '\b':
        output << "\\b";
        break;
      case '\f':
        output << "\\f";
        break;
      case '\n':
        output << "\\n";
        break;
      case '\r':
        output << "\\r";
        break;
      case '\t':
        output << "\\t";
        break;
      default:
        if (character < 0x20u) {
          output << "\\u00" << std::setw(2)
                 << static_cast<unsigned int>(character);
        } else {
          output << static_cast<char>(character);
        }
        break;
    }
  }
  return output.str();
}

const char* jsonBoolean(const bool value) {
  return value ? "true" : "false";
}

void appendString(
    std::ostringstream& output,
    const char* name,
    const std::string& value) {
  output << '"' << name << "\":\"" << escapeJson(value) << '"';
}

}  // namespace

std::string business_response_json(
    const bool success,
    const std::string& code,
    const std::string& message,
    const std::string& data_json) {
  std::ostringstream output;
  output.imbue(std::locale::classic());
  output << "{\"success\":" << jsonBoolean(success) << ',';
  appendString(output, "code", code);
  output << ',';
  appendString(output, "message", message);
  output << ",\"data\":" << (data_json.empty() ? "{}" : data_json) << '}';
  return output.str();
}

std::string vehicle_state_json(const VehicleStateJsonData& state) {
  std::ostringstream data;
  data.imbue(std::locale::classic());
  data << "{\"stamp\":{\"sec\":" << state.stamp_sec
       << ",\"nanosec\":" << state.stamp_nanosec << "},";
  appendString(data, "controlState", state.control_state);
  data << ',';
  appendString(data, "healthState", state.health_state);
  data << ',';
  appendString(data, "faultState", state.fault_state);
  data << ',';
  appendString(data, "currentAction", state.current_action);
  data << ',';
  appendString(data, "message", state.message);
  data << ",\"startReady\":" << jsonBoolean(state.start_ready)
       << ",\"parking\":" << jsonBoolean(state.parking)
       << ",\"cleaning\":" << jsonBoolean(state.cleaning)
       << ",\"rtkFixed\":" << jsonBoolean(state.rtk_fixed)
       << ",\"inGarage\":" << jsonBoolean(state.in_garage)
       << ",\"batteryPercent\":";
  if (std::isfinite(state.battery_percent)) {
    data << std::setprecision(15) << state.battery_percent;
  } else {
    data << "-1.0";
  }
  data << ",\"currentSegment\":" << state.current_segment
       << ",\"totalSegments\":" << state.total_segments << '}';
  return business_response_json(
      true, "OK", "vehicle state available", data.str());
}

}  // namespace http
}  // namespace cleanbot
