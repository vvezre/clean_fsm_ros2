#include <cstdint>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "cleanbot_rtk/ntrip_protocol.hpp"

namespace {

using cleanbot::rtk::NtripConfig;
using cleanbot::rtk::NtripResponseParser;
using cleanbot::rtk::build_ntrip_request;

TEST(NtripProtocol, BuildsAuthenticatedRequest) {
  NtripConfig config;
  config.host = "caster.example";
  config.port = 2101;
  config.mountpoint = "RTCM33";
  config.username = "user";
  config.password = "pass";

  const auto request = build_ntrip_request(config);
  EXPECT_NE(request.find("GET /RTCM33 HTTP/1.0"), std::string::npos);
  EXPECT_NE(request.find("Authorization: Basic dXNlcjpwYXNz"), std::string::npos);
}

TEST(NtripProtocol, PreservesRtcmInResponsePacket) {
  NtripResponseParser parser;
  const std::vector<std::uint8_t> packet{
      'I', 'C', 'Y', ' ', '2', '0', '0', ' ', 'O', 'K', '\r', '\n', '\r', '\n',
      0xd3, 0x00, 0x13};
  const auto result = parser.append(packet);
  ASSERT_TRUE(result.accepted);
  EXPECT_EQ(result.rtcm_bytes, std::vector<std::uint8_t>({0xd3, 0x00, 0x13}));
}

}  // namespace
