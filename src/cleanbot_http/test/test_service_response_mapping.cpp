#include "cleanbot_http/service_response_mapping.hpp"

#include <gtest/gtest.h>

using cleanbot::http::DownstreamResponse;
using cleanbot::http::DownstreamResponseState;
using cleanbot::http::map_execute_plan_response;
using cleanbot::http::map_mission_pause_response;

TEST(ServiceResponseMapping, MapsMissionUnavailableAndTimeout) {
  const auto unavailable = map_mission_pause_response(
      DownstreamResponseState::kUnavailable, DownstreamResponse());
  EXPECT_EQ(unavailable.status_code, 503);
  EXPECT_FALSE(unavailable.success);
  EXPECT_EQ(unavailable.code, "MISSION_SERVICE_UNAVAILABLE");

  const auto timeout = map_mission_pause_response(
      DownstreamResponseState::kTimeout, DownstreamResponse());
  EXPECT_EQ(timeout.status_code, 504);
  EXPECT_FALSE(timeout.success);
  EXPECT_EQ(timeout.code, "MISSION_SERVICE_TIMEOUT");
}

TEST(ServiceResponseMapping, PreservesMissionServiceOutcome) {
  DownstreamResponse response;
  response.accepted = true;
  response.code = "MISSION_PAUSE_REQUESTED";
  response.message = "mission is pausing";

  const auto accepted = map_mission_pause_response(
      DownstreamResponseState::kResponded, response);
  EXPECT_EQ(accepted.status_code, 200);
  EXPECT_TRUE(accepted.success);
  EXPECT_EQ(accepted.code, response.code);
  EXPECT_EQ(accepted.message, response.message);

  response.accepted = false;
  response.code = "NO_ACTIVE_MISSION";
  const auto rejected = map_mission_pause_response(
      DownstreamResponseState::kResponded, response);
  EXPECT_EQ(rejected.status_code, 409);
  EXPECT_FALSE(rejected.success);
  EXPECT_EQ(rejected.code, "NO_ACTIVE_MISSION");
}

TEST(ServiceResponseMapping, MapsModelingUnavailableAndTimeout) {
  const auto unavailable = map_execute_plan_response(
      DownstreamResponseState::kUnavailable, DownstreamResponse());
  EXPECT_EQ(unavailable.status_code, 503);
  EXPECT_FALSE(unavailable.success);
  EXPECT_EQ(unavailable.code, "MODELING_SERVICE_UNAVAILABLE");

  const auto timeout = map_execute_plan_response(
      DownstreamResponseState::kTimeout, DownstreamResponse());
  EXPECT_EQ(timeout.status_code, 504);
  EXPECT_FALSE(timeout.success);
  EXPECT_EQ(timeout.code, "MODELING_SERVICE_TIMEOUT");
}

TEST(ServiceResponseMapping, PreservesModelingServiceOutcome) {
  DownstreamResponse response;
  response.accepted = true;
  response.code = "MISSION_GOAL_ACCEPTED";
  response.message = "cleaning mission goal accepted";

  const auto accepted = map_execute_plan_response(
      DownstreamResponseState::kResponded, response);
  EXPECT_EQ(accepted.status_code, 202);
  EXPECT_TRUE(accepted.success);
  EXPECT_EQ(accepted.code, response.code);
  EXPECT_EQ(accepted.message, response.message);

  response.accepted = false;
  response.code = "MISSION_GOAL_REJECTED";
  const auto rejected = map_execute_plan_response(
      DownstreamResponseState::kResponded, response);
  EXPECT_EQ(rejected.status_code, 409);
  EXPECT_FALSE(rejected.success);
  EXPECT_EQ(rejected.code, "MISSION_GOAL_REJECTED");
}
