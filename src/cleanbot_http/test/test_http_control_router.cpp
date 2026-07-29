#include <gtest/gtest.h>

#include "cleanbot_http/http_control_router.hpp"
#include "cleanbot_http/runtime_deadlines.hpp"

namespace {

TEST(HttpControlRouter, KeepsServiceWaitShorterThanIoDeadline) {
  EXPECT_EQ(cleanbot::http::kHttpIoOperationDeadline.count(), 3000);
  EXPECT_EQ(cleanbot::http::kRosServiceResponseDeadline.count(), 2000);
  EXPECT_LT(
      cleanbot::http::kRosServiceResponseDeadline,
      cleanbot::http::kHttpIoOperationDeadline);
}

TEST(HttpControlRouter, PreservesLegacyJoystickAndParkingRoutes) {
  cleanbot::control::JoystickParameters parameters;
  parameters.max_linear_speed = 600;
  const cleanbot::http::HttpControlRouter router(parameters);

  const auto joystick = router.route(
      "GET", "/vehicle/joystickMove/65/0/1");
  EXPECT_EQ(joystick.status_code, 200);
  EXPECT_EQ(joystick.body, "1");
  EXPECT_EQ(joystick.action, cleanbot::http::HttpControlAction::kManual);
  EXPECT_EQ(joystick.x_speed, 600);
  EXPECT_FALSE(joystick.brake);

  const auto parking = router.route("GET", "/vehicle/parking");
  EXPECT_EQ(parking.status_code, 200);
  EXPECT_EQ(
      parking.action,
      cleanbot::http::HttpControlAction::kEmergencyStop);
  EXPECT_TRUE(parking.brake);
}

TEST(HttpControlRouter, RejectsInvalidInputWithoutControlAction) {
  const cleanbot::http::HttpControlRouter router;
  const auto result = router.route(
      "GET", "/vehicle/joystickMove/nan/0/1");

  EXPECT_EQ(result.status_code, 400);
  EXPECT_EQ(result.action, cleanbot::http::HttpControlAction::kNone);
}

TEST(HttpControlRouter, RejectsStaleJoystickSequence) {
  const cleanbot::http::HttpControlRouter router;
  EXPECT_EQ(
      router.route(
          "GET",
          "/vehicle/joystickMove/50/0/1?sessionId=s1&sequence=10")
          .status_code,
      200);
  EXPECT_EQ(
      router.route(
          "GET",
          "/vehicle/joystickMove/50/0/1?sessionId=s1&sequence=12")
          .status_code,
      200);
  const auto stale = router.route(
      "GET",
      "/vehicle/joystickMove/50/0/1?sessionId=s1&sequence=11");
  EXPECT_EQ(stale.status_code, 409);
  EXPECT_EQ(stale.action, cleanbot::http::HttpControlAction::kNone);
}

TEST(HttpControlRouter, ReleasePreventsDelayedMovement) {
  const cleanbot::http::HttpControlRouter router;
  const auto released = router.route(
      "GET",
      "/vehicle/joystickMove/0/0/0?sessionId=s2&sequence=20");
  EXPECT_EQ(released.status_code, 200);
  EXPECT_TRUE(released.brake);

  const auto delayed = router.route(
      "GET",
      "/vehicle/joystickMove/80/0/1?sessionId=s2&sequence=19");
  EXPECT_EQ(delayed.status_code, 409);
  EXPECT_EQ(delayed.action, cleanbot::http::HttpControlAction::kNone);
}

TEST(HttpControlRouter, KeepsLegacyButRejectsPartialSequencePair) {
  const cleanbot::http::HttpControlRouter router;
  EXPECT_EQ(
      router.route("GET", "/vehicle/joystickMove/50/0/1").status_code,
      200);
  EXPECT_EQ(
      router.route(
          "GET",
          "/vehicle/joystickMove/50/0/1?sessionId=s3")
          .status_code,
      400);
  EXPECT_EQ(
      router.route(
          "GET",
          "/vehicle/joystickMove/50/0/1?sequence=1")
          .status_code,
      400);
}

TEST(HttpControlRouter, RoutesVehicleStateAndMissionControl) {
  const cleanbot::http::HttpControlRouter router;

  const auto state = router.route("GET", "/api/v1/vehicle/state");
  EXPECT_EQ(state.status_code, 200);
  EXPECT_EQ(
      state.action,
      cleanbot::http::HttpControlAction::kVehicleState);

  const auto pause = router.route("POST", "/api/v1/mission/pause");
  EXPECT_EQ(pause.status_code, 200);
  EXPECT_EQ(
      pause.action,
      cleanbot::http::HttpControlAction::kMissionPause);

  const auto resume = router.route("POST", "/api/v1/mission/resume");
  EXPECT_EQ(resume.status_code, 200);
  EXPECT_EQ(
      resume.action,
      cleanbot::http::HttpControlAction::kMissionResume);
}

TEST(HttpControlRouter, EnforcesMethodsForBusinessRoutes) {
  const cleanbot::http::HttpControlRouter router;

  const auto state_error =
      router.route("POST", "/api/v1/vehicle/state");
  EXPECT_EQ(state_error.status_code, 405);
  EXPECT_NE(state_error.body.find("\"data\":{}"), std::string::npos);
  EXPECT_EQ(
      router.route("GET", "/api/v1/mission/pause").status_code,
      405);
}

TEST(HttpControlRouter, ValidatesSavedPlanExecutionQuery) {
  const cleanbot::http::HttpControlRouter router;

  const auto result = router.route(
      "POST",
      "/api/v1/modeling/execute-plan?planId=plan_1&brushSpeed=900");

  EXPECT_EQ(result.status_code, 200);
  EXPECT_EQ(
      result.action,
      cleanbot::http::HttpControlAction::kExecuteModelPlan);
  EXPECT_EQ(result.plan_id, "plan_1");
  EXPECT_EQ(result.brush_speed, 900);
}

TEST(HttpControlRouter, AcceptsPlanExecutionBoundaryValues) {
  const cleanbot::http::HttpControlRouter router;
  const std::string plan_id(128u, 'a');

  const auto result = router.route(
      "POST",
      "/api/v1/modeling/execute-plan?planId=" + plan_id +
          "&brushSpeed=2147483647");

  EXPECT_EQ(result.status_code, 200);
  EXPECT_EQ(result.plan_id, plan_id);
  EXPECT_EQ(result.brush_speed, 2147483647);
}

TEST(HttpControlRouter, RejectsOversizedAndNonAsciiPlanIds) {
  const cleanbot::http::HttpControlRouter router;
  const std::string oversized_id(129u, 'a');
  const std::string non_ascii_id("\xE8\xAE\xA1\xE5\x88\x92");

  EXPECT_EQ(
      router.route(
          "POST",
          "/api/v1/modeling/execute-plan?planId=" + oversized_id +
              "&brushSpeed=900").status_code,
      400);
  EXPECT_EQ(
      router.route(
          "POST",
          "/api/v1/modeling/execute-plan?planId=" + non_ascii_id +
              "&brushSpeed=900").status_code,
      400);
}

TEST(HttpControlRouter, RejectsInvalidPlanExecutionQuery) {
  const cleanbot::http::HttpControlRouter router;

  EXPECT_EQ(
      router.route(
          "POST",
          "/api/v1/modeling/execute-plan?"
          "planId=a&planId=b&brushSpeed=900").status_code,
      400);
  EXPECT_EQ(
      router.route(
          "POST",
          "/api/v1/modeling/execute-plan?"
          "planId=a&brushSpeed=900&extra=1").status_code,
      400);
  EXPECT_EQ(
      router.route(
          "POST",
          "/api/v1/modeling/execute-plan?"
          "planId=bad%20id&brushSpeed=900").status_code,
      400);
  EXPECT_EQ(
      router.route(
          "POST",
          "/api/v1/modeling/execute-plan?"
          "planId=a&brushSpeed=0").status_code,
      400);
  EXPECT_EQ(
      router.route(
          "POST",
          "/api/v1/modeling/execute-plan?"
          "planId=a&brushSpeed=2147483648").status_code,
      400);
}

}  // namespace
