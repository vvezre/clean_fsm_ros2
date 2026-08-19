// 文件作用：为对应模块的核心算法、协议处理和边界条件提供单元测试。
#include <gtest/gtest.h>

#include <cstdint>
#include <initializer_list>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "cleanbot_common/publisher_epoch_tracker.hpp"

namespace {

using cleanbot::common::PublisherEpochStatus;
using cleanbot::common::PublisherEpochTracker;
using cleanbot::common::PublisherIdentity;

// 辅助函数作用：通过 identity 构造当前测试所需的输入数据。
PublisherIdentity identity(
  std::string implementation_identifier,
  std::initializer_list<std::uint8_t> gid)
{
  return PublisherIdentity{
    std::move(implementation_identifier),
    std::vector<std::uint8_t>(gid)};
}

// 测试目的：验证 PublisherEpochTracker.RejectsInvalidIdentitiesWithoutChangingSession 场景的行为、状态变化和边界条件。
TEST(PublisherEpochTracker, RejectsInvalidIdentitiesWithoutChangingSession)
{
  PublisherEpochTracker tracker;

  const auto empty_identifier = tracker.observe(identity("", {1U}));
  const auto empty_gid = tracker.observe(identity("rmw-a", {}));
  const auto all_zero_gid = tracker.observe(identity("rmw-a", {0U, 0U, 0U}));

  for (const auto & result : {empty_identifier, empty_gid, all_zero_gid}) {
    EXPECT_EQ(result.status, PublisherEpochStatus::kInvalid);
    EXPECT_EQ(result.epoch, 0U);
    EXPECT_FALSE(result.session_changed);
  }

  const auto first_valid = tracker.observe(identity("rmw-a", {0U, 1U, 0U}));
  EXPECT_EQ(first_valid.status, PublisherEpochStatus::kAccepted);
  EXPECT_EQ(first_valid.epoch, 1U);
  EXPECT_TRUE(first_valid.session_changed);
}

// 测试目的：验证 PublisherEpochTracker.BoundsOwnedIdentityFields 场景的行为、状态变化和边界条件。
TEST(PublisherEpochTracker, BoundsOwnedIdentityFields)
{
  PublisherEpochTracker tracker;
  const std::string maximum_identifier(
    PublisherEpochTracker::kMaximumImplementationIdentifierBytes,
    'r');
  std::vector<std::uint8_t> maximum_gid(
    PublisherEpochTracker::kMaximumGidBytes,
    0U);
  maximum_gid.back() = 1U;

  PublisherIdentity maximum_identity{
    maximum_identifier,
    maximum_gid};
  const auto maximum = tracker.observe(maximum_identity);
  EXPECT_EQ(maximum.status, PublisherEpochStatus::kAccepted);
  EXPECT_EQ(maximum.epoch, 1U);
  EXPECT_TRUE(maximum.session_changed);

  PublisherIdentity overlong_identifier{
    maximum_identifier + "x",
    {1U}};
  const auto invalid_identifier =
    tracker.observe(overlong_identifier);
  EXPECT_EQ(invalid_identifier.status, PublisherEpochStatus::kInvalid);
  EXPECT_EQ(invalid_identifier.epoch, 1U);
  EXPECT_FALSE(invalid_identifier.session_changed);

  auto overlong_gid = maximum_gid;
  overlong_gid.push_back(1U);
  PublisherIdentity overlong_gid_identity{
    "rmw-a",
    std::move(overlong_gid)};
  const auto invalid_gid = tracker.observe(overlong_gid_identity);
  EXPECT_EQ(invalid_gid.status, PublisherEpochStatus::kInvalid);
  EXPECT_EQ(invalid_gid.epoch, 1U);
  EXPECT_FALSE(invalid_gid.session_changed);
}

// 测试目的：验证 PublisherEpochTracker.KeepsTheCurrentIdentityInTheSameEpoch 场景的行为、状态变化和边界条件。
TEST(PublisherEpochTracker, KeepsTheCurrentIdentityInTheSameEpoch)
{
  PublisherEpochTracker tracker;
  const auto publisher = identity("rmw-a", {1U, 2U, 3U});

  const auto first = tracker.observe(publisher);
  const auto repeated = tracker.observe(publisher);

  EXPECT_EQ(first.status, PublisherEpochStatus::kAccepted);
  EXPECT_EQ(first.epoch, 1U);
  EXPECT_TRUE(first.session_changed);
  EXPECT_EQ(repeated.status, PublisherEpochStatus::kAccepted);
  EXPECT_EQ(repeated.epoch, 1U);
  EXPECT_FALSE(repeated.session_changed);
}

// 测试目的：验证 PublisherEpochTracker.OwnsTheAcceptedIdentity 场景的行为、状态变化和边界条件。
TEST(PublisherEpochTracker, OwnsTheAcceptedIdentity)
{
  PublisherEpochTracker tracker;
  auto publisher = identity("rmw-a", {1U, 2U, 3U});

  ASSERT_EQ(
    tracker.observe(publisher).status,
    PublisherEpochStatus::kAccepted);
  publisher.implementation_identifier = "mutated";
  publisher.gid.assign({9U});

  const auto original = tracker.observe(identity("rmw-a", {1U, 2U, 3U}));
  EXPECT_EQ(original.status, PublisherEpochStatus::kAccepted);
  EXPECT_EQ(original.epoch, 1U);
  EXPECT_FALSE(original.session_changed);
}

// 测试目的：验证 PublisherEpochTracker.NewIdentityAdvancesEpochAndRetiresOldIdentity 场景的行为、状态变化和边界条件。
TEST(PublisherEpochTracker, NewIdentityAdvancesEpochAndRetiresOldIdentity)
{
  PublisherEpochTracker tracker;
  const auto first_publisher = identity("rmw-a", {1U});
  const auto second_publisher = identity("rmw-a", {2U});

  ASSERT_EQ(
    tracker.observe(first_publisher).status,
    PublisherEpochStatus::kAccepted);
  const auto switched = tracker.observe(second_publisher);
  const auto replayed_old = tracker.observe(first_publisher);
  const auto repeated_current = tracker.observe(second_publisher);

  EXPECT_EQ(switched.status, PublisherEpochStatus::kAccepted);
  EXPECT_EQ(switched.epoch, 2U);
  EXPECT_TRUE(switched.session_changed);
  EXPECT_EQ(replayed_old.status, PublisherEpochStatus::kRetired);
  EXPECT_EQ(replayed_old.epoch, 2U);
  EXPECT_FALSE(replayed_old.session_changed);
  EXPECT_EQ(repeated_current.status, PublisherEpochStatus::kAccepted);
  EXPECT_EQ(repeated_current.epoch, 2U);
  EXPECT_FALSE(repeated_current.session_changed);
}

// 测试目的：验证 PublisherEpochTracker.IdentityIncludesImplementationIdentifierAndEveryGidByte 场景的行为、状态变化和边界条件。
TEST(PublisherEpochTracker, IdentityIncludesImplementationIdentifierAndEveryGidByte)
{
  PublisherEpochTracker tracker;

  ASSERT_EQ(
    tracker.observe(identity("rmw-a", {1U, 2U, 3U})).epoch,
    1U);
  const auto different_implementation =
    tracker.observe(identity("rmw-b", {1U, 2U, 3U}));
  const auto different_final_byte =
    tracker.observe(identity("rmw-b", {1U, 2U, 4U}));

  EXPECT_EQ(
    different_implementation.status,
    PublisherEpochStatus::kAccepted);
  EXPECT_EQ(different_implementation.epoch, 2U);
  EXPECT_TRUE(different_implementation.session_changed);
  EXPECT_EQ(different_final_byte.status, PublisherEpochStatus::kAccepted);
  EXPECT_EQ(different_final_byte.epoch, 3U);
  EXPECT_TRUE(different_final_byte.session_changed);
}

// 测试目的：验证 PublisherEpochTracker.RetiredStatusTakesPrecedenceOverExhaustion 场景的行为、状态变化和边界条件。
TEST(PublisherEpochTracker, RetiredStatusTakesPrecedenceOverExhaustion)
{
  PublisherEpochTracker tracker(1U, 2U);
  const auto first_publisher = identity("rmw-a", {1U});
  const auto second_publisher = identity("rmw-a", {2U});

  ASSERT_EQ(tracker.observe(first_publisher).epoch, 1U);
  ASSERT_EQ(tracker.observe(second_publisher).epoch, 2U);
  const auto retired = tracker.observe(first_publisher);

  EXPECT_EQ(retired.status, PublisherEpochStatus::kRetired);
  EXPECT_EQ(retired.epoch, 2U);
  EXPECT_FALSE(retired.session_changed);
}

// 测试目的：验证 PublisherEpochTracker.FailsClosedBeforeRetiredIdentityCapacityIsExceeded 场景的行为、状态变化和边界条件。
TEST(PublisherEpochTracker, FailsClosedBeforeRetiredIdentityCapacityIsExceeded)
{
  PublisherEpochTracker tracker(1U);
  const auto first_publisher = identity("rmw-a", {1U});
  const auto second_publisher = identity("rmw-a", {2U});
  const auto third_publisher = identity("rmw-a", {3U});

  ASSERT_EQ(tracker.observe(first_publisher).epoch, 1U);
  ASSERT_EQ(tracker.observe(second_publisher).epoch, 2U);
  const auto exhausted = tracker.observe(third_publisher);
  const auto still_current = tracker.observe(second_publisher);

  EXPECT_EQ(exhausted.status, PublisherEpochStatus::kExhausted);
  EXPECT_EQ(exhausted.epoch, 2U);
  EXPECT_FALSE(exhausted.session_changed);
  EXPECT_EQ(still_current.status, PublisherEpochStatus::kAccepted);
  EXPECT_EQ(still_current.epoch, 2U);
  EXPECT_FALSE(still_current.session_changed);
}

// 测试目的：验证 PublisherEpochTracker.ZeroRetiredCapacityAllowsOnlyTheFirstIdentity 场景的行为、状态变化和边界条件。
TEST(PublisherEpochTracker, ZeroRetiredCapacityAllowsOnlyTheFirstIdentity)
{
  PublisherEpochTracker tracker(0U);

  const auto first = tracker.observe(identity("rmw-a", {1U}));
  const auto switch_attempt = tracker.observe(identity("rmw-a", {2U}));

  EXPECT_EQ(first.status, PublisherEpochStatus::kAccepted);
  EXPECT_EQ(first.epoch, 1U);
  EXPECT_TRUE(first.session_changed);
  EXPECT_EQ(switch_attempt.status, PublisherEpochStatus::kExhausted);
  EXPECT_EQ(switch_attempt.epoch, 1U);
  EXPECT_FALSE(switch_attempt.session_changed);
}

// 测试目的：验证 PublisherEpochTracker.FailsClosedAtConfiguredEpochCeilingWithoutWrapping 场景的行为、状态变化和边界条件。
TEST(PublisherEpochTracker, FailsClosedAtConfiguredEpochCeilingWithoutWrapping)
{
  PublisherEpochTracker tracker(
    8U,
    2U);
  const auto first_publisher = identity("rmw-a", {1U});
  const auto second_publisher = identity("rmw-a", {2U});
  const auto third_publisher = identity("rmw-a", {3U});

  ASSERT_EQ(tracker.observe(first_publisher).epoch, 1U);
  ASSERT_EQ(tracker.observe(second_publisher).epoch, 2U);
  const auto exhausted = tracker.observe(third_publisher);

  EXPECT_EQ(exhausted.status, PublisherEpochStatus::kExhausted);
  EXPECT_EQ(exhausted.epoch, 2U);
  EXPECT_FALSE(exhausted.session_changed);
}

// 测试目的：验证 PublisherEpochTracker.ZeroEpochCeilingRejectsEvenTheFirstValidIdentity 场景的行为、状态变化和边界条件。
TEST(PublisherEpochTracker, ZeroEpochCeilingRejectsEvenTheFirstValidIdentity)
{
  PublisherEpochTracker tracker(8U, 0U);

  const auto exhausted = tracker.observe(identity("rmw-a", {1U}));

  EXPECT_EQ(exhausted.status, PublisherEpochStatus::kExhausted);
  EXPECT_EQ(exhausted.epoch, 0U);
  EXPECT_FALSE(exhausted.session_changed);
}

// 测试目的：验证 PublisherEpochTracker.DefaultEpochCeilingIsUint64Max 场景的行为、状态变化和边界条件。
TEST(PublisherEpochTracker, DefaultEpochCeilingIsUint64Max)
{
  EXPECT_EQ(
    PublisherEpochTracker::default_maximum_epoch(),
    std::numeric_limits<std::uint64_t>::max());
}

}  // namespace
// 文件作用：验证发布者身份、会话代际切换和历史身份拒绝规则。
