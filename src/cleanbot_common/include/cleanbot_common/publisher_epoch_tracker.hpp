// 文件作用：跟踪ROS2发布者身份和会话代际，防止旧发布者数据污染当前状态。
// 作用：识别首次发布、发布者重启、历史发布者回归和代际耗尽等情况。
#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace cleanbot {
namespace common {

struct PublisherIdentity
{
  // 发布者实现标识和底层GID共同构成一次发布会话的身份。
  std::string implementation_identifier;
  std::vector<std::uint8_t> gid;
};

// 比较中间件实现标识和发布者 GID，判断两个身份是否属于同一发布者会话。
inline bool operator==(
  const PublisherIdentity & lhs,
  const PublisherIdentity & rhs)
{
  return
    lhs.implementation_identifier == rhs.implementation_identifier &&
    lhs.gid == rhs.gid;
}

enum class PublisherEpochStatus
{
  // 发布者状态：接受、历史身份、身份非法或代际空间耗尽。
  kAccepted,
  kRetired,
  kInvalid,
  kExhausted,
};

struct PublisherEpochResult
{
  // observe方法返回的状态、当前代际以及是否切换了发布会话。
  PublisherEpochStatus status{PublisherEpochStatus::kInvalid};
  std::uint64_t epoch{0U};
  bool session_changed{false};
};

class PublisherEpochTracker
{
public:
  static constexpr std::size_t
    kMaximumImplementationIdentifierBytes = 256U;
  static constexpr std::size_t kMaximumGidBytes = 256U;

  static constexpr std::uint64_t default_maximum_epoch() noexcept
  {
    // 方法作用：返回允许使用的最大代际值，默认覆盖完整uint64范围。
    return std::numeric_limits<std::uint64_t>::max();
  }

  explicit PublisherEpochTracker(
    std::size_t maximum_retired_identities = 1024U,
    std::uint64_t maximum_epoch = default_maximum_epoch()) noexcept
  : maximum_retired_identities_(maximum_retired_identities),
    maximum_epoch_(maximum_epoch)
  {
    // 构造方法作用：设置历史身份容量和代际上限。
  }

  PublisherEpochResult observe(const PublisherIdentity & identity)
  {
    // 方法作用：观察发布者身份并决定是否接受、切换或拒绝当前会话。
    if (!is_valid(identity)) {
      return result(PublisherEpochStatus::kInvalid);
    }

    if (has_current_identity_ && identity == current_identity_) {
      return result(PublisherEpochStatus::kAccepted);
    }

    if (is_retired(identity)) {
      return result(PublisherEpochStatus::kRetired);
    }

    if (current_epoch_ >= maximum_epoch_) {
      return result(PublisherEpochStatus::kExhausted);
    }

    if (!has_current_identity_) {
      current_identity_ = identity;
      has_current_identity_ = true;
      ++current_epoch_;
      return result(PublisherEpochStatus::kAccepted, true);
    }

    if (retired_identities_.size() >= maximum_retired_identities_) {
      return result(PublisherEpochStatus::kExhausted);
    }

    PublisherIdentity next_identity = identity;
    retired_identities_.push_back(current_identity_);
    current_identity_ = std::move(next_identity);
    ++current_epoch_;
    return result(PublisherEpochStatus::kAccepted, true);
  }

private:
  static bool is_valid(const PublisherIdentity & identity)
  {
    // 方法作用：校验实现标识和GID是否满足长度及非零约束。
    if (
      identity.implementation_identifier.empty() ||
      identity.implementation_identifier.size() >
      kMaximumImplementationIdentifierBytes ||
      identity.gid.empty() ||
      identity.gid.size() > kMaximumGidBytes)
    {
      return false;
    }

    return std::any_of(
      identity.gid.cbegin(),
      identity.gid.cend(),
      // 筛选谓词作用：检查当前元素是否符合查找、确认或删除条件。
      [](std::uint8_t value) {return value != 0U;});
  }

  bool is_retired(const PublisherIdentity & identity) const
  {
    // 方法作用：判断身份是否属于已经退休的旧发布者会话。
    return std::find(
      retired_identities_.cbegin(),
      retired_identities_.cend(),
      identity) != retired_identities_.cend();
  }

  PublisherEpochResult result(
    PublisherEpochStatus status,
    bool session_changed = false) const noexcept
  {
    // 方法作用：用当前代际组装统一的观察结果对象。
    return PublisherEpochResult{status, current_epoch_, session_changed};
  }

  std::size_t maximum_retired_identities_;
  std::uint64_t maximum_epoch_;
  std::vector<PublisherIdentity> retired_identities_;
  PublisherIdentity current_identity_;
  std::uint64_t current_epoch_{0U};
  bool has_current_identity_{false};
};

}  // namespace common
}  // namespace cleanbot
