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
  std::string implementation_identifier;
  std::vector<std::uint8_t> gid;
};

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
  kAccepted,
  kRetired,
  kInvalid,
  kExhausted,
};

struct PublisherEpochResult
{
  PublisherEpochStatus status{PublisherEpochStatus::kInvalid};
  std::uint64_t epoch{0U};
  bool session_changed{false};
};

class PublisherEpochTracker
{
public:
  static constexpr std::uint64_t default_maximum_epoch() noexcept
  {
    return std::numeric_limits<std::uint64_t>::max();
  }

  explicit PublisherEpochTracker(
    std::size_t maximum_retired_identities = 1024U,
    std::uint64_t maximum_epoch = default_maximum_epoch()) noexcept
  : maximum_retired_identities_(maximum_retired_identities),
    maximum_epoch_(maximum_epoch)
  {
  }

  PublisherEpochResult observe(const PublisherIdentity & identity)
  {
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
    if (identity.implementation_identifier.empty() || identity.gid.empty()) {
      return false;
    }

    return std::any_of(
      identity.gid.cbegin(),
      identity.gid.cend(),
      [](std::uint8_t value) {return value != 0U;});
  }

  bool is_retired(const PublisherIdentity & identity) const
  {
    return std::find(
      retired_identities_.cbegin(),
      retired_identities_.cend(),
      identity) != retired_identities_.cend();
  }

  PublisherEpochResult result(
    PublisherEpochStatus status,
    bool session_changed = false) const noexcept
  {
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
