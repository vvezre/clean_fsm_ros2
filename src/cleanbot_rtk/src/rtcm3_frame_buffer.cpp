#include "cleanbot_rtk/rtcm3_frame_buffer.hpp"

namespace cleanbot {
namespace rtk {

namespace {
constexpr std::uint8_t kRtcm3Preamble = 0xd3u;
constexpr std::size_t kHeaderLength = 3u;
constexpr std::size_t kCrcLength = 3u;
}  // namespace

Rtcm3FrameBuffer::Rtcm3FrameBuffer(const std::size_t max_buffer_size)
    : max_buffer_size_(max_buffer_size < kHeaderLength + kCrcLength
          ? kHeaderLength + kCrcLength
          : max_buffer_size) {}

void Rtcm3FrameBuffer::append(const std::vector<std::uint8_t>& bytes) {
  if (bytes.empty()) {
    return;
  }
  data_.insert(data_.end(), bytes.begin(), bytes.end());
  if (data_.size() > max_buffer_size_) {
    data_.erase(
        data_.begin(),
        data_.begin() + static_cast<std::ptrdiff_t>(data_.size() - max_buffer_size_));
  }
}

bool Rtcm3FrameBuffer::pop(std::vector<std::uint8_t>& frame) {
  while (true) {
    std::size_t start_index = 0u;
    while (start_index < data_.size() && data_[start_index] != kRtcm3Preamble) {
      ++start_index;
    }
    if (start_index == data_.size()) {
      data_.clear();
      return false;
    }
    if (start_index > 0u) {
      data_.erase(
          data_.begin(),
          data_.begin() + static_cast<std::ptrdiff_t>(start_index));
    }
    if (data_.size() < kHeaderLength) {
      return false;
    }
    if ((data_[1] & 0xfcu) != 0u) {
      data_.erase(data_.begin());
      continue;
    }

    const std::size_t payload_length =
        (static_cast<std::size_t>(data_[1] & 0x03u) << 8u) |
        static_cast<std::size_t>(data_[2]);
    const std::size_t frame_length = kHeaderLength + payload_length + kCrcLength;
    if (data_.size() < frame_length) {
      return false;
    }

    const std::uint32_t expected_crc =
        (static_cast<std::uint32_t>(data_[frame_length - 3u]) << 16u) |
        (static_cast<std::uint32_t>(data_[frame_length - 2u]) << 8u) |
        static_cast<std::uint32_t>(data_[frame_length - 1u]);
    if (crc24q(data_, frame_length - kCrcLength) != expected_crc) {
      ++invalid_crc_count_;
      data_.erase(data_.begin());
      continue;
    }

    frame.assign(data_.begin(), data_.begin() + static_cast<std::ptrdiff_t>(frame_length));
    data_.erase(data_.begin(), data_.begin() + static_cast<std::ptrdiff_t>(frame_length));
    return true;
  }
}

void Rtcm3FrameBuffer::clear() { data_.clear(); }

std::size_t Rtcm3FrameBuffer::size() const { return data_.size(); }

std::size_t Rtcm3FrameBuffer::invalid_crc_count() const {
  return invalid_crc_count_;
}

std::uint32_t Rtcm3FrameBuffer::crc24q(
    const std::vector<std::uint8_t>& bytes, const std::size_t length) {
  std::uint32_t crc = 0u;
  const std::size_t end = length < bytes.size() ? length : bytes.size();
  for (std::size_t index = 0u; index < end; ++index) {
    crc ^= static_cast<std::uint32_t>(bytes[index]) << 16u;
    for (int bit = 0; bit < 8; ++bit) {
      crc <<= 1u;
      if ((crc & 0x1000000u) != 0u) {
        crc ^= 0x1864cfbu;
      }
    }
  }
  return crc & 0x00ffffffu;
}

}  // namespace rtk
}  // namespace cleanbot
