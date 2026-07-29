#ifndef CLEANBOT_RTK__RTCM3_FRAME_BUFFER_HPP_
#define CLEANBOT_RTK__RTCM3_FRAME_BUFFER_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

namespace cleanbot {
namespace rtk {

class Rtcm3FrameBuffer {
 public:
  explicit Rtcm3FrameBuffer(std::size_t max_buffer_size = 8192u);

  void append(const std::vector<std::uint8_t>& bytes);
  bool pop(std::vector<std::uint8_t>& frame);
  void clear();
  std::size_t size() const;
  std::size_t invalid_crc_count() const;

 private:
  static std::uint32_t crc24q(
      const std::vector<std::uint8_t>& bytes, std::size_t length);

  std::size_t max_buffer_size_;
  std::size_t invalid_crc_count_{0u};
  std::vector<std::uint8_t> data_;
};

}  // namespace rtk
}  // namespace cleanbot

#endif  // CLEANBOT_RTK__RTCM3_FRAME_BUFFER_HPP_
