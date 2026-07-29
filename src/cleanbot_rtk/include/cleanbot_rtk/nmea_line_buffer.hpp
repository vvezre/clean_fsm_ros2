#ifndef CLEANBOT_RTK__NMEA_LINE_BUFFER_HPP_
#define CLEANBOT_RTK__NMEA_LINE_BUFFER_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace cleanbot {
namespace rtk {

class NmeaLineBuffer {
 public:
  explicit NmeaLineBuffer(std::size_t capacity = 8192u);

  void append(const std::vector<std::uint8_t>& bytes);
  bool pop(std::string& line);
  void clear();
  std::size_t size() const;

 private:
  void trim();

  std::size_t capacity_;
  std::vector<std::uint8_t> data_;
};

}  // namespace rtk
}  // namespace cleanbot

#endif  // CLEANBOT_RTK__NMEA_LINE_BUFFER_HPP_
