#ifndef CLEANBOT_RTK__NMEA_LINE_BUFFER_HPP_
#define CLEANBOT_RTK__NMEA_LINE_BUFFER_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

// 文件作用：声明串口 NMEA 字节流的定界、缓存和逐行提取工具。
namespace cleanbot {
namespace rtk {

class NmeaLineBuffer {
 public:
  // 创建容量受限的 NMEA 行缓存。
  explicit NmeaLineBuffer(std::size_t capacity = 8192u);

  // 追加串口接收的原始字节。
  void append(const std::vector<std::uint8_t>& bytes);
  // 取出一条完整 NMEA 行；暂无完整行时返回 false。
  bool pop(std::string& line);
  // 清空尚未处理的字节。
  void clear();
  // 返回当前缓存字节数。
  std::size_t size() const;

 private:
  // 在容量超限时丢弃最早数据，保证内存有界。
  void trim();

  std::size_t capacity_;
  std::vector<std::uint8_t> data_;
};

}  // namespace rtk
}  // namespace cleanbot

#endif  // CLEANBOT_RTK__NMEA_LINE_BUFFER_HPP_
