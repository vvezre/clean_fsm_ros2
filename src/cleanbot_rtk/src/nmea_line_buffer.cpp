/*
 * 文件作用：NMEA行缓冲实现：从串口字节流中重组完整NMEA文本行。
 * 说明：本文件只负责本模块的实现逻辑，输入输出和线程约束以对应头文件为准。
 */
#include "cleanbot_rtk/nmea_line_buffer.hpp"

namespace cleanbot {
namespace rtk {

// 以给定上限创建串口 NMEA 字节缓存。
NmeaLineBuffer::NmeaLineBuffer(const std::size_t capacity)
    : capacity_(capacity == 0u ? 1u : capacity) {}

// 追加新接收字节，并在超限时保留最近数据。
void NmeaLineBuffer::append(const std::vector<std::uint8_t>& bytes) {
  data_.insert(data_.end(), bytes.begin(), bytes.end());
  trim();
}

// 提取以换行结束的一条完整 NMEA 文本行。
bool NmeaLineBuffer::pop(std::string& line) {
  line.clear();
  while (true) {
    std::size_t start = 0u;
    while (start < data_.size() && data_[start] != '$') {
      ++start;
    }
    data_.erase(data_.begin(), data_.begin() + static_cast<std::ptrdiff_t>(start));
    if (data_.empty()) {
      return false;
    }

    std::size_t newline = 0u;
    while (newline < data_.size() && data_[newline] != '\n') {
      ++newline;
    }
    if (newline == data_.size()) {
      return false;
    }

    std::size_t nested_start = 1u;
    while (nested_start < newline && data_[nested_start] != '$') {
      ++nested_start;
    }
    if (nested_start < newline) {
      data_.erase(
          data_.begin(), data_.begin() + static_cast<std::ptrdiff_t>(nested_start));
      continue;
    }

    std::size_t line_end = newline;
    if (line_end > 0u && data_[line_end - 1u] == '\r') {
      --line_end;
    }
    line.assign(
        data_.begin(), data_.begin() + static_cast<std::ptrdiff_t>(line_end));
    data_.erase(
        data_.begin(), data_.begin() + static_cast<std::ptrdiff_t>(newline + 1u));
    if (!line.empty()) {
      return true;
    }
  }
}

// 清除未处理的串口字节。
void NmeaLineBuffer::clear() { data_.clear(); }

// 返回当前尚未组成完整行的缓存长度。
std::size_t NmeaLineBuffer::size() const { return data_.size(); }

// 丢弃过旧字节，使缓存不超过设定容量。
void NmeaLineBuffer::trim() {
  if (data_.size() <= capacity_) {
    return;
  }

  const std::size_t keep_begin = data_.size() - capacity_;
  std::size_t start = keep_begin;
  while (start < data_.size() && data_[start] != '$') {
    ++start;
  }
  const std::size_t erase_count = start == data_.size() ? keep_begin : start;
  data_.erase(data_.begin(), data_.begin() + static_cast<std::ptrdiff_t>(erase_count));
}

}  // namespace rtk
}  // namespace cleanbot
