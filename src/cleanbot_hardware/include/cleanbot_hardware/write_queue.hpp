/*
 * 文件作用：声明旧下位机串口的有界优先级发送队列。
 *
 * 输入：已经编码好的命令批次、写入优先级和可选合并键。
 * 输出：按安全级别排序的待写数据及写入/替换/淘汰事件。
 * 安全规则：正在异步写入的队首不可移动；刹车和高优先级帧不被低优先级命令淘汰。
 */
#ifndef CLEANBOT_HARDWARE__WRITE_QUEUE_HPP_
#define CLEANBOT_HARDWARE__WRITE_QUEUE_HPP_

#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace cleanbot {
namespace hardware {

// 数值越大越优先；Critical为传输层预留的最高等级。
enum class WritePriority : std::uint8_t {
  kContinuous = 10,
  kFinite = 20,
  kSafety = 30,
  kCritical = 40,
};

// 队列项终态，用于上层映射ROS2命令生命周期。
enum class WriteQueueEvent : std::uint8_t {
  kWritten = 0,
  kSuperseded = 1,
  kEvicted = 2,
};

using WriteQueueCallback = std::function<void(WriteQueueEvent)>;

/// 根据已编码命令帧的状态和刹车字段，推断重发帧应使用的写入优先级。
WritePriority classify_command_frame(const std::vector<std::uint8_t>& frame);

// 单线程使用的有界稳定优先级队列，由LowerMachineSerial的I/O线程独占。
class PriorityWriteQueue {
 public:
  /// 创建有容量上限的优先级发送队列。
  explicit PriorityWriteQueue(std::size_t max_size = 32u);

  /// 按优先级插入协议帧，并按合并键替换旧连续命令；必要时淘汰低优先级帧。
  bool enqueue(
      const std::vector<std::uint8_t>& frame,
      WritePriority priority,
      const std::string& coalescing_key,
      bool preserve_front,
      WriteQueueCallback callback = WriteQueueCallback());
  /// 返回当前最高优先级队首帧的只读引用。
  const std::vector<std::uint8_t>& front() const;
  /// 返回队首帧共享句柄，保证异步写入期间帧内存持续有效。
  std::shared_ptr<const std::vector<std::uint8_t>> front_handle() const;
  /// 返回队首帧的发送结果回调；没有注册回调时返回空函数。
  WriteQueueCallback front_callback() const;
  /// 移除已经完成写入的队首帧。
  void pop_front();
  /// 清空全部待发送帧。
  void clear();
  /// 判断当前是否没有待发送帧。
  bool empty() const;
  /// 返回当前待发送帧数量。
  std::size_t size() const;

 private:
  // 共享帧句柄确保Boost.Asio异步写期间底层字节内存不会失效。
  struct Item {
    std::shared_ptr<std::vector<std::uint8_t>> frame;
    WritePriority priority{WritePriority::kContinuous};
    std::string coalescing_key;
    WriteQueueCallback callback;
  };

  std::size_t max_size_;
  std::deque<Item> items_;
};

}  // namespace hardware
}  // namespace cleanbot

#endif  // CLEANBOT_HARDWARE__WRITE_QUEUE_HPP_
