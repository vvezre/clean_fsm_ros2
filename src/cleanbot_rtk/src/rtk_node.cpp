/*
 * 文件作用：RTK定位与NTRIP差分链路节点。
 *
 * 输入：从RTK串口接收NMEA数据；从NTRIP服务接收RTCM3差分数据。
 * 输出：发布 /rtk/fix，提供原始坐标、车体中心坐标、航向、固定解有效性和NTRIP状态。
 *
 * 主流程：串口字节流 -> 提取NMEA语句 -> 解析GGA/航向 -> 时间同步
 *          -> 天线坐标转换为车体中心坐标 -> 有效性判断 -> 发布RtkFix。
 * 差分链路：NTRIP数据 -> 提取并校验完整RTCM3帧 -> 写入RTK串口，提高定位解算精度。
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "cleanbot_common/qos_profiles.hpp"
#include "cleanbot_config/config_client.hpp"
#include "cleanbot_interfaces/msg/rtk_fix.hpp"
#include "cleanbot_rtk/nmea_line_buffer.hpp"
#include "cleanbot_rtk/nmea_parser.hpp"
#include "cleanbot_rtk/ntrip_client.hpp"
#include "cleanbot_rtk/rtcm3_frame_buffer.hpp"
#include "cleanbot_rtk/rtk_sample_synchronizer.hpp"
#include "cleanbot_rtk/rtk_serial.hpp"
#include "cleanbot_rtk/rtk_validity.hpp"
#include "cleanbot_rtk/vehicle_center_transform.hpp"
#include "rclcpp/rclcpp.hpp"

namespace cleanbot {
namespace rtk {

class RtkNode : public rclcpp::Node {
 public:
  RtkNode() : Node("rtk_node") {
    // 1. 发布器可以先建立，但配置READY之前不创建串口和NTRIP线程。
    publisher_ = create_publisher<cleanbot_interfaces::msg::RtkFix>(
        "/rtk/fix", common::rtk_fix_qos());

    // 2. RTK和NTRIP参数统一从配置中心读取，密码仅在本节点内部按敏感配置获取。
    config_client_ = std::make_unique<config::ConfigClient>(
        this,
        std::vector<std::string>{
            "rtk.port",
            "rtk.baudrate",
            "rtk.save_config_on_connect",
            "rtk.nmea_sync_threshold_sec",
            "rtk.nmea_max_age_sec",
            "rtk.heading_max_age_sec",
            "rtk.center_offset_along_heading_m",
            "rtk.center_offset_right_m",
            "ntrip.enabled",
            "ntrip.host",
            "ntrip.port",
            "ntrip.mountpoint",
            "ntrip.username",
            "ntrip.password",
            "ntrip.gga_interval_sec",
            "ntrip.connect_timeout_sec",
            "ntrip.reconnect_interval_sec",
            "ntrip.rtcm_timeout_sec",
        },
        true,
        [this](const config::ConfigSnapshot& snapshot, const bool initial) {
          configure(snapshot, initial);
        });
  }

  ~RtkNode() override {
    if (ntrip_client_) {
      ntrip_client_->stop();
    }
    if (rtk_serial_) {
      rtk_serial_->stop();
    }
  }

 private:
  // NTRIP差分数据入口：处理网络分包/粘包，只把CRC正确的完整RTCM3帧写入RTK串口。
  void onRtcmBytes(const std::vector<std::uint8_t>& bytes) {
    std::vector<std::vector<std::uint8_t>> frames;
    {
      std::lock_guard<std::mutex> lock(rtcm_mutex_);
      rtcm_frame_buffer_.append(bytes);
      std::vector<std::uint8_t> frame;
      while (rtcm_frame_buffer_.pop(frame)) {
        frames.push_back(frame);
      }
    }
    if (!rtk_serial_) {
      return;
    }
    for (const auto& frame : frames) {
      rtk_serial_->enqueueWrite(frame);
    }
  }

  // RTK串口数据入口：从字节流提取完整NMEA语句，并分别解析航向和GGA定位数据。
  void onSerialData(const std::vector<std::uint8_t>& bytes) {
    std::lock_guard<std::mutex> lock(nmea_mutex_);
    line_buffer_.append(bytes);
    std::string sentence;
    while (line_buffer_.pop(sentence)) {
      const double received_at = monotonicSeconds();
      const HeadingParseResult heading = parser_.parse_heading(sentence);
      if (heading.parsed) {
        // 航向帧先缓存，等待时间接近的GGA帧进行组合。
        synchronizer_->observe_heading(heading.data, received_at);
        continue;
      }

      const GgaParseResult gga = parser_.parse_gga(sentence);
      if (!gga.parsed) {
        continue;
      }
      if (gga.data.position_valid && ntrip_client_) {
        // 将最新有效GGA提供给NTRIP客户端，作为周期性上行位置报文。
        ntrip_client_->updateGga(gga.data.sentence);
      }
      last_gga_monotonic_sec_ = received_at;
      const SynchronizeResult synchronized = synchronizer_->observe_gga(gga.data, received_at);
      if (synchronized.produced || synchronized.stale) {
        // 同步成功或已经判定过期时都发布状态，让控制层及时知道RTK当前是否可用。
        publishSample(synchronized.sample);
      }
    }
  }

  // 将同步后的RTK样本转换成ROS2消息，并统一计算车体中心坐标和fixed_valid。
  void publishSample(const RtkSample& sample) {
    cleanbot_interfaces::msg::RtkFix message;
    message.stamp = now();
    message.serial_connected = serial_connected_;
    message.coordinate_valid = sample.gga.position_valid;
    message.raw_lat = sample.gga.lat;
    message.raw_lon = sample.gga.lon;
    message.heading_deg = sample.heading_deg;
    message.heading_valid = sample.heading_valid;
    message.heading_age_sec = sample.heading_age_sec;
    message.gga_age_sec = sample.gga_age_sec;
    message.fix_quality = sample.gga.fix_quality;
    message.satellite_count = sample.gga.satellite_count;
    message.hdop = sample.gga.hdop;
    message.source = "nmea_gga_heading_vehicle_center";

    CenterPoint center;
    if (sample.gga.position_valid && sample.heading_valid) {
      // GGA是RTK天线位置；结合航向和安装偏移换算纠偏真正使用的车体中心位置。
      center = center_transform_.compute(
          sample.gga.lat,
          sample.gga.lon,
          sample.heading_deg,
          center_offset_along_heading_m_,
          center_offset_right_m_);
    }
    if (center.valid) {
      last_center_ = center;
      message.lat = center.lat;
      message.lon = center.lon;
      message.center_valid = true;
    } else {
      message.lat = last_center_.valid ? last_center_.lat : sample.gga.lat;
      message.lon = last_center_.valid ? last_center_.lon : sample.gga.lon;
      message.center_valid = false;
    }

    RtkValidityInput validity;
    validity.serial_connected = message.serial_connected;
    validity.coordinate_valid = message.coordinate_valid;
    validity.center_valid = message.center_valid;
    validity.heading_valid = message.heading_valid;
    validity.fix_quality = message.fix_quality;
    validity.gga_age_sec = message.gga_age_sec;
    validity.max_gga_age_sec = nmea_max_age_sec_;
    message.fixed_valid = is_fixed_valid(validity);

    // NTRIP连接状态与定位结果一起发布，RTK看门狗可据此区分定位失效原因。
    populateNtripStatus(message);
    last_message_ = message;
    has_last_message_ = true;
    publisher_->publish(message);
  }

  // 补充NTRIP连接状态和最近一帧RTCM的时间间隔。
  void populateNtripStatus(cleanbot_interfaces::msg::RtkFix& message) const {
    const NtripStatus ntrip = ntrip_client_ ? ntrip_client_->status() : NtripStatus();
    message.ntrip_enabled = ntrip.enabled;
    message.ntrip_configured = ntrip.configured;
    message.ntrip_connected = ntrip.connected;
    message.ntrip_last_error = ntrip.last_error;
    message.rtcm_age_sec = ntrip.last_rtcm_monotonic_sec < 0.0
        ? -1.0
        : std::max(0.0, monotonicSeconds() - ntrip.last_rtcm_monotonic_sec);
  }

  // 每200毫秒更新数据年龄；即使串口不再产生新GGA，也能主动发布“RTK已过期”。
  void publishFreshness() {
    cleanbot_interfaces::msg::RtkFix message;
    {
      std::lock_guard<std::mutex> lock(nmea_mutex_);
      if (has_last_message_) {
        message = last_message_;
        const double stream_age = last_gga_monotonic_sec_ < 0.0
            ? -1.0
            : std::max(0.0, monotonicSeconds() - last_gga_monotonic_sec_);
        if (stream_age >= 0.0) {
          message.gga_age_sec = std::max(0.0, last_message_.gga_age_sec) + stream_age;
          message.heading_age_sec = std::max(0.0, last_message_.heading_age_sec) + stream_age;
        }
      } else {
        message.source = "rtk_freshness_heartbeat";
        message.gga_age_sec = -1.0;
        message.heading_age_sec = -1.0;
        message.rtcm_age_sec = -1.0;
      }

      message.stamp = now();
      message.serial_connected = serial_connected_;
      if (!should_publish_freshness_heartbeat(
              has_last_message_,
              message.serial_connected,
              message.gga_age_sec,
              nmea_max_age_sec_)) {
        return;
      }
      if (!message.serial_connected || message.gga_age_sec < 0.0 ||
          message.gga_age_sec > nmea_max_age_sec_) {
        // 串口断开或数据超时后立即撤销位置、中心点和航向有效标志，禁止控制层继续使用旧坐标。
        message.coordinate_valid = false;
        message.center_valid = false;
        message.heading_valid = false;
      }

      RtkValidityInput validity;
      validity.serial_connected = message.serial_connected;
      validity.coordinate_valid = message.coordinate_valid;
      validity.center_valid = message.center_valid;
      validity.heading_valid = message.heading_valid;
      validity.fix_quality = message.fix_quality;
      validity.gga_age_sec = message.gga_age_sec;
      validity.max_gga_age_sec = nmea_max_age_sec_;
      message.fixed_valid = is_fixed_valid(validity);
      populateNtripStatus(message);
    }
    publisher_->publish(message);
  }

  // 串口连接变化回调：断线时清空残留半包和同步历史，并立即发布最新失效状态。
  void onSerialConnection(const bool connected, const std::string& detail) {
    {
      std::lock_guard<std::mutex> lock(nmea_mutex_);
      serial_connected_ = connected;
      if (!connected) {
        line_buffer_.clear();
        if (synchronizer_) {
          synchronizer_->reset();
        }
      }
    }
    if (connected) {
      RCLCPP_INFO(get_logger(), "%s", detail.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "%s", detail.c_str());
    }
    publishFreshness();
  }

  // 返回单调时钟秒数，用于计算NMEA/RTCM数据年龄，避免受系统时间校准影响。
  static double monotonicSeconds() {
    return std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  // 读取RTK/NTRIP配置，并启动串口、同步器和差分客户端。
  void configure(const config::ConfigSnapshot& snapshot, const bool initial) {
    if (!initial && configured_) {
      RCLCPP_WARN(
          get_logger(),
          "RTK configuration changed; restart required before it becomes active");
      return;
    }
    const auto port = snapshot.get_string("rtk.port");
    const auto baudrate = snapshot.get_integer("rtk.baudrate");
    const auto save_config_on_connect =
        snapshot.get_boolean("rtk.save_config_on_connect");
    const auto sync_threshold = snapshot.get_double("rtk.nmea_sync_threshold_sec");
    nmea_max_age_sec_ = snapshot.get_double("rtk.nmea_max_age_sec");
    const auto heading_max_age = snapshot.get_double("rtk.heading_max_age_sec");
    center_offset_along_heading_m_ =
        snapshot.get_double("rtk.center_offset_along_heading_m");
    center_offset_right_m_ = snapshot.get_double("rtk.center_offset_right_m");

    NtripConfig ntrip_config;
    ntrip_config.enabled = snapshot.get_boolean("ntrip.enabled");
    ntrip_config.host = snapshot.get_string("ntrip.host");
    ntrip_config.port = static_cast<std::uint16_t>(
        snapshot.get_integer("ntrip.port"));
    ntrip_config.mountpoint = snapshot.get_string("ntrip.mountpoint");
    ntrip_config.username = snapshot.get_string("ntrip.username");
    ntrip_config.password = snapshot.get_string("ntrip.password");
    ntrip_config.gga_interval_sec = snapshot.get_double("ntrip.gga_interval_sec");
    ntrip_config.connect_timeout_sec =
        snapshot.get_double("ntrip.connect_timeout_sec");
    ntrip_config.reconnect_interval_sec =
        snapshot.get_double("ntrip.reconnect_interval_sec");
    ntrip_config.rtcm_timeout_sec = snapshot.get_double("ntrip.rtcm_timeout_sec");

    synchronizer_ = std::make_unique<RtkSampleSynchronizer>(
        sync_threshold, nmea_max_age_sec_, heading_max_age);
    rtk_serial_ = std::make_unique<RtkSerial>(
        port,
        static_cast<unsigned int>(baudrate),
        std::bind(&RtkNode::onSerialData, this, std::placeholders::_1),
        std::bind(
            &RtkNode::onSerialConnection,
            this,
            std::placeholders::_1,
            std::placeholders::_2),
        save_config_on_connect);
    ntrip_client_ = std::make_unique<NtripClient>(
        ntrip_config,
        [this](const std::vector<std::uint8_t>& bytes) {
          onRtcmBytes(bytes);
        });
    configured_ = true;
    rtk_serial_->start();
    ntrip_client_->start();
    freshness_timer_ = create_wall_timer(
        std::chrono::milliseconds(200),
        std::bind(&RtkNode::publishFreshness, this));
    RCLCPP_INFO(
        get_logger(),
        "RTK configuration ready port=%s baudrate=%lld center_offset=(%.3f forward, %.3f right) ntrip=%s",
        port.c_str(),
        static_cast<long long>(baudrate),
        center_offset_along_heading_m_,
        center_offset_right_m_,
        ntrip_config.enabled ? "enabled" : "disabled");
  }

  NmeaLineBuffer line_buffer_;
  Rtcm3FrameBuffer rtcm_frame_buffer_;
  NmeaParser parser_;
  std::unique_ptr<RtkSampleSynchronizer> synchronizer_;
  VehicleCenterTransform center_transform_;
  CenterPoint last_center_;
  double center_offset_along_heading_m_{0.10};
  double center_offset_right_m_{0.18};
  double nmea_max_age_sec_{2.0};
  double last_gga_monotonic_sec_{-1.0};
  bool serial_connected_{false};
  bool has_last_message_{false};
  cleanbot_interfaces::msg::RtkFix last_message_;
  std::mutex nmea_mutex_;
  std::mutex rtcm_mutex_;
  std::unique_ptr<config::ConfigClient> config_client_;
  std::unique_ptr<RtkSerial> rtk_serial_;
  std::unique_ptr<NtripClient> ntrip_client_;
  bool configured_{false};
  rclcpp::TimerBase::SharedPtr freshness_timer_;
  rclcpp::Publisher<cleanbot_interfaces::msg::RtkFix>::SharedPtr publisher_;
};

}  // namespace rtk
}  // namespace cleanbot

// 程序入口：初始化ROS2并运行RTK节点，回调持续处理串口、NTRIP和新鲜度定时器。
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<cleanbot::rtk::RtkNode>());
  rclcpp::shutdown();
  return 0;
}
