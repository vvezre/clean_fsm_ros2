#ifndef CLEANBOT_MISSION__MAINTENANCE_RUNTIME_HPP_
#define CLEANBOT_MISSION__MAINTENANCE_RUNTIME_HPP_

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>

#include "cleanbot_common/publisher_epoch_tracker.hpp"
#include "cleanbot_mission/maintenance_gate.hpp"
#include "cleanbot_mission/maintenance_store.hpp"

namespace cleanbot {
namespace mission {

struct MaintenanceRuntimeSnapshot {
  bool initialized{false};
  bool store_fault{false};
  bool admission_closed{true};
  std::uint64_t generation{0u};
  std::uint64_t last_generation{0u};
  bool gate_active{false};
  bool mission_idle{false};
  bool command_gate_applied{false};
  bool brake_acknowledged{false};
  bool hardware_fresh{false};
  bool linear_speed_zero{false};
  bool angular_speed_zero{false};
  bool brush_off{false};
  bool ready{false};
  std::string requester;
  std::string reason;
  std::string phase;
  std::string blocker_code;
  std::string message;
};

struct MaintenanceTransitionResult {
  bool accepted{false};
  bool gate_active{false};
  bool ready{false};
  std::uint64_t generation{0u};
  std::string code;
  std::string message;
};

struct MaintenanceHardwareSample {
  std::uint64_t frame_sequence{0u};
  bool connected{false};
  std::int32_t x_speed{0};
  std::int32_t z_speed{0};
  std::int32_t brush_speed{0};
};

enum class MaintenanceHardwareStatus {
  kAccepted,
  kRetired,
  kRevoked,
};

struct MaintenanceHardwareObservation {
  MaintenanceHardwareStatus status{MaintenanceHardwareStatus::kRevoked};
  std::uint64_t publisher_epoch{0u};
  bool session_changed{false};
};

class MaintenanceRuntime {
 public:
  explicit MaintenanceRuntime(
      std::filesystem::path state_path,
      std::size_t maximum_retired_publishers = 1024u,
      std::uint64_t maximum_publisher_epoch =
          cleanbot::common::PublisherEpochTracker::
              default_maximum_epoch());

  MaintenanceRuntime(const MaintenanceRuntime&) = delete;
  MaintenanceRuntime& operator=(const MaintenanceRuntime&) = delete;
  MaintenanceRuntime(MaintenanceRuntime&&) = delete;
  MaintenanceRuntime& operator=(MaintenanceRuntime&&) = delete;

  // Initialization is single-shot. Repeated calls return the latched result
  // without rereading persistent state or mutating the restored gate.
  bool initialize(bool mission_idle) noexcept;
  void setMissionIdle(bool mission_idle) noexcept;
  MaintenanceTransitionResult enable(
      const std::string& requester,
      const std::string& reason,
      bool mission_idle) noexcept;
  MaintenanceTransitionResult disable(
      std::uint64_t generation,
      const std::string& requester) noexcept;
  void observeFinalCommand(
      const FinalCommandEvidence& command) noexcept;
  void observeCommandStatus(
      const CommandStatusEvidence& status) noexcept;
  MaintenanceHardwareObservation observeHardware(
      const cleanbot::common::PublisherIdentity& publisher,
      const MaintenanceHardwareSample& sample,
      std::uint64_t observed_at_nanoseconds) noexcept;
  void refreshHardware(
      std::uint64_t now_nanoseconds,
      std::uint64_t freshness_timeout_nanoseconds) noexcept;

  const MaintenanceRuntimeSnapshot& snapshot() const noexcept;
  bool initialized() const noexcept;
  bool storeFault() const noexcept;
  bool admissionClosed() const noexcept;
  std::uint64_t lastGeneration() const noexcept;

 private:
  static MaintenanceRuntimeSnapshot initialSnapshot();
  static MaintenanceRuntimeSnapshot emergencyFaultSnapshot();
  static bool validRecord(const MaintenanceStoreRecord& record) noexcept;
  static const char* storeCodeName(MaintenanceStoreCode code) noexcept;
  static bool isOperationalFailure(
      MaintenanceStoreCode code) noexcept;
  static void swapSnapshots(
      MaintenanceRuntimeSnapshot& lhs,
      MaintenanceRuntimeSnapshot& rhs) noexcept;

  bool restore(
      const MaintenanceStoreRecord& record,
      bool mission_idle) noexcept;
  bool refreshHealthySnapshot(
      const MaintenanceStoreRecord& record) noexcept;
  bool refreshHealthySnapshot(
      const std::string& requester,
      const std::string& reason) noexcept;
  MaintenanceTransitionResult transitionResult(
      bool accepted,
      MaintenanceStoreCode code,
      const std::string& message,
      std::uint64_t generation) const noexcept;
  MaintenanceTransitionResult rejectUnavailable(
      const char* code,
      const char* message) const noexcept;
  void restoreAdmissionFromRecord() noexcept;
  void applyUncertainActivation(
      const MaintenanceStoreResult& result,
      bool mission_idle) noexcept;
  void latchStoreFault(const std::string& message) noexcept;
  void latchEmergencyFault() noexcept;

  MaintenanceStore store_;
  MaintenanceGate gate_;
  bool initialized_{false};
  bool store_fault_{false};
  bool admission_closed_{true};
  std::optional<MaintenanceStoreRecord> record_;
  cleanbot::common::PublisherEpochTracker hardware_publishers_;
  bool has_latest_hardware_{false};
  HardwareEvidence latest_hardware_;
  std::uint64_t latest_hardware_observed_at_nanoseconds_{0u};
  MaintenanceRuntimeSnapshot snapshot_;
  MaintenanceRuntimeSnapshot emergency_fault_snapshot_;
};

}  // namespace mission
}  // namespace cleanbot

#endif  // CLEANBOT_MISSION__MAINTENANCE_RUNTIME_HPP_
