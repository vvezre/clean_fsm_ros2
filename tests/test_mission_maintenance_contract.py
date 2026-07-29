import re
import unittest
from pathlib import Path


WORKSPACE = Path(__file__).resolve().parents[1]
PACKAGE = WORKSPACE / "src" / "cleanbot_mission"
SOURCE = PACKAGE / "src" / "mission_manager_node.cpp"
CMAKE = PACKAGE / "CMakeLists.txt"
PACKAGE_XML = PACKAGE / "package.xml"


class MissionMaintenanceContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = SOURCE.read_text(encoding="utf-8")
        cls.cmake = CMAKE.read_text(encoding="utf-8")
        cls.package_xml = PACKAGE_XML.read_text(encoding="utf-8")

    def body_between(self, start, end):
        begin = self.source.index(start)
        finish = self.source.index(end, begin)
        return self.source[begin:finish]

    def test_fixed_store_is_restored_before_action_servers(self):
        for token in (
            '#include "cleanbot_mission/maintenance_runtime.hpp"',
            '"/var/lib/cleanbot/runtime/maintenance.lock"',
            "MaintenanceRuntime maintenance_runtime_",
            "maintenance_runtime_.initialize(true)",
        ):
            self.assertIn(token, self.source)
        restored = self.source.index("maintenance_runtime_.initialize(true)")
        first_action = self.source.index(
            "rclcpp_action::create_server<ExecuteCleaning>"
        )
        self.assertLess(restored, first_action)
        self.assertNotIn("declare_parameter", self.source)
        self.assertNotIn("initializeGenesis", self.source)

    def test_latched_state_topic_and_service_are_exposed(self):
        for token in (
            '#include "cleanbot_interfaces/msg/maintenance_state.hpp"',
            '#include "cleanbot_interfaces/srv/set_maintenance_mode.hpp"',
            '"/system/maintenance_state"',
            '"/system/set_maintenance"',
            "common::latched_status_qos()",
            "onSetMaintenance",
            "publishMaintenanceState",
        ):
            self.assertIn(token, self.source)

    def test_all_action_goals_check_maintenance_before_other_admission(self):
        callbacks = (
            ("onGoal(", "onCancel("),
            ("onWaypointGoal(", "onWaypointCancel("),
            ("onReturnHomeGoal(", "onReturnHomeCancel("),
            ("onRecoverGoal(", "onRecoverCancel("),
        )
        for start, end in callbacks:
            body = self.body_between(start, end)
            gate = body.index("maintenanceAdmissionClosedLocked")
            configured = body.index("configured_")
            reservation = body.index("goal_reserved_")
            self.assertLess(gate, configured, start)
            self.assertLess(gate, reservation, start)

    def test_service_enforces_enable_and_release_tokens_under_mutex(self):
        body = self.body_between("void onSetMaintenance(", "void onRtkFix(")
        for token in (
            "std::lock_guard<std::mutex> lock(mutex_)",
            "request->enable",
            "request->generation != 0u",
            "maintenance_runtime_.enable(",
            "maintenance_runtime_.disable(",
            "response->accepted",
            "response->gate_active",
            "response->ready",
            "response->generation",
        ):
            self.assertIn(token, body)
        self.assertNotIn("sleep", body)
        self.assertNotIn("join", body)
        self.assertNotIn("publish(", body)

    def test_hardware_uses_message_info_full_gid_and_monotonic_refresh(self):
        for token in (
            "const rclcpp::MessageInfo& info",
            "info.get_rmw_message_info().publisher_gid",
            "RMW_GID_STORAGE_SIZE",
            "implementation_identifier",
            "maintenance_runtime_.observeHardware(",
            "maintenance_runtime_.refreshHardware(",
            "MaintenanceHardwareStatus::kRetired",
            "monotonicNanoseconds()",
        ):
            self.assertIn(token, self.source)
        self.assertIn("find_package(rmw REQUIRED)", self.cmake)
        self.assertRegex(self.package_xml, r"<depend>rmw</depend>")

    def test_final_and_status_evidence_use_message_request_id(self):
        status = self.body_between(
            "void onCommandStatus(", "void onFinalCommand("
        )
        final = self.body_between(
            "void onFinalCommand(", "void execute("
        )
        for token in (
            "evidence.generation = status->request_id",
            "evidence.request_id = status->request_id",
            "evidence.command_id = status->command_id",
            "maintenance_runtime_.observeCommandStatus(evidence)",
        ):
            self.assertIn(token, status)
        for token in (
            "evidence.generation = command->request_id",
            "evidence.request_id = command->request_id",
            "evidence.command_id = command->command_id",
            "evidence.brush_speed = command->brush_speed",
            "maintenance_runtime_.observeFinalCommand(evidence)",
        ):
            self.assertIn(token, final)

    def test_periodic_callback_publishes_vehicle_and_maintenance_state(self):
        body = self.body_between(
            "void publishPeriodicState(", "bool isLowBattery("
        )
        self.assertIn("maintenance_runtime_.setMissionIdle(", body)
        self.assertIn("maintenance_runtime_.refreshHardware(", body)
        self.assertIn("publishMaintenanceState(maintenance_snapshot)", body)
        self.assertIn("vehicle_state_publisher_->publish(message)", body)


if __name__ == "__main__":
    unittest.main()
