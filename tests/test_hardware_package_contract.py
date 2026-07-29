import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


WORKSPACE = Path(__file__).resolve().parents[1]
PACKAGE = WORKSPACE / "src" / "cleanbot_hardware"


class HardwarePackageContractTest(unittest.TestCase):
    def test_hardware_package_contains_protocol_library_node_and_tests(self):
        required = [
            "package.xml",
            "CMakeLists.txt",
            "include/cleanbot_hardware/frame_buffer.hpp",
            "include/cleanbot_hardware/status_frame_parser.hpp",
            "include/cleanbot_hardware/command_encoder.hpp",
            "include/cleanbot_hardware/ack_codec.hpp",
            "include/cleanbot_hardware/command_lifecycle.hpp",
            "include/cleanbot_hardware/lower_machine_serial.hpp",
            "include/cleanbot_hardware/write_queue.hpp",
            "include/cleanbot_hardware/gateway_readiness.hpp",
            "src/frame_buffer.cpp",
            "src/status_frame_parser.cpp",
            "src/command_encoder.cpp",
            "src/ack_codec.cpp",
            "src/command_lifecycle.cpp",
            "src/lower_machine_serial.cpp",
            "src/write_queue.cpp",
            "src/gateway_readiness.cpp",
            "src/lower_machine_node.cpp",
            "test/test_frame_buffer.cpp",
            "test/test_status_frame_parser.cpp",
            "test/test_ack_codec.cpp",
            "test/test_command_lifecycle.cpp",
            "test/test_write_queue.cpp",
            "test/test_gateway_readiness.cpp",
        ]
        missing = [item for item in required if not (PACKAGE / item).is_file()]
        self.assertEqual(missing, [])

    def test_hardware_package_declares_required_dependencies(self):
        root = ET.parse(str(PACKAGE / "package.xml")).getroot()
        self.assertEqual(root.findtext("name"), "cleanbot_hardware")
        dependencies = {item.text for item in root.findall("depend")}
        self.assertTrue({"rclcpp", "cleanbot_interfaces", "boost"}.issubset(dependencies))

        cmake = (PACKAGE / "CMakeLists.txt").read_text(encoding="utf-8")
        self.assertIn("CXX_STANDARD 17", cmake)
        self.assertIn("if(MSVC)", cmake)
        self.assertIn("/utf-8", cmake)
        self.assertIn("add_library(lower_machine_protocol", cmake)
        self.assertIn("add_executable(lower_machine_node", cmake)
        self.assertIn("ament_add_gtest(test_frame_buffer", cmake)
        self.assertIn("ament_add_gtest(test_status_frame_parser", cmake)
        self.assertIn("ament_add_gtest(test_command_encoder", cmake)
        self.assertIn("ament_add_gtest(test_ack_codec", cmake)
        self.assertIn("ament_add_gtest(test_command_lifecycle", cmake)
        self.assertIn("find_package(Boost REQUIRED COMPONENTS system)", cmake)

    def test_integer_configuration_uses_typed_config_snapshot(self):
        source = (PACKAGE / "src/lower_machine_node.cpp").read_text(encoding="utf-8")
        for key in (
            "hardware.lower_machine_baudrate",
            "hardware.command_ack_timeout_ms",
            "hardware.command_max_retries",
            "hardware.command_history_size",
        ):
            self.assertIn('snapshot.get_integer("{}")'.format(key), source)

    def test_lower_machine_node_is_only_protocol_adapter_not_mission_logic(self):
        source = (PACKAGE / "src" / "lower_machine_node.cpp").read_text(encoding="utf-8").lower()
        self.assertIn("hardware/status", source)
        self.assertIn("control/final_cmd", source)
        self.assertNotIn("autodrive", source)
        self.assertNotIn("waypoint", source)
        self.assertNotIn("mqtt", source)
        self.assertIn("lowermachineserial", source)
        self.assertIn("while", (PACKAGE / "src" / "lower_machine_serial.cpp").read_text(encoding="utf-8"))

    def test_protocol_uses_fixed_23_byte_frames_and_signed_speed(self):
        frame_header = (PACKAGE / "include" / "cleanbot_hardware" / "frame_buffer.hpp").read_text(
            encoding="utf-8"
        )
        parser_header = (
            PACKAGE / "include" / "cleanbot_hardware" / "status_frame_parser.hpp"
        ).read_text(encoding="utf-8")
        parser_source = (PACKAGE / "src" / "status_frame_parser.cpp").read_text(encoding="utf-8")

        self.assertIn("kStatusFrameLength = 23", frame_header)
        self.assertIn("kAckFrameLength = 8", frame_header)
        self.assertNotIn("short_frame_length", frame_header)
        self.assertNotIn("short_finish", parser_header)
        self.assertIn("frame.size() != kFrameLength", parser_source)
        self.assertIn("frame.back() != kFrameEnd", parser_source)
        self.assertIn("static_cast<std::int16_t>", parser_source)

    def test_protocol_implements_command_and_completion_acknowledgement(self):
        command_header = (PACKAGE / "include/cleanbot_hardware/command_encoder.hpp").read_text(
            encoding="utf-8"
        )
        ack_header = (PACKAGE / "include/cleanbot_hardware/ack_codec.hpp").read_text(
            encoding="utf-8"
        )
        node_source = (PACKAGE / "src/lower_machine_node.cpp").read_text(encoding="utf-8")

        self.assertIn("kCommandLength = 21", command_header)
        self.assertIn("kCommandAck", ack_header)
        self.assertIn("kCompletionAck", ack_header)
        self.assertIn("collect_due_retries", node_source)
        self.assertIn("encode_completion_ack", node_source)

    def test_node_publishes_correlated_command_execution_outcomes(self):
        source = (PACKAGE / "src/lower_machine_node.cpp").read_text(encoding="utf-8")
        serial_header = (PACKAGE / "include/cleanbot_hardware/lower_machine_serial.hpp").read_text(
            encoding="utf-8"
        )
        status_message = (
            WORKSPACE / "src/cleanbot_interfaces/msg/CommandExecutionStatus.msg"
        ).read_text(encoding="utf-8")

        for token in (
            "command_execution_status.hpp",
            '"/hardware/command_status"',
            "command->command_id",
            "command->request_id",
            "command->source",
            "command_correlations_",
            "STATE_SENT",
            "STATE_ACKNOWLEDGED",
            "STATE_REJECTED",
            "STATE_TIMED_OUT",
            "STATE_COMPLETED",
            "STATE_QUEUED",
            "STATE_TRANSPORT_LOST",
            "STATE_SUPERSEDED",
        ):
            self.assertIn(token, source)

        self.assertIn("status.request_id = request_id", source)
        self.assertIn("status.source = source", source)
        self.assertIn("SendEvent", serial_header)
        self.assertIn("SendCallback", serial_header)
        self.assertIn("STATE_QUEUED", status_message)
        self.assertIn("STATE_TRANSPORT_LOST", status_message)
        self.assertIn("STATE_SUPERSEDED", status_message)

    def test_disconnect_reports_each_interrupted_pending_command(self):
        lifecycle_header = (
            PACKAGE / "include/cleanbot_hardware/command_lifecycle.hpp"
        ).read_text(encoding="utf-8")
        node_source = (PACKAGE / "src/lower_machine_node.cpp").read_text(encoding="utf-8")

        self.assertIn("struct InterruptedCommand", lifecycle_header)
        self.assertIn("std::vector<InterruptedCommand> clear_pending()", lifecycle_header)
        self.assertIn("publishInterruptedCommands", node_source)
        self.assertIn("STATE_TRANSPORT_LOST", node_source)

    def test_gateway_uses_priority_delivery_and_reconnect_brake_handshake(self):
        source = (PACKAGE / "src/lower_machine_node.cpp").read_text(encoding="utf-8")
        serial = (PACKAGE / "src/lower_machine_serial.cpp").read_text(encoding="utf-8")

        for token in (
            "CommandDeliveryPolicy::kLatestOnly",
            "WritePriority::kContinuous",
            "WritePriority::kSafety",
            "WritePriority::kProtocolAck",
            "gateway_not_ready",
            "startReconnectBrakeHandshake",
            "observe_valid_status",
        ):
            self.assertIn(token, source)
        self.assertIn("write_queue_.enqueue", serial)
        self.assertIn("active_frame = write_queue_.front_handle()", serial)
        self.assertIn("[this, active_frame]", serial)


if __name__ == "__main__":
    unittest.main()
