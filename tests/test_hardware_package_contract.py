# 文件作用：验证 hardware package contract 相关契约、运行逻辑和边界条件。
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


WORKSPACE = Path(__file__).resolve().parents[1]
PACKAGE = WORKSPACE / "src" / "cleanbot_hardware"


class HardwarePackageContractTest(unittest.TestCase):
    # 测试作用：验证“hardware_package_contains_protocol_library_node_and_tests”场景的契约、输出结果和边界行为。
    def test_hardware_package_contains_protocol_library_node_and_tests(self):
        required = [
            "package.xml",
            "CMakeLists.txt",
            "include/cleanbot_hardware/frame_buffer.hpp",
            "include/cleanbot_hardware/status_frame_parser.hpp",
            "include/cleanbot_hardware/command_encoder.hpp",
            "include/cleanbot_hardware/lower_machine_serial.hpp",
            "include/cleanbot_hardware/write_queue.hpp",
            "src/frame_buffer.cpp",
            "src/status_frame_parser.cpp",
            "src/command_encoder.cpp",
            "src/lower_machine_serial.cpp",
            "src/write_queue.cpp",
            "src/lower_machine_node.cpp",
            "test/test_frame_buffer.cpp",
            "test/test_status_frame_parser.cpp",
            "test/test_write_queue.cpp",
        ]
        missing = [item for item in required if not (PACKAGE / item).is_file()]
        self.assertEqual(missing, [])

    # 测试作用：验证“hardware_package_declares_required_dependencies”场景的契约、输出结果和边界行为。
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
        self.assertIn("find_package(Boost REQUIRED COMPONENTS system)", cmake)

    # 测试作用：验证“integer_configuration_uses_typed_config_snapshot”场景的契约、输出结果和边界行为。
    def test_integer_configuration_uses_typed_config_snapshot(self):
        source = (PACKAGE / "src/lower_machine_node.cpp").read_text(encoding="utf-8")
        for key in (
            "hardware.lower_machine_baudrate",
            "hardware.command_repeat_count",
        ):
            self.assertIn('snapshot.get_integer("{}")'.format(key), source)

    # 测试作用：验证“lower_machine_node_is_only_protocol_adapter_not_mission_logic”场景的契约、输出结果和边界行为。
    def test_lower_machine_node_is_only_protocol_adapter_not_mission_logic(self):
        source = (PACKAGE / "src" / "lower_machine_node.cpp").read_text(encoding="utf-8").lower()
        self.assertIn("hardware/status", source)
        self.assertIn("control/final_cmd", source)
        self.assertNotIn("autodrive", source)
        self.assertNotIn("waypoint", source)
        self.assertNotIn("mqtt", source)
        self.assertIn("lowermachineserial", source)
        self.assertIn("while", (PACKAGE / "src" / "lower_machine_serial.cpp").read_text(encoding="utf-8"))

    # 测试作用：验证“protocol_uses_fixed_23_byte_frames_and_signed_speed”场景的契约、输出结果和边界行为。
    def test_protocol_uses_fixed_23_byte_frames_and_signed_speed(self):
        frame_header = (PACKAGE / "include" / "cleanbot_hardware" / "frame_buffer.hpp").read_text(
            encoding="utf-8"
        )
        parser_header = (
            PACKAGE / "include" / "cleanbot_hardware" / "status_frame_parser.hpp"
        ).read_text(encoding="utf-8")
        parser_source = (PACKAGE / "src" / "status_frame_parser.cpp").read_text(encoding="utf-8")

        self.assertIn("kStatusFrameLength = 23", frame_header)
        self.assertNotIn("kAckFrameLength", frame_header)
        self.assertNotIn("short_frame_length", frame_header)
        self.assertNotIn("short_finish", parser_header)
        self.assertIn("frame.size() != kFrameLength", parser_source)
        self.assertIn("frame.back() != kFrameEnd", parser_source)
        self.assertIn("static_cast<std::int16_t>", parser_source)

    # 测试作用：验证“protocol_uses_legacy_command_layout_without_wire_ack”场景的契约、输出结果和边界行为。
    def test_protocol_uses_legacy_command_layout_without_wire_ack(self):
        command_header = (PACKAGE / "include/cleanbot_hardware/command_encoder.hpp").read_text(
            encoding="utf-8"
        )
        node_source = (PACKAGE / "src/lower_machine_node.cpp").read_text(encoding="utf-8")

        self.assertIn("kCommandLength = 19", command_header)
        self.assertIn("encodeInitHeading", command_header)
        self.assertIn("encodeLegacyControl", command_header)
        self.assertIn("repeat_count_", node_source)
        self.assertNotIn("AckCodec", node_source)
        self.assertNotIn("encode_completion_ack", node_source)

    # 测试作用：验证“node_publishes_correlated_command_execution_outcomes”场景的契约、输出结果和边界行为。
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
            "STATE_SENT",
            "STATE_REJECTED",
            "STATE_COMPLETED",
            "STATE_QUEUED",
            "STATE_TRANSPORT_LOST",
        ):
            self.assertIn(token, source)

        self.assertIn("status.request_id = request_id", source)
        self.assertIn("status.source = source", source)
        self.assertIn("SendEvent", serial_header)
        self.assertIn("SendCallback", serial_header)
        self.assertIn("STATE_QUEUED", status_message)
        self.assertIn("STATE_TRANSPORT_LOST", status_message)
        self.assertIn("STATE_SUPERSEDED", status_message)

    # 测试作用：验证“disconnect_reports_transport_loss_for_active_command”场景的契约、输出结果和边界行为。
    def test_disconnect_reports_transport_loss_for_active_command(self):
        node_source = (PACKAGE / "src/lower_machine_node.cpp").read_text(encoding="utf-8")

        self.assertIn("STATE_TRANSPORT_LOST", node_source)

    # 测试作用：验证“reconnect_sends_legacy_stop_burst”场景的契约、输出结果和边界行为。
    def test_reconnect_sends_legacy_stop_burst(self):
        source = (PACKAGE / "src/lower_machine_node.cpp").read_text(encoding="utf-8")
        serial = (PACKAGE / "src/lower_machine_serial.cpp").read_text(encoding="utf-8")

        for token in (
            "WritePriority::kContinuous",
            "WritePriority::kSafety",
            "reconnect-stop",
            "repeat_count_",
        ):
            self.assertIn(token, source)
        self.assertIn("write_queue_.enqueue", serial)
        self.assertIn("active_frame = write_queue_.front_handle()", serial)
        self.assertIn("[this, active_frame]", serial)


if __name__ == "__main__":
    unittest.main()
