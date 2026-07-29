import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


WORKSPACE = Path(__file__).resolve().parents[1]
PACKAGE = WORKSPACE / "src" / "cleanbot_rtk"


class RtkPackageContractTest(unittest.TestCase):
    def test_rtk_package_metadata_targets_humble_cpp17(self):
        package_xml = PACKAGE / "package.xml"
        cmake_file = PACKAGE / "CMakeLists.txt"

        self.assertTrue(package_xml.is_file())
        self.assertTrue(cmake_file.is_file())

        root = ET.parse(str(package_xml)).getroot()
        self.assertEqual(root.findtext("name"), "cleanbot_rtk")
        dependencies = {item.text for item in root.findall("depend")}
        self.assertTrue({"rclcpp", "cleanbot_interfaces", "boost"}.issubset(dependencies))

        cmake = cmake_file.read_text(encoding="utf-8")
        self.assertIn("CXX_STANDARD 17", cmake)
        self.assertIn("find_package(Boost REQUIRED COMPONENTS system)", cmake)
        self.assertIn("add_library(rtk_protocol", cmake)
        self.assertIn("add_executable(rtk_node", cmake)
        self.assertIn("src/rtk_validity.cpp", cmake)
        self.assertIn("ament_add_gtest(test_rtk_validity", cmake)

    def test_integer_configuration_uses_typed_config_snapshot(self):
        source = (PACKAGE / "src/rtk_node.cpp").read_text(encoding="utf-8")
        self.assertIn('snapshot.get_integer("rtk.baudrate")', source)
        self.assertIn('snapshot.get_integer("ntrip.port")', source)

    def test_rtk_validity_is_dependency_free_and_tested(self):
        required = (
            "include/cleanbot_rtk/rtk_validity.hpp",
            "src/rtk_validity.cpp",
            "test/test_rtk_validity.cpp",
        )
        missing = [item for item in required if not (PACKAGE / item).is_file()]
        self.assertEqual(missing, [])

    def test_rtcm3_is_validated_and_forwarded_as_complete_frames(self):
        required = (
            "include/cleanbot_rtk/rtcm3_frame_buffer.hpp",
            "src/rtcm3_frame_buffer.cpp",
            "test/test_rtcm3_frame_buffer.cpp",
        )
        missing = [item for item in required if not (PACKAGE / item).is_file()]
        self.assertEqual(missing, [])

        node = (PACKAGE / "src/rtk_node.cpp").read_text(encoding="utf-8")
        for token in ("Rtcm3FrameBuffer", "onRtcmBytes", "rtcm_frame_buffer_.pop"):
            self.assertIn(token, node)

    def test_rtk_fix_distinguishes_antenna_and_vehicle_center(self):
        message = (
            WORKSPACE / "src" / "cleanbot_interfaces" / "msg" / "RtkFix.msg"
        ).read_text(encoding="utf-8")
        required = (
            "float64 raw_lat",
            "float64 raw_lon",
            "bool center_valid",
            "float64 heading_age_sec",
            "float64 gga_age_sec",
            "float64 rtcm_age_sec",
            "bool ntrip_enabled",
            "bool ntrip_configured",
            "string ntrip_last_error",
        )
        for field in required:
            self.assertIn(field, message)

    def test_rtk_serial_is_the_only_serial_owner(self):
        required = (
            "include/cleanbot_rtk/rtk_serial.hpp",
            "src/rtk_serial.cpp",
        )
        for item in required:
            self.assertTrue((PACKAGE / item).is_file(), item)

        header = (PACKAGE / required[0]).read_text(encoding="utf-8")
        source = (PACKAGE / required[1]).read_text(encoding="utf-8")
        self.assertIn("boost::asio::serial_port", header)
        self.assertIn("async_read_some", source)
        self.assertIn("write_queue_", header)
        self.assertIn("std::shared_ptr<std::vector<std::uint8_t>>", header)
        for command in ("unlog\\r\\n", "gngga 0.1\\r\\n", "gphpr 0.1\\r\\n", "saveconfig\\r\\n"):
            self.assertIn(command, source)
        self.assertIn("save_config_on_connect_", header)
        self.assertIn("if (save_config_on_connect_)", source)
        self.assertIn("active_frame = write_queue_.front()", source)
        self.assertIn("[this, active_frame]", source)

        owners = []
        for path in PACKAGE.rglob("*"):
            if path.suffix not in {".hpp", ".cpp"} or not path.is_file():
                continue
            if "boost::asio::serial_port serial_port_" in path.read_text(encoding="utf-8"):
                owners.append(str(path.relative_to(PACKAGE)))
        self.assertEqual(owners, ["include\\cleanbot_rtk\\rtk_serial.hpp"])

    def test_ntrip_client_is_async_and_does_not_own_serial(self):
        header_path = PACKAGE / "include/cleanbot_rtk/ntrip_client.hpp"
        source_path = PACKAGE / "src/ntrip_client.cpp"
        self.assertTrue(header_path.is_file())
        self.assertTrue(source_path.is_file())

        header = header_path.read_text(encoding="utf-8")
        source = source_path.read_text(encoding="utf-8")
        self.assertIn("RtcmCallback", header)
        self.assertIn("async_resolve", source)
        self.assertIn("async_connect", source)
        self.assertIn("connect_timeout_sec", source)
        self.assertIn("reconnect_interval_sec", source)
        self.assertIn("rtcm_timeout_sec", source)
        self.assertIn("gga_interval_sec", source)
        self.assertNotIn("serial_port", header.lower())
        self.assertNotIn("serial_port", source.lower())

    def test_rtk_node_publishes_antenna_and_vehicle_center_without_motion_control(self):
        source_path = PACKAGE / "src/rtk_node.cpp"
        self.assertTrue(source_path.is_file())
        source = source_path.read_text(encoding="utf-8")

        required = (
            "NmeaLineBuffer",
            "NmeaParser",
            "RtkSampleSynchronizer",
            "VehicleCenterTransform",
            "RtkSerial",
            "NtripClient",
            "create_publisher<cleanbot_interfaces::msg::RtkFix>",
            '"/rtk/fix"',
            "message.raw_lat",
            "message.raw_lon",
            "message.center_valid",
            "center_transform_.compute",
            "rtk_serial_->enqueueWrite(frame)",
            "ntrip_client_->updateGga",
        )
        for text in required:
            self.assertIn(text, source)

        self.assertNotIn("/control/final_cmd", source)
        self.assertNotIn("sendBraking", source)
        self.assertNotIn("redis", source.lower())

    def test_rtk_node_reports_stream_freshness_when_serial_becomes_silent(self):
        source = (PACKAGE / "src/rtk_node.cpp").read_text(encoding="utf-8")

        for token in (
            "create_wall_timer",
            "publishFreshness",
            "last_gga_monotonic_sec_",
            "message.serial_connected",
            "message.coordinate_valid",
            "message.fixed_valid",
            "is_fixed_valid",
            "should_publish_freshness_heartbeat",
        ):
            self.assertIn(token, source)
        self.assertNotIn("utcSecondsNow", source)
        self.assertIn("synchronizer_->observe_gga(gga.data, received_at)", source)

    def test_receiver_persistence_is_explicit_and_disabled_by_default(self):
        node = (PACKAGE / "src/rtk_node.cpp").read_text(encoding="utf-8")
        registry = (WORKSPACE / "src/cleanbot_config/src/config_registry.cpp").read_text(
            encoding="utf-8"
        )

        self.assertIn('snapshot.get_boolean("rtk.save_config_on_connect")', node)
        self.assertIn('boolean_default("rtk.save_config_on_connect", "false")', registry)


if __name__ == "__main__":
    unittest.main()
