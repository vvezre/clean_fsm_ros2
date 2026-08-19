# 文件作用：验证 rtk protocol runtime 相关契约、运行逻辑和边界条件。
import ctypes
import os
import sysconfig
import unittest
from pathlib import Path


CPPYY_BACKEND_BIN = Path(sysconfig.get_paths()["purelib"]) / "cppyy_backend" / "bin"
if os.name == "nt" and CPPYY_BACKEND_BIN.is_dir():
    os.add_dll_directory(str(CPPYY_BACKEND_BIN))
    os.environ["PATH"] = str(CPPYY_BACKEND_BIN) + os.pathsep + os.environ.get("PATH", "")
    for dll_name in (
        "msvcp140.dll",
        "msvcp140_1.dll",
        "msvcp140_2.dll",
        "vcruntime140.dll",
        "vcruntime140_1.dll",
    ):
        ctypes.WinDLL(str(CPPYY_BACKEND_BIN / dll_name))

import cppyy


WORKSPACE = Path(__file__).resolve().parents[1]
PACKAGE = WORKSPACE / "src" / "cleanbot_rtk"

cppyy.add_include_path(str(PACKAGE / "include"))
cppyy.add_include_path(str(PACKAGE / "src"))
cppyy.cppdef(
    '#include "nmea_line_buffer.cpp"\n'
    '#include "nmea_parser.cpp"\n'
    '#include "rtk_sample_synchronizer.cpp"\n'
    '#include "rtk_validity.cpp"\n'
    '#include "vehicle_center_transform.cpp"\n'
    '#include "ntrip_protocol.cpp"\n'
)


# 辅助方法：为 byte_vector 测试场景准备输入、执行操作或整理结果。
def byte_vector(data):
    vector = cppyy.gbl.std.vector["unsigned char"]()
    for value in data:
        vector.push_back(value)
    return vector


# 辅助方法：为 nmea 测试场景准备输入、执行操作或整理结果。
def nmea(body):
    checksum = 0
    for value in body.encode("ascii"):
        checksum ^= value
    return "${}*{:02X}".format(body, checksum)


class RtkProtocolRuntimeTest(unittest.TestCase):
    # 测试作用：验证“line_buffer_reassembles_split_nmea”场景的契约、输出结果和边界行为。
    def test_line_buffer_reassembles_split_nmea(self):
        buffer = cppyy.gbl.cleanbot.rtk.NmeaLineBuffer(128)
        line = cppyy.gbl.std.string()

        buffer.append(byte_vector(b"$GNGGA,123"))
        self.assertFalse(buffer.pop(line))
        buffer.append(byte_vector(b"519,1,2*00\r\n"))

        self.assertTrue(buffer.pop(line))
        self.assertEqual(str(line), "$GNGGA,123519,1,2*00")

    # 测试作用：验证“line_buffer_extracts_sticky_lines_and_discards_noise”场景的契约、输出结果和边界行为。
    def test_line_buffer_extracts_sticky_lines_and_discards_noise(self):
        buffer = cppyy.gbl.cleanbot.rtk.NmeaLineBuffer(256)
        line = cppyy.gbl.std.string()
        buffer.append(byte_vector(
            b"noise\x00$GNGGA,1*00\n$GPHPR,2,90.0,0.0*00\r\n"
        ))

        self.assertTrue(buffer.pop(line))
        self.assertEqual(str(line), "$GNGGA,1*00")
        self.assertTrue(buffer.pop(line))
        self.assertEqual(str(line), "$GPHPR,2,90.0,0.0*00")
        self.assertFalse(buffer.pop(line))

    # 测试作用：验证“line_buffer_keeps_memory_bounded”场景的契约、输出结果和边界行为。
    def test_line_buffer_keeps_memory_bounded(self):
        buffer = cppyy.gbl.cleanbot.rtk.NmeaLineBuffer(32)
        buffer.append(byte_vector(b"x" * 100))
        self.assertLessEqual(buffer.size(), 32)

    # 测试作用：验证“parser_accepts_fixed_and_float_gga”场景的契约、输出结果和边界行为。
    def test_parser_accepts_fixed_and_float_gga(self):
        parser = cppyy.gbl.cleanbot.rtk.NmeaParser()
        fixed = parser.parse_gga(nmea(
            "GNGGA,123519.00,1220.74073400,N,09845.92592600,E,4,12,0.9,"
            "10.0,M,0.0,M,1.0,0001"
        ))
        floating = parser.parse_gga(nmea(
            "GPGGA,123519.20,1220.74073400,N,09845.92592600,E,5,08,1.5,"
            "10.0,M,0.0,M,,"
        ))

        self.assertTrue(fixed.parsed)
        self.assertAlmostEqual(fixed.data.lat, 12.34567890, places=8)
        self.assertAlmostEqual(fixed.data.lon, 98.76543210, places=8)
        self.assertEqual(fixed.data.fix_quality, 4)
        self.assertEqual(fixed.data.satellite_count, 12)
        self.assertAlmostEqual(fixed.data.hdop, 0.9)
        self.assertTrue(floating.parsed)
        self.assertEqual(floating.data.fix_quality, 5)

    # 测试作用：验证“parser_keeps_quality_zero_coordinates_invalid”场景的契约、输出结果和边界行为。
    def test_parser_keeps_quality_zero_coordinates_invalid(self):
        parser = cppyy.gbl.cleanbot.rtk.NmeaParser()
        invalid = parser.parse_gga(nmea(
            "GNGGA,123519.00,1220.74073400,N,09845.92592600,E,0,00,9.9,"
            "0.0,M,0.0,M,,"
        ))

        self.assertTrue(invalid.parsed)
        self.assertEqual(invalid.data.fix_quality, 0)
        self.assertFalse(invalid.data.position_valid)

    # 测试作用：验证“parser_parses_hpr_and_ths_heading”场景的契约、输出结果和边界行为。
    def test_parser_parses_hpr_and_ths_heading(self):
        parser = cppyy.gbl.cleanbot.rtk.NmeaParser()
        hpr = parser.parse_heading(nmea(
            "GNHPR,123519.10,90.00,00.00,000.00,4,12,0.00,0001"
        ))
        ths = parser.parse_heading(nmea("GNTHS,180.00,A"))

        self.assertTrue(hpr.parsed)
        self.assertAlmostEqual(hpr.data.utc_seconds, 12 * 3600 + 35 * 60 + 19.10)
        self.assertAlmostEqual(hpr.data.heading_deg, 90.0)
        self.assertAlmostEqual(hpr.data.pitch_deg, 0.0)
        self.assertTrue(ths.parsed)
        self.assertAlmostEqual(ths.data.heading_deg, 180.0)

    # 测试作用：验证“parser_rejects_bad_checksum_and_out_of_range_coordinate”场景的契约、输出结果和边界行为。
    def test_parser_rejects_bad_checksum_and_out_of_range_coordinate(self):
        parser = cppyy.gbl.cleanbot.rtk.NmeaParser()
        bad_checksum = parser.parse_gga(
            "$GNGGA,123519.00,1220.7,N,09845.9,E,4,07,0.5,0,M,0,M,,*00"
        )
        bad_latitude = parser.parse_gga(nmea(
            "GNGGA,123519.00,9900.0,N,09845.9,E,4,07,0.5,0,M,0,M,,"
        ))

        self.assertFalse(bad_checksum.parsed)
        self.assertEqual(bad_checksum.error, "nmea_checksum_invalid")
        self.assertFalse(bad_latitude.parsed)
        self.assertEqual(bad_latitude.error, "gga_coordinate_invalid")

    # 测试作用：验证“synchronizer_pairs_gga_and_hpr_by_gnss_utc”场景的契约、输出结果和边界行为。
    def test_synchronizer_pairs_gga_and_hpr_by_gnss_utc(self):
        parser = cppyy.gbl.cleanbot.rtk.NmeaParser()
        sync = cppyy.gbl.cleanbot.rtk.RtkSampleSynchronizer(0.5, 2.0, 0.5)
        heading = parser.parse_heading(nmea(
            "GNHPR,123519.10,90.00,00.00,000.00,4,12,0.00,0001"
        )).data
        gga = parser.parse_gga(nmea(
            "GNGGA,123519.20,1220.74073400,N,09845.92592600,E,4,12,0.9,"
            "10.0,M,0.0,M,,"
        )).data

        sync.observe_heading(heading, 100.0)
        result = sync.observe_gga(gga, 100.1)

        self.assertTrue(result.produced)
        self.assertTrue(result.sample.heading_valid)
        self.assertAlmostEqual(result.sample.heading_deg, 90.0)
        self.assertEqual(result.sample.gga.fix_quality, 4)

    # 测试作用：验证“synchronizer_expires_old_heading_without_using_host_utc”场景的契约、输出结果和边界行为。
    def test_synchronizer_expires_old_heading_without_using_host_utc(self):
        parser = cppyy.gbl.cleanbot.rtk.NmeaParser()
        sync = cppyy.gbl.cleanbot.rtk.RtkSampleSynchronizer(0.5, 2.0, 0.5)
        heading = parser.parse_heading(nmea(
            "GNHPR,123519.10,90.00,00.00,000.00,4,12,0.00,0001"
        )).data
        fresh_gga = parser.parse_gga(nmea(
            "GNGGA,123519.20,1220.74073400,N,09845.92592600,E,5,08,1.5,"
            "10.0,M,0.0,M,,"
        )).data
        stale_gga = parser.parse_gga(nmea(
            "GNGGA,123515.00,1220.74073400,N,09845.92592600,E,4,12,0.9,"
            "10.0,M,0.0,M,,"
        )).data

        sync.observe_heading(heading, 100.0)
        expired = sync.observe_gga(fresh_gga, 100.6)
        older_gnss_time = sync.observe_gga(stale_gga, 100.7)

        self.assertTrue(expired.produced)
        self.assertFalse(expired.sample.heading_valid)
        self.assertEqual(expired.sample.gga.fix_quality, 5)
        self.assertTrue(older_gnss_time.produced)
        self.assertFalse(older_gnss_time.stale)
        self.assertFalse(older_gnss_time.sample.heading_valid)

    # 测试作用：验证“synchronizer_handles_midnight_utc_wrap”场景的契约、输出结果和边界行为。
    def test_synchronizer_handles_midnight_utc_wrap(self):
        heading = cppyy.gbl.cleanbot.rtk.HeadingData()
        heading.utc_seconds = 86399.8
        heading.heading_deg = 5.0
        gga = cppyy.gbl.cleanbot.rtk.GgaData()
        gga.utc_seconds = 0.1
        gga.position_valid = True
        sync = cppyy.gbl.cleanbot.rtk.RtkSampleSynchronizer(0.5, 2.0, 0.5)

        sync.observe_heading(heading, 10.0)
        result = sync.observe_gga(gga, 10.1)

        self.assertTrue(result.produced)
        self.assertTrue(result.sample.heading_valid)

    # 测试作用：验证“fixed_validity_requires_connected_fresh_fixed_vehicle_center”场景的契约、输出结果和边界行为。
    def test_fixed_validity_requires_connected_fresh_fixed_vehicle_center(self):
        value = cppyy.gbl.cleanbot.rtk.RtkValidityInput()
        value.serial_connected = True
        value.coordinate_valid = True
        value.center_valid = True
        value.heading_valid = True
        value.fix_quality = 4
        value.gga_age_sec = 0.2
        value.max_gga_age_sec = 2.0

        self.assertTrue(cppyy.gbl.cleanbot.rtk.is_fixed_valid(value))

        invalid_cases = (
            ("serial_connected", False),
            ("coordinate_valid", False),
            ("center_valid", False),
            ("heading_valid", False),
            ("fix_quality", 5),
            ("gga_age_sec", 2.1),
        )
        for field, invalid in invalid_cases:
            original = getattr(value, field)
            setattr(value, field, invalid)
            self.assertFalse(
                cppyy.gbl.cleanbot.rtk.is_fixed_valid(value),
                field,
            )
            setattr(value, field, original)

    # 测试作用：验证“freshness_heartbeat_only_publishes_for_missing_disconnected_or_stale_data”场景的契约、输出结果和边界行为。
    def test_freshness_heartbeat_only_publishes_for_missing_disconnected_or_stale_data(self):
        should_publish = cppyy.gbl.cleanbot.rtk.should_publish_freshness_heartbeat

        self.assertTrue(should_publish(False, True, -1.0, 2.0))
        self.assertTrue(should_publish(True, False, 0.1, 2.0))
        self.assertFalse(should_publish(True, True, 0.1, 2.0))
        self.assertTrue(should_publish(True, True, 2.1, 2.0))

    # 测试作用：验证“vehicle_center_transform_matches_python_golden_points”场景的契约、输出结果和边界行为。
    def test_vehicle_center_transform_matches_python_golden_points(self):
        transform = cppyy.gbl.cleanbot.rtk.VehicleCenterTransform()
        expected = {
            0.0: (12.34567980, 98.76543376),
            90.0: (12.34567728, 98.76543302),
            180.0: (12.34567800, 98.76543044),
            270.0: (12.34568052, 98.76543118),
        }
        for heading, point in expected.items():
            result = transform.compute(
                12.34567890, 98.76543210, heading, 0.10, 0.18
            )
            self.assertTrue(result.valid)
            self.assertAlmostEqual(result.lat, point[0], places=8)
            self.assertAlmostEqual(result.lon, point[1], places=8)

    # 测试作用：验证“vehicle_center_transform_rejects_invalid_heading”场景的契约、输出结果和边界行为。
    def test_vehicle_center_transform_rejects_invalid_heading(self):
        transform = cppyy.gbl.cleanbot.rtk.VehicleCenterTransform()
        result = transform.compute(12.0, 98.0, float("nan"), 0.10, 0.18)
        self.assertFalse(result.valid)

    # 测试作用：验证“ntrip_request_contains_mountpoint_and_basic_auth”场景的契约、输出结果和边界行为。
    def test_ntrip_request_contains_mountpoint_and_basic_auth(self):
        config = cppyy.gbl.cleanbot.rtk.NtripConfig()
        config.host = "caster.example"
        config.port = 2101
        config.mountpoint = "RTCM33"
        config.username = "user"
        config.password = "pass"

        request = str(cppyy.gbl.cleanbot.rtk.build_ntrip_request(config))

        self.assertIn("GET /RTCM33 HTTP/1.0\r\n", request)
        self.assertIn("Authorization: Basic dXNlcjpwYXNz\r\n", request)
        self.assertNotIn("user:pass", request)

    # 测试作用：验证“ntrip_response_preserves_rtcm_after_header”场景的契约、输出结果和边界行为。
    def test_ntrip_response_preserves_rtcm_after_header(self):
        parser = cppyy.gbl.cleanbot.rtk.NtripResponseParser(1024)
        first = parser.append(byte_vector(b"ICY 200 OK\r\nServer: caster\r\n"))
        self.assertFalse(first.header_complete)

        second = parser.append(byte_vector(b"\r\n\xd3\x00\x13\x01\x02"))

        self.assertTrue(second.header_complete)
        self.assertTrue(second.accepted)
        self.assertEqual([ord(value) for value in second.rtcm_bytes], [0xD3, 0x00, 0x13, 0x01, 0x02])

    # 测试作用：验证“ntrip_response_rejects_non_200_and_oversized_header”场景的契约、输出结果和边界行为。
    def test_ntrip_response_rejects_non_200_and_oversized_header(self):
        rejected = cppyy.gbl.cleanbot.rtk.NtripResponseParser(1024).append(
            byte_vector(b"HTTP/1.0 401 Unauthorized\r\n\r\n")
        )
        oversized_parser = cppyy.gbl.cleanbot.rtk.NtripResponseParser(16)
        oversized = oversized_parser.append(byte_vector(b"HTTP/1.0 200 OK without terminator"))

        self.assertTrue(rejected.header_complete)
        self.assertFalse(rejected.accepted)
        self.assertEqual(rejected.error, "ntrip_response_rejected")
        self.assertEqual(oversized.error, "ntrip_header_too_large")


if __name__ == "__main__":
    unittest.main()
