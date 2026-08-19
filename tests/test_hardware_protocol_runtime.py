# 文件作用：验证 hardware protocol runtime 相关契约、运行逻辑和边界条件。
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
PACKAGE = WORKSPACE / "src" / "cleanbot_hardware"

cppyy.add_include_path(str(PACKAGE / "include"))
cppyy.add_include_path(str(PACKAGE / "src"))
cppyy.cppdef(
    '#include "frame_buffer.cpp"\n'
    '#include "status_frame_parser.cpp"\n'
    '#include "command_encoder.cpp"\n'
)


# 辅助方法：为 byte_vector 测试场景准备输入、执行操作或整理结果。
def byte_vector(values):
    vector = cppyy.gbl.std.vector["unsigned char"]()
    for value in values:
        vector.push_back(value)
    return vector


class HardwareProtocolRuntimeTest(unittest.TestCase):
    # 测试作用：验证“command_encoder_keeps_the_legacy_19_byte_wire_frame”场景的契约、输出结果和边界行为。
    def test_command_encoder_keeps_the_legacy_19_byte_wire_frame(self):
        fields = cppyy.gbl.cleanbot.hardware.CommandFields()
        fields.status = 1
        fields.power_on = 1
        fields.x_speed = 300
        fields.z_speed = -100
        fields.brush_speed = 50
        fields.target_distance = 90
        fields.target_rotation = 1800
        fields.heading_deg = 359.99
        fields.sequence = 0x1234

        frame = cppyy.gbl.cleanbot.hardware.CommandEncoder().encode(fields)

        self.assertEqual(
            [ord(value) for value in frame],
            [
                0x7B, 0x01, 0x01, 0x00, 0x01, 0x2C, 0xFF, 0x9C,
                0x00, 0x5A, 0x32, 0x00, 0x00, 0x07, 0x08, 0x8C,
                0x9F, 0x41, 0x7D,
            ],
        )

    # 测试作用：验证“command_encoder_brake_clears_all_motion_outputs”场景的契约、输出结果和边界行为。
    def test_command_encoder_brake_clears_all_motion_outputs(self):
        fields = cppyy.gbl.cleanbot.hardware.CommandFields()
        fields.status = 3
        fields.x_speed = 500
        fields.z_speed = -500
        fields.brush_speed = 80
        fields.target_distance = 1000
        fields.target_rotation = 900
        fields.brake = True

        frame = cppyy.gbl.cleanbot.hardware.CommandEncoder().encode(fields)

        self.assertEqual(ord(frame[1]), 0)
        self.assertEqual([ord(frame[index]) for index in range(4, 15)], [0] * 11)
        checksum = 0
        for value in frame[:17]:
            checksum ^= ord(value)
        self.assertEqual(ord(frame[17]), checksum)
        self.assertEqual(ord(frame[18]), 0x7D)

    # 测试作用：验证“command_encoder_uses_signed_twos_complement_for_negative_rotation”场景的契约、输出结果和边界行为。
    def test_command_encoder_uses_signed_twos_complement_for_negative_rotation(self):
        fields = cppyy.gbl.cleanbot.hardware.CommandFields()
        fields.target_rotation = -900

        frame = cppyy.gbl.cleanbot.hardware.CommandEncoder().encode(fields)

        self.assertEqual([ord(frame[13]), ord(frame[14])], [0xFC, 0x7C])

    # 测试作用：验证“command_encoder_uses_signed_twos_complement_for_manual_steering”场景的契约、输出结果和边界行为。
    def test_command_encoder_uses_signed_twos_complement_for_manual_steering(self):
        fields = cppyy.gbl.cleanbot.hardware.CommandFields()
        fields.steering_offset = -1000

        frame = cppyy.gbl.cleanbot.hardware.CommandEncoder().encode(fields)

        self.assertEqual([ord(frame[11]), ord(frame[12])], [0xFC, 0x18])

        fields.brake = True
        stopped = cppyy.gbl.cleanbot.hardware.CommandEncoder().encode(fields)
        self.assertEqual([ord(stopped[11]), ord(stopped[12])], [0x00, 0x00])

    # 测试作用：验证“command_encoder_preserves_python_init_heading_special_layout”场景的契约、输出结果和边界行为。
    def test_command_encoder_preserves_python_init_heading_special_layout(self):
        frame = cppyy.gbl.cleanbot.hardware.CommandEncoder().encodeInitHeading(90.0)

        self.assertEqual(len(frame), 19)
        self.assertEqual(ord(frame[0]), 0x7B)
        self.assertEqual([ord(frame[17]), ord(frame[18])], [0x23, 0x28])

    # 测试作用：验证“command_encoder_preserves_python_legacy_control_layout”场景的契约、输出结果和边界行为。
    def test_command_encoder_preserves_python_legacy_control_layout(self):
        frame = cppyy.gbl.cleanbot.hardware.CommandEncoder().encodeLegacyControl(0xE0)

        self.assertEqual(len(frame), 19)
        self.assertEqual([ord(frame[0]), ord(frame[1]), ord(frame[2])], [0x7B, 0xE0, 0x01])
        self.assertEqual([ord(frame[17]), ord(frame[18])], [0x00, 0x7D])

    # 测试作用：验证“command_encoder_builds_one_write_with_five_identical_frames”场景的契约、输出结果和边界行为。
    def test_command_encoder_builds_one_write_with_five_identical_frames(self):
        encoder = cppyy.gbl.cleanbot.hardware.CommandEncoder()
        fields = cppyy.gbl.cleanbot.hardware.CommandFields()
        fields.status = 1
        frame = encoder.encode(fields)

        burst = encoder.repeat(frame, 5)

        frame_values = [ord(value) for value in frame]
        self.assertEqual(len(burst), 95)
        self.assertEqual(
            [ord(value) for value in burst],
            frame_values * 5,
        )

    # 测试作用：验证“fixed_frame_waits_for_all_23_bytes”场景的契约、输出结果和边界行为。
    def test_fixed_frame_waits_for_all_23_bytes(self):
        buffer = cppyy.gbl.cleanbot.hardware.FrameBuffer(512)
        frame = byte_vector([])

        buffer.append(byte_vector([
            0x7B, 0x01, 0x01, 0x01, 0x01, 0x2C, 0x00,
            0x64, 0x32, 0x00, 0x64, 0x01, 0xBB, 0x00,
        ]))
        self.assertFalse(buffer.pop(frame))
        self.assertEqual(buffer.size(), 14)

        buffer.append(byte_vector([
            0x14, 0x1E, 0x00, 0x5A, 0x04, 0xD2, 0x12, 0x34, 0x7D,
        ]))
        self.assertTrue(buffer.pop(frame))
        self.assertEqual(len(frame), 23)

    # 测试作用：验证“buffer_ignores_ack_like_bytes_and_extracts_only_legacy_status”场景的契约、输出结果和边界行为。
    def test_buffer_ignores_ack_like_bytes_and_extracts_only_legacy_status(self):
        buffer = cppyy.gbl.cleanbot.hardware.FrameBuffer(512)
        frame = byte_vector([])
        ack = [0x7C, 0xA1, 0x00, 0x19, 0x03, 0x00]
        checksum = 0
        for value in ack:
            checksum ^= value
        ack.extend([checksum, 0x7E])
        status = [0x7B] + [0x00] * 21 + [0x7D]

        buffer.append(byte_vector([0x55] + ack + status))

        self.assertTrue(buffer.pop(frame))
        self.assertEqual([ord(value) for value in frame], status)
        self.assertFalse(buffer.pop(frame))

    # 测试作用：验证“payload_end_byte_does_not_terminate_frame_early”场景的契约、输出结果和边界行为。
    def test_payload_end_byte_does_not_terminate_frame_early(self):
        buffer = cppyy.gbl.cleanbot.hardware.FrameBuffer(512)
        frame = byte_vector([])
        buffer.append(byte_vector([
            0x7B, 0x01, 0x01, 0x01, 0x00, 0x7D, 0x00, 0x01,
            0x20, 0x00, 0x64, 0x01, 0x00, 0x00, 0x14, 0x1E,
            0x00, 0x5A, 0x00, 0x01, 0x00, 0x00, 0x7D,
        ]))

        self.assertTrue(buffer.pop(frame))
        self.assertEqual(len(frame), 23)
        self.assertEqual(ord(frame[5]), 0x7D)

    # 测试作用：验证“parser_decodes_signed_speed_without_fake_command_sequence”场景的契约、输出结果和边界行为。
    def test_parser_decodes_signed_speed_without_fake_command_sequence(self):
        parser = cppyy.gbl.cleanbot.hardware.StatusFrameParser()
        frame = byte_vector([
            0x7B, 0x01, 0x01, 0x02, 0x01, 0x2C, 0xFF, 0x9C,
            0x32, 0x00, 0x64, 0x01, 0xBB, 0x00, 0x14, 0x1E,
            0x00, 0x5A, 0x04, 0xD2, 0x12, 0x34, 0x7D,
        ])

        result = parser.parse(frame)

        self.assertTrue(result.parsed)
        self.assertEqual(result.status.x_speed, 300)
        self.assertEqual(result.status.z_speed, -100)
        self.assertEqual(result.status.z_speed_raw, 0xFF9C)
        self.assertEqual(result.status.command_sequence, 0)

    # 测试作用：验证“parser_rejects_short_frame_and_invalid_fixed_tail”场景的契约、输出结果和边界行为。
    def test_parser_rejects_short_frame_and_invalid_fixed_tail(self):
        parser = cppyy.gbl.cleanbot.hardware.StatusFrameParser()
        short_result = parser.parse(byte_vector([0x7B] + [0x00] * 13))
        bad_tail_result = parser.parse(byte_vector([0x7B] + [0x00] * 21 + [0x00]))

        self.assertFalse(short_result.parsed)
        self.assertEqual(short_result.error, "frame_length_invalid")
        self.assertFalse(bad_tail_result.parsed)
        self.assertEqual(bad_tail_result.error, "frame_end_missing")


if __name__ == "__main__":
    unittest.main()
