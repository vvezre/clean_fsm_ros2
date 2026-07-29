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
    '#include "ack_codec.cpp"\n'
    '#include "command_lifecycle.cpp"\n'
)


def byte_vector(values):
    vector = cppyy.gbl.std.vector["unsigned char"]()
    for value in values:
        vector.push_back(value)
    return vector


class HardwareProtocolRuntimeTest(unittest.TestCase):
    def test_command_encoder_adds_sequence_to_legacy_fields(self):
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
                0x9F, 0x12, 0x34, 0x67, 0x7D,
            ],
        )

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
        for value in frame[:19]:
            checksum ^= ord(value)
        self.assertEqual(ord(frame[19]), checksum)
        self.assertEqual(ord(frame[20]), 0x7D)

    def test_command_encoder_uses_signed_twos_complement_for_negative_rotation(self):
        fields = cppyy.gbl.cleanbot.hardware.CommandFields()
        fields.target_rotation = -900

        frame = cppyy.gbl.cleanbot.hardware.CommandEncoder().encode(fields)

        self.assertEqual([ord(frame[13]), ord(frame[14])], [0xFC, 0x7C])

    def test_command_encoder_uses_signed_twos_complement_for_manual_steering(self):
        fields = cppyy.gbl.cleanbot.hardware.CommandFields()
        fields.steering_offset = -1000

        frame = cppyy.gbl.cleanbot.hardware.CommandEncoder().encode(fields)

        self.assertEqual([ord(frame[11]), ord(frame[12])], [0xFC, 0x18])

        fields.brake = True
        stopped = cppyy.gbl.cleanbot.hardware.CommandEncoder().encode(fields)
        self.assertEqual([ord(stopped[11]), ord(stopped[12])], [0x00, 0x00])

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

    def test_buffer_extracts_ack_and_status_frames_from_one_read(self):
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
        self.assertEqual([ord(value) for value in frame], ack)
        self.assertTrue(buffer.pop(frame))
        self.assertEqual([ord(value) for value in frame], status)

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

    def test_parser_decodes_signed_speed_and_command_sequence(self):
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
        self.assertEqual(result.status.command_sequence, 0x1234)

    def test_ack_codec_parses_a1_and_encodes_a2(self):
        codec = cppyy.gbl.cleanbot.hardware.AckCodec()
        a1 = [0x7C, 0xA1, 0x00, 0x19, 0x03, 0x00]
        checksum = 0
        for value in a1:
            checksum ^= value
        a1.extend([checksum, 0x7E])

        parsed = codec.parse(byte_vector(a1))
        self.assertTrue(parsed.parsed)
        self.assertEqual(parsed.ack.sequence, 25)
        self.assertEqual(parsed.ack.subject_type, 3)
        self.assertEqual(parsed.ack.result, 0)

        a2 = codec.encode_completion_ack(25, 2)
        self.assertEqual(
            [ord(value) for value in a2],
            [0x7C, 0xA2, 0x00, 0x19, 0x02, 0x00, 0xC5, 0x7E],
        )

        a1[6] = 0x00
        invalid = codec.parse(byte_vector(a1))
        self.assertFalse(invalid.parsed)
        self.assertEqual(invalid.error, "ack_checksum_invalid")

    def test_lifecycle_retries_same_frame_and_deduplicates_completion(self):
        lifecycle = cppyy.gbl.cleanbot.hardware.CommandLifecycle(200, 3, 64)
        command = byte_vector([0x7B] + [0x00] * 19 + [0x7D])
        lifecycle.track(25, 3, command, 1000)

        first_retry = lifecycle.collect_due_retries(1200)
        self.assertEqual(len(first_retry.frames), 1)
        self.assertEqual(
            [ord(value) for value in first_retry.frames[0]],
            [ord(value) for value in command],
        )

        ack = cppyy.gbl.cleanbot.hardware.AckFrame()
        ack.frame_type = 0xA1
        ack.sequence = 25
        ack.subject_type = 3
        ack.result = 0
        self.assertEqual(int(lifecycle.handle_command_ack(ack)), 1)
        self.assertEqual(len(lifecycle.collect_due_retries(1400).frames), 0)

        self.assertEqual(int(lifecycle.observe_completion(25, 2)), 1)
        self.assertEqual(int(lifecycle.observe_completion(25, 2)), 2)

    def test_lifecycle_times_out_after_three_retries(self):
        lifecycle = cppyy.gbl.cleanbot.hardware.CommandLifecycle(200, 3, 64)
        command = byte_vector([0x7B] + [0x00] * 19 + [0x7D])
        lifecycle.track(7, 1, command, 0)

        self.assertEqual(len(lifecycle.collect_due_retries(200).frames), 1)
        self.assertEqual(len(lifecycle.collect_due_retries(400).frames), 1)
        self.assertEqual(len(lifecycle.collect_due_retries(600).frames), 1)
        timed_out = lifecycle.collect_due_retries(800)
        self.assertEqual(len(timed_out.frames), 0)
        self.assertEqual([int(value) for value in timed_out.timed_out_sequences], [7])

    def test_new_continuous_command_supersedes_old_retry(self):
        lifecycle = cppyy.gbl.cleanbot.hardware.CommandLifecycle(200, 3, 64)
        first = byte_vector([0x7B, 0x01, 0x01, 0x7D])
        second = byte_vector([0x7B, 0x01, 0x02, 0x7D])
        latest_only = cppyy.gbl.cleanbot.hardware.CommandDeliveryPolicy.kLatestOnly

        lifecycle.track(10, 1, first, 0, latest_only)
        lifecycle.track(11, 1, second, 100, latest_only)

        retry = lifecycle.collect_due_retries(300)
        self.assertEqual(len(retry.frames), 1)
        self.assertEqual([ord(value) for value in retry.frames[0]], [0x7B, 0x01, 0x02, 0x7D])

        old_ack = cppyy.gbl.cleanbot.hardware.AckFrame()
        old_ack.frame_type = 0xA1
        old_ack.sequence = 10
        old_ack.subject_type = 1
        old_ack.result = 0
        self.assertEqual(int(lifecycle.handle_command_ack(old_ack)), 0)

    def test_timeout_keeps_original_command_type(self):
        lifecycle = cppyy.gbl.cleanbot.hardware.CommandLifecycle(100, 0, 64)
        lifecycle.track(9, 3, byte_vector([0x7B, 0x03, 0x7D]), 0)

        timed_out = lifecycle.collect_due_retries(100)

        self.assertEqual(len(timed_out.timed_out_commands), 1)
        self.assertEqual(int(timed_out.timed_out_commands[0].sequence), 9)
        self.assertEqual(int(timed_out.timed_out_commands[0].command_type), 3)

    def test_disconnect_clears_pending_retries(self):
        lifecycle = cppyy.gbl.cleanbot.hardware.CommandLifecycle(100, 3, 64)
        lifecycle.track(15, 2, byte_vector([0x7B, 0x02, 0x7D]), 0)

        interrupted = lifecycle.clear_pending()

        self.assertIsNotNone(interrupted)
        self.assertEqual(len(interrupted), 1)
        self.assertEqual(int(interrupted[0].sequence), 15)
        self.assertEqual(int(interrupted[0].command_type), 2)
        self.assertEqual(len(lifecycle.collect_due_retries(100).frames), 0)

    def test_superseded_command_can_be_cancelled_before_retry(self):
        lifecycle = cppyy.gbl.cleanbot.hardware.CommandLifecycle(100, 3, 64)
        lifecycle.track(16, 1, byte_vector([0x7B, 0x01, 0x7D]), 0)

        self.assertTrue(lifecycle.cancel_pending(16))
        self.assertFalse(lifecycle.cancel_pending(16))
        self.assertEqual(len(lifecycle.collect_due_retries(100).frames), 0)

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
