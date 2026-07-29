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
cppyy.cppdef('#include "rtcm3_frame_buffer.cpp"\n')


def byte_vector(data):
    result = cppyy.gbl.std.vector["unsigned char"]()
    for value in data:
        result.push_back(value)
    return result


def as_bytes(values):
    return bytes(ord(value) for value in values)


def crc24q(data):
    crc = 0
    for value in data:
        crc ^= value << 16
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= 0x1864CFB
    return crc & 0xFFFFFF


def rtcm_frame(payload):
    header = bytes([0xD3, (len(payload) >> 8) & 0x03, len(payload) & 0xFF])
    body = header + bytes(payload)
    crc = crc24q(body)
    return body + bytes([(crc >> 16) & 0xFF, (crc >> 8) & 0xFF, crc & 0xFF])


class Rtcm3FrameBufferRuntimeTest(unittest.TestCase):
    def test_reassembles_split_frame_and_extracts_sticky_frames(self):
        buffer = cppyy.gbl.cleanbot.rtk.Rtcm3FrameBuffer(4096)
        first = rtcm_frame([0x3E, 0xD0, 0x01, 0x02])
        second = rtcm_frame([0x43, 0x50, 0x03])
        frame = byte_vector([])

        buffer.append(byte_vector(b"noise" + first[:4]))
        self.assertFalse(buffer.pop(frame))
        buffer.append(byte_vector(first[4:] + second))

        self.assertTrue(buffer.pop(frame))
        self.assertEqual(as_bytes(frame), first)
        self.assertTrue(buffer.pop(frame))
        self.assertEqual(as_bytes(frame), second)

    def test_discards_bad_crc_and_resynchronizes_to_next_frame(self):
        buffer = cppyy.gbl.cleanbot.rtk.Rtcm3FrameBuffer(4096)
        bad = bytearray(rtcm_frame([1, 2, 3]))
        bad[-1] ^= 0xFF
        good = rtcm_frame([4, 5, 6])
        frame = byte_vector([])

        buffer.append(byte_vector(bytes(bad) + good))

        self.assertTrue(buffer.pop(frame))
        self.assertEqual(as_bytes(frame), good)
        self.assertEqual(buffer.invalid_crc_count(), 1)

    def test_buffer_remains_bounded_under_noise(self):
        buffer = cppyy.gbl.cleanbot.rtk.Rtcm3FrameBuffer(64)
        buffer.append(byte_vector(b"x" * 1000))
        self.assertLessEqual(buffer.size(), 64)


if __name__ == "__main__":
    unittest.main()
