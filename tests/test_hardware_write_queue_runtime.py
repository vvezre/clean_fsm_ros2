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
cppyy.cppdef('#include "write_queue.cpp"\n')


def byte_vector(values):
    result = cppyy.gbl.std.vector["unsigned char"]()
    for value in values:
        result.push_back(value)
    return result


def as_list(values):
    return [ord(value) for value in values]


def enum_value(value):
    try:
        return int(value)
    except ValueError:
        return ord(value)


class HardwareWriteQueueRuntimeTest(unittest.TestCase):
    def test_continuous_writes_coalesce_to_latest_frame(self):
        queue = cppyy.gbl.cleanbot.hardware.PriorityWriteQueue(8)
        priority = cppyy.gbl.cleanbot.hardware.WritePriority.kContinuous

        self.assertTrue(queue.enqueue(byte_vector([1]), priority, "motion", False))
        self.assertTrue(queue.enqueue(byte_vector([2]), priority, "motion", False))

        self.assertEqual(queue.size(), 1)
        self.assertEqual(as_list(queue.front()), [2])

    def test_ack_and_safety_run_before_waiting_motion(self):
        queue = cppyy.gbl.cleanbot.hardware.PriorityWriteQueue(8)
        priority = cppyy.gbl.cleanbot.hardware.WritePriority

        queue.enqueue(byte_vector([1]), priority.kContinuous, "motion", False)
        queue.enqueue(byte_vector([2]), priority.kFinite, "", True)
        queue.enqueue(byte_vector([3]), priority.kSafety, "brake", True)
        queue.enqueue(byte_vector([4]), priority.kProtocolAck, "ack-4", True)

        self.assertEqual(as_list(queue.front()), [1])
        queue.pop_front()
        self.assertEqual(as_list(queue.front()), [4])
        queue.pop_front()
        self.assertEqual(as_list(queue.front()), [3])
        queue.pop_front()
        self.assertEqual(as_list(queue.front()), [2])

    def test_full_queue_drops_low_priority_not_ack_or_safety(self):
        queue = cppyy.gbl.cleanbot.hardware.PriorityWriteQueue(3)
        priority = cppyy.gbl.cleanbot.hardware.WritePriority

        queue.enqueue(byte_vector([1]), priority.kContinuous, "", False)
        queue.enqueue(byte_vector([2]), priority.kFinite, "", False)
        queue.enqueue(byte_vector([3]), priority.kSafety, "brake", False)
        self.assertTrue(queue.enqueue(byte_vector([4]), priority.kProtocolAck, "ack-4", False))

        frames = []
        while not queue.empty():
            frames.append(as_list(queue.front())[0])
            queue.pop_front()
        self.assertEqual(frames, [4, 3, 2])

    def test_full_high_priority_queue_rejects_without_growing(self):
        queue = cppyy.gbl.cleanbot.hardware.PriorityWriteQueue(2)
        priority = cppyy.gbl.cleanbot.hardware.WritePriority

        self.assertTrue(queue.enqueue(byte_vector([1]), priority.kSafety, "brake-1", False))
        self.assertTrue(queue.enqueue(byte_vector([2]), priority.kProtocolAck, "ack-2", False))

        self.assertFalse(
            queue.enqueue(byte_vector([3]), priority.kProtocolAck, "ack-3", False)
        )
        self.assertEqual(queue.size(), 2)

    def test_low_priority_frame_cannot_evict_accepted_finite_command(self):
        queue = cppyy.gbl.cleanbot.hardware.PriorityWriteQueue(2)
        priority = cppyy.gbl.cleanbot.hardware.WritePriority

        self.assertTrue(queue.enqueue(byte_vector([1]), priority.kFinite, "", False))
        self.assertTrue(queue.enqueue(byte_vector([2]), priority.kFinite, "", False))

        self.assertFalse(
            queue.enqueue(byte_vector([3]), priority.kContinuous, "", False)
        )
        self.assertEqual(queue.size(), 2)
        self.assertEqual(as_list(queue.front()), [1])

    def test_replaced_queue_item_receives_superseded_event(self):
        queue = cppyy.gbl.cleanbot.hardware.PriorityWriteQueue(2)
        priority = cppyy.gbl.cleanbot.hardware.WritePriority
        events = []

        def record(event):
            events.append(enum_value(event))

        self.assertTrue(
            queue.enqueue(byte_vector([1]), priority.kContinuous, "motion", False, record)
        )
        self.assertTrue(
            queue.enqueue(byte_vector([2]), priority.kContinuous, "motion", False)
        )

        self.assertEqual(events, [1])
        self.assertEqual(queue.size(), 1)
        self.assertEqual(as_list(queue.front()), [2])

    def test_unsent_brake_is_replaced_without_touching_active_frame(self):
        queue = cppyy.gbl.cleanbot.hardware.PriorityWriteQueue(3)
        priority = cppyy.gbl.cleanbot.hardware.WritePriority

        queue.enqueue(byte_vector([1]), priority.kContinuous, "motion", False)
        queue.enqueue(byte_vector([2]), priority.kSafety, "brake", True)
        queue.enqueue(byte_vector([3]), priority.kSafety, "brake", True)

        self.assertEqual(queue.size(), 2)
        self.assertEqual(as_list(queue.front()), [1])
        queue.pop_front()
        self.assertEqual(as_list(queue.front()), [3])

    def test_active_frame_storage_remains_stable_when_priority_items_arrive(self):
        queue = cppyy.gbl.cleanbot.hardware.PriorityWriteQueue(8)
        priority = cppyy.gbl.cleanbot.hardware.WritePriority
        queue.enqueue(byte_vector([1, 2, 3]), priority.kContinuous, "motion", False)
        before = cppyy.addressof(queue.front())

        queue.enqueue(byte_vector([4]), priority.kFinite, "", True)
        queue.enqueue(byte_vector([5]), priority.kSafety, "brake", True)
        queue.enqueue(byte_vector([6]), priority.kProtocolAck, "ack", True)

        self.assertEqual(cppyy.addressof(queue.front()), before)
        self.assertEqual(as_list(queue.front()), [1, 2, 3])

    def test_brush_only_status_zero_is_continuous_not_safety(self):
        classify = cppyy.gbl.cleanbot.hardware.classify_command_frame
        priority = cppyy.gbl.cleanbot.hardware.WritePriority
        brake = [0x7B, 0x00] + [0x00] * 19
        brush_only = list(brake)
        brush_only[10] = 70

        self.assertEqual(enum_value(classify(byte_vector(brake))), enum_value(priority.kSafety))
        self.assertEqual(
            enum_value(classify(byte_vector(brush_only))),
            enum_value(priority.kContinuous),
        )
        self.assertEqual(
            enum_value(classify(byte_vector([0x7B, 0x03] + [0x00] * 19))),
            enum_value(priority.kFinite),
        )


if __name__ == "__main__":
    unittest.main()
