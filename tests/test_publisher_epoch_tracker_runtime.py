import ctypes
import os
import sysconfig
import unittest
from pathlib import Path


WORKSPACE = Path(__file__).resolve().parents[1]
PACKAGE = WORKSPACE / "src" / "cleanbot_common"
HEADER = PACKAGE / "include" / "cleanbot_common" / "publisher_epoch_tracker.hpp"
RUNTIME_AVAILABLE = HEADER.is_file()

if RUNTIME_AVAILABLE:
    CPPYY_BACKEND_BIN = Path(sysconfig.get_paths()["purelib"]) / "cppyy_backend" / "bin"
    if os.name == "nt" and CPPYY_BACKEND_BIN.is_dir():
        os.add_dll_directory(str(CPPYY_BACKEND_BIN))
        os.environ["PATH"] = str(CPPYY_BACKEND_BIN) + os.pathsep + os.environ.get(
            "PATH", ""
        )
        for dll_name in (
            "msvcp140.dll",
            "msvcp140_1.dll",
            "msvcp140_2.dll",
            "vcruntime140.dll",
            "vcruntime140_1.dll",
        ):
            ctypes.WinDLL(str(CPPYY_BACKEND_BIN / dll_name))

    import cppyy

    cppyy.add_include_path(str(PACKAGE / "include"))
    cppyy.cppdef('#include "cleanbot_common/publisher_epoch_tracker.hpp"\n')
else:
    cppyy = None


def identity(implementation_identifier, gid):
    publisher = cppyy.gbl.cleanbot.common.PublisherIdentity()
    publisher.implementation_identifier = implementation_identifier
    for value in gid:
        publisher.gid.push_back(value)
    return publisher


@unittest.skipUnless(RUNTIME_AVAILABLE, "publisher epoch tracker header is unavailable")
class PublisherEpochTrackerRuntimeTest(unittest.TestCase):
    def setUp(self):
        self.api = cppyy.gbl.cleanbot.common

    def test_first_current_switch_and_retired_observations(self):
        tracker = self.api.PublisherEpochTracker()
        accepted = self.api.PublisherEpochStatus.kAccepted
        retired = self.api.PublisherEpochStatus.kRetired
        first = identity("rmw-a", [1, 2, 3])
        second = identity("rmw-a", [4, 5, 6])

        first_result = tracker.observe(first)
        repeated_result = tracker.observe(first)
        switched_result = tracker.observe(second)
        retired_result = tracker.observe(first)

        self.assertEqual(first_result.status, accepted)
        self.assertEqual(first_result.epoch, 1)
        self.assertTrue(first_result.session_changed)
        self.assertEqual(repeated_result.status, accepted)
        self.assertEqual(repeated_result.epoch, 1)
        self.assertFalse(repeated_result.session_changed)
        self.assertEqual(switched_result.status, accepted)
        self.assertEqual(switched_result.epoch, 2)
        self.assertTrue(switched_result.session_changed)
        self.assertEqual(retired_result.status, retired)
        self.assertEqual(retired_result.epoch, 2)
        self.assertFalse(retired_result.session_changed)

    def test_rejects_invalid_identity_forms(self):
        tracker = self.api.PublisherEpochTracker()
        invalid = self.api.PublisherEpochStatus.kInvalid

        results = (
            tracker.observe(identity("", [1])),
            tracker.observe(identity("rmw-a", [])),
            tracker.observe(identity("rmw-a", [0, 0, 0])),
        )

        for result in results:
            self.assertEqual(result.status, invalid)
            self.assertEqual(result.epoch, 0)
            self.assertFalse(result.session_changed)

    def test_bounds_owned_identity_fields(self):
        tracker = self.api.PublisherEpochTracker()
        maximum_identifier_bytes = int(
            self.api.PublisherEpochTracker
            .kMaximumImplementationIdentifierBytes
        )
        maximum_gid_bytes = int(
            self.api.PublisherEpochTracker.kMaximumGidBytes
        )

        maximum = tracker.observe(
            identity(
                "r" * maximum_identifier_bytes,
                [0] * (maximum_gid_bytes - 1) + [1],
            )
        )
        self.assertEqual(
            maximum.status,
            self.api.PublisherEpochStatus.kAccepted,
        )
        self.assertEqual(maximum.epoch, 1)
        self.assertTrue(maximum.session_changed)

        invalid_identifier = tracker.observe(
            identity("r" * (maximum_identifier_bytes + 1), [1])
        )
        invalid_gid = tracker.observe(
            identity("rmw-a", [1] * (maximum_gid_bytes + 1))
        )
        for result in (invalid_identifier, invalid_gid):
            self.assertEqual(
                result.status,
                self.api.PublisherEpochStatus.kInvalid,
            )
            self.assertEqual(result.epoch, 1)
            self.assertFalse(result.session_changed)

    def test_distinguishes_implementation_identifier_and_full_gid(self):
        tracker = self.api.PublisherEpochTracker()

        first = tracker.observe(identity("rmw-a", [1, 2, 3]))
        implementation_changed = tracker.observe(identity("rmw-b", [1, 2, 3]))
        gid_changed = tracker.observe(identity("rmw-b", [1, 2, 4]))

        self.assertEqual(first.epoch, 1)
        self.assertEqual(implementation_changed.epoch, 2)
        self.assertTrue(implementation_changed.session_changed)
        self.assertEqual(gid_changed.epoch, 3)
        self.assertTrue(gid_changed.session_changed)

    def test_retired_identity_precedes_exhaustion(self):
        tracker = self.api.PublisherEpochTracker(1, 2)
        first = identity("rmw-a", [1])
        second = identity("rmw-a", [2])

        tracker.observe(first)
        tracker.observe(second)
        result = tracker.observe(first)

        self.assertEqual(result.status, self.api.PublisherEpochStatus.kRetired)
        self.assertEqual(result.epoch, 2)
        self.assertFalse(result.session_changed)

    def test_capacity_and_epoch_limits_fail_closed_without_mutating(self):
        capacity_limited = self.api.PublisherEpochTracker(0)
        first = identity("rmw-a", [1])
        second = identity("rmw-a", [2])
        capacity_limited.observe(first)

        capacity_result = capacity_limited.observe(second)
        current_after_capacity = capacity_limited.observe(first)

        self.assertEqual(
            capacity_result.status,
            self.api.PublisherEpochStatus.kExhausted,
        )
        self.assertEqual(capacity_result.epoch, 1)
        self.assertFalse(capacity_result.session_changed)
        self.assertEqual(
            current_after_capacity.status,
            self.api.PublisherEpochStatus.kAccepted,
        )
        self.assertEqual(current_after_capacity.epoch, 1)

        epoch_limited = self.api.PublisherEpochTracker(8, 1)
        epoch_limited.observe(first)
        epoch_result = epoch_limited.observe(second)
        current_after_epoch = epoch_limited.observe(first)

        self.assertEqual(
            epoch_result.status,
            self.api.PublisherEpochStatus.kExhausted,
        )
        self.assertEqual(epoch_result.epoch, 1)
        self.assertFalse(epoch_result.session_changed)
        self.assertEqual(
            current_after_epoch.status,
            self.api.PublisherEpochStatus.kAccepted,
        )
        self.assertEqual(current_after_epoch.epoch, 1)


class PublisherEpochTrackerAvailabilityTest(unittest.TestCase):
    def test_header_exists(self):
        self.assertTrue(HEADER.is_file(), str(HEADER))


if __name__ == "__main__":
    unittest.main()
