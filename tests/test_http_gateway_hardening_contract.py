import unittest
from pathlib import Path


WORKSPACE = Path(__file__).resolve().parents[1]
PACKAGE = WORKSPACE / "src" / "cleanbot_http"
FRONTEND_API = WORKSPACE.parent / "clean-frontend-github" / "api" / "login.js"


class HttpGatewayHardeningContractTest(unittest.TestCase):
    def test_async_server_and_session_are_split_from_ros_node(self):
        required = (
            "include/cleanbot_http/http_server.hpp",
            "include/cleanbot_http/http_session.hpp",
            "src/http_server.cpp",
            "src/http_session.cpp",
            "test/test_http_server.cpp",
        )
        missing = [item for item in required if not (PACKAGE / item).is_file()]
        self.assertEqual(missing, [])

        node = (PACKAGE / "src/http_gateway_node.cpp").read_text(encoding="utf-8")
        self.assertNotIn("beast_http::read(", node)
        self.assertNotIn("acceptor->accept(", node)
        self.assertNotIn("std::this_thread::sleep_for", node)
        self.assertIn("HttpServer", node)

    def test_sequence_guard_is_part_of_http_core(self):
        required = (
            "include/cleanbot_http/joystick_sequence_guard.hpp",
            "src/joystick_sequence_guard.cpp",
        )
        missing = [item for item in required if not (PACKAGE / item).is_file()]
        self.assertEqual(missing, [])

        router = (PACKAGE / "src/http_control_router.cpp").read_text(
            encoding="utf-8"
        )
        for token in (
            "sessionId",
            "sequence",
            "STALE_JOYSTICK_COMMAND",
        ):
            self.assertIn(token, router)

    def test_frontend_adds_sequence_only_to_joystick_requests(self):
        if not FRONTEND_API.is_file():
            self.skipTest("frontend repository is not present beside ROS2 workspace")
        source = FRONTEND_API.read_text(encoding="utf-8")
        self.assertIn("joystickSessionId", source)
        self.assertIn("joystickSequence", source)
        self.assertIn("sessionId=", source)
        self.assertIn("sequence=", source)

        parking_function = source[source.index("export function stop()") :]
        parking_function = parking_function[: parking_function.index("}\n") + 2]
        self.assertNotIn("sessionId", parking_function)
        self.assertNotIn("sequence", parking_function)


if __name__ == "__main__":
    unittest.main()
