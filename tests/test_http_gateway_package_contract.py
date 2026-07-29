import unittest
from pathlib import Path


WORKSPACE = Path(__file__).resolve().parents[1]
PACKAGE = WORKSPACE / "src" / "cleanbot_http"


class HttpGatewayPackageContractTest(unittest.TestCase):
    def test_package_contains_router_node_and_native_tests(self):
        expected = (
            "CMakeLists.txt",
            "package.xml",
            "include/cleanbot_http/http_control_router.hpp",
            "include/cleanbot_http/http_server.hpp",
            "include/cleanbot_http/http_session.hpp",
            "src/http_control_router.cpp",
            "src/http_server.cpp",
            "src/http_session.cpp",
            "src/http_gateway_node.cpp",
            "test/test_http_control_router.cpp",
            "test/test_http_server.cpp",
        )
        for relative in expected:
            self.assertTrue((PACKAGE / relative).is_file(), relative)

    def test_node_uses_beast_config_and_existing_control_topics(self):
        node = (PACKAGE / "src/http_gateway_node.cpp").read_text(encoding="utf-8")
        session = (PACKAGE / "src/http_session.cpp").read_text(encoding="utf-8")

        self.assertIn("beast_http::async_read", session)
        self.assertIn("beast_http::async_write", session)
        self.assertIn("ConfigClient", node)
        self.assertIn('"/control/manual_cmd"', node)
        self.assertIn('"/control/emergency_cmd"', node)
        self.assertIn("expires_after", session)
        self.assertIn("Access-Control-Allow-Origin", session)
        self.assertIn("server_thread_", node)
        self.assertIn("join", node)

    def test_accept_loop_is_non_blocking_so_shutdown_cannot_deadlock(self):
        server = (PACKAGE / "src/http_server.cpp").read_text(encoding="utf-8")
        node = (PACKAGE / "src/http_gateway_node.cpp").read_text(encoding="utf-8")

        self.assertIn("async_accept", server)
        self.assertIn("acceptNext", server)
        self.assertIn("server_->stop()", node)
        self.assertNotIn("acceptor_.accept(", server)

    def test_build_links_ros_control_boost_and_threads(self):
        cmake = (PACKAGE / "CMakeLists.txt").read_text(encoding="utf-8")
        manifest = (PACKAGE / "package.xml").read_text(encoding="utf-8")

        self.assertIn("find_package(Boost REQUIRED COMPONENTS system)", cmake)
        self.assertIn("find_package(Threads REQUIRED)", cmake)
        self.assertIn("cleanbot_control", cmake)
        self.assertIn("Boost::system", cmake)
        self.assertIn("Threads::Threads", cmake)
        self.assertIn("<depend>boost</depend>", manifest)


if __name__ == "__main__":
    unittest.main()
