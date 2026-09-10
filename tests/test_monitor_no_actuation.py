import ast
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]


class MonitorNoActuationTests(unittest.TestCase):
    def test_service_has_vendor_library_path(self):
        unit = (ROOT / "deploy/systemd/galbot-feeding-monitor.service").read_text()
        self.assertIn("Environment=LD_LIBRARY_PATH=/data/galbot/lib\n", unit)

    def test_monitor_source_has_get_routes_only_and_no_actuation_symbols(self):
        source = (ROOT / "src/g1_grasp/monitor.py").read_text(encoding="utf-8")
        tree = ast.parse(source)
        attributes = {node.attr for node in ast.walk(tree) if isinstance(node, ast.Attribute)}
        imported = {alias.name for node in ast.walk(tree) if isinstance(node, ast.ImportFrom) for alias in node.names}
        for forbidden in (
            "GalbotMotion",
            "GalbotNavigation",
            "set_joint_positions",
            "set_gripper_command",
            "set_end_effector_pose",
            "navigate_to_goal",
        ):
            self.assertNotIn(forbidden, attributes | imported)
        handler_methods = {
            node.name
            for node in ast.walk(tree)
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
            and node.name.startswith("do_")
        }
        self.assertEqual(handler_methods, {"do_GET"})

    def test_deployed_monitor_uses_camera_only_v4l2_fallback(self):
        unit = (ROOT / "deploy/systemd/galbot-feeding-monitor.service").read_text(
            encoding="utf-8"
        )
        self.assertIn("Environment=G1_CAMERA_MODE=v4l2", unit)
        self.assertIn("Environment=G1_HEAD_DEVICE=/dev/video6", unit)
        self.assertIn("Environment=G1_LEFT_WRIST_DEVICE=/dev/video4", unit)
        self.assertIn("Environment=G1_RIGHT_WRIST_DEVICE=/dev/video12", unit)


if __name__ == "__main__":
    unittest.main()
