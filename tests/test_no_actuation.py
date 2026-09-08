import ast
import inspect
import unittest

import g1_grasp.adapters.galbot_sdk_readonly as adapter


class NoActuationTests(unittest.TestCase):
    def test_robot_adapter_has_no_motion_commands(self):
        source = inspect.getsource(adapter)
        tree = ast.parse(source)
        referenced = {
            node.id for node in ast.walk(tree) if isinstance(node, ast.Name)
        } | {
            node.attr for node in ast.walk(tree) if isinstance(node, ast.Attribute)
        }
        for forbidden in (
            "GalbotMotion",
            "set_end_effector_pose",
            "set_joint_positions",
            "set_gripper_command",
            "navigate_to_goal",
        ):
            self.assertNotIn(forbidden, referenced)


if __name__ == "__main__":
    unittest.main()
