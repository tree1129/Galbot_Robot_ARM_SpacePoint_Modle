"""Read-only Galbot G1 camera bridge.

This module deliberately imports neither GalbotMotion nor any joint/gripper command.
It can be deployed for calibration and frame collection without creating an actuator path.
"""

from __future__ import annotations


class GalbotG1ReadOnlyCameras:
    def __init__(self):
        try:
            from galbot_sdk.g1 import GalbotRobot, SensorType
        except ImportError as exc:
            raise RuntimeError("Galbot G1 SDK is available only on the robot runtime") from exc
        self._SensorType = SensorType
        self._robot = GalbotRobot()
        sensors = {
            SensorType.HEAD_LEFT_CAMERA,
            SensorType.HEAD_RIGHT_CAMERA,
            SensorType.LEFT_ARM_CAMERA,
            SensorType.LEFT_ARM_DEPTH_CAMERA,
            SensorType.RIGHT_ARM_CAMERA,
            SensorType.RIGHT_ARM_DEPTH_CAMERA,
        }
        if not self._robot.init(sensors):
            raise RuntimeError("GalbotRobot sensor-only initialization failed")

    def snapshot(self) -> dict:
        sensor = self._SensorType
        return {
            "head_left_rgb": self._robot.get_rgb_data(sensor.HEAD_LEFT_CAMERA),
            "head_right_rgb": self._robot.get_rgb_data(sensor.HEAD_RIGHT_CAMERA),
            "left_rgb": self._robot.get_rgb_data(sensor.LEFT_ARM_CAMERA),
            "left_depth": self._robot.get_depth_data(sensor.LEFT_ARM_DEPTH_CAMERA),
            "right_rgb": self._robot.get_rgb_data(sensor.RIGHT_ARM_CAMERA),
            "right_depth": self._robot.get_depth_data(sensor.RIGHT_ARM_DEPTH_CAMERA),
        }

    def calibration(self, camera_name: str) -> dict:
        mapping = {
            "left": self._SensorType.LEFT_ARM_DEPTH_CAMERA,
            "right": self._SensorType.RIGHT_ARM_DEPTH_CAMERA,
        }
        sensor = mapping[camera_name]
        extrinsic, timestamp_ns = self._robot.get_sensor_extrinsic(sensor, "base_link")
        return {
            "intrinsic": self._robot.get_camera_intrinsic(sensor),
            "camera_to_base": extrinsic,
            "timestamp_ns": timestamp_ns,
        }

