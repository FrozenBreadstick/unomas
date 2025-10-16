"""
ROS2 Bridge (stubbed). Replace stubs with real rclpy integration when ready.
"""
import os
USE_STUB = os.environ.get("SIM_MODE", "1") == "1"

class ROS2Bridge:
    def __init__(self):
        self.initialized = not USE_STUB  # when real ROS is wired, flip this
        # TODO: if integrating, init rclpy and create publishers/subscribers here

    def publish_command(self, serial, cmd_type, params):
        # TODO: publish to ROS2 topic for the specific robot/serial
        if USE_STUB:
            return True
        return True

    def list_robots(self):
        # TODO: query registered robots from ROS or parameter server
        return []

    def get_robot_status(self, serial):
        # TODO: read from ROS topics
        return None
