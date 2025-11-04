#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from unomas.msg import StatusUpdatePacket
from unomas.srv import StatusUpdateService


class UIBridge(Node):
    def __init__(self):
        super().__init__('ui_bridge_node')

        # Latest packet cache
        self._latest = StatusUpdatePacket()

        # Subscriber: /unomas/UpdatePacket
        self._packet_sub = self.create_subscription(
            StatusUpdatePacket,
            '/unomas/UpdatePacket',
            self._on_packet,
            10
        )

        # Service: "update"
        self._srv = self.create_service(
            StatusUpdateService,
            'update',
            self._handle_update
        )

        self.get_logger().info('UI Bridge node started (Python).')

    # Callback for incoming packets
    def _on_packet(self, msg: StatusUpdatePacket):
        self._latest = msg
        # Match C++ log line:
        self.get_logger().info(f"Received Status Update Packet from station: '{self._latest.name}'")

    # Service handler: returns the latest cached packet
    def _handle_update(self, request: StatusUpdateService.Request,
                       response: StatusUpdateService.Response) -> StatusUpdateService.Response:
        response.data = self._latest
        # Match C++ log line:
        self.get_logger().info(f"Sent Status Update Packet to UI for station: '{self._latest.name}'")
        return response


def main():
    rclpy.init()
    node = UIBridge()
    try:
        rclpy.spin(node)
    finally:
        node.get_logger().info('UI Bridge node is shutting down.')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
