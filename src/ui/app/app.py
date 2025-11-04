import atexit
from threading import Lock
from typing import Any, Dict

from flask import Flask, jsonify, render_template

import rclpy
from rclpy.node import Node

from unomas.srv import StatusUpdateService

app = Flask(__name__)
app.config["JSONIFY_PRETTYPRINT_REGULAR"] = False


class _StatusClient(Node):
    """Thin rclpy client that requests the latest StatusUpdatePacket."""

    def __init__(self):
        super().__init__("ui_app_status_client")
        self._client = self.create_client(StatusUpdateService, "update")
        self._lock = Lock()

    def fetch_packet(self, timeout_sec: float = 1.0):
        if not self._client.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError("Status service 'update' is not available.")
        request = StatusUpdateService.Request()
        with self._lock:
            future = self._client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        if future.result() is not None:
            return future.result().data
        if future.exception() is not None:
            raise RuntimeError("Status service call failed.") from future.exception()
        raise RuntimeError("Status service returned no result.")


def _ensure_rclpy_init():
    if not rclpy.is_initialized():
        rclpy.init(args=None)


def _packet_to_dict(packet) -> Dict[str, Any]:
    return {
        "name": packet.name,
        "emergency": bool(packet.emergency),
        "battery": int(packet.battery),
        "current_state": packet.current_state,
        "current_position": {
            "x": float(packet.current_position.x),
            "y": float(packet.current_position.y),
            "z": float(packet.current_position.z),
        },
        "target_position": {
            "x": float(packet.target_position.x),
            "y": float(packet.target_position.y),
            "z": float(packet.target_position.z),
        },
    }


_ensure_rclpy_init()
_status_client = _StatusClient()


@atexit.register
def _cleanup():
    _status_client.destroy_node()
    if rclpy.is_initialized():
        rclpy.shutdown()


@app.route("/")
def index():
    return render_template("index.html")


@app.get("/api/status")
def api_status():
    try:
        packet = _status_client.fetch_packet(timeout_sec=2.0)
        payload = _packet_to_dict(packet)
    except RuntimeError as err:
        app.logger.warning("Failed to fetch latest status packet: %s", err)
        return jsonify({"error": str(err)}), 503
    return jsonify(payload)


if __name__ == "__main__":
    app.run(host="127.0.0.1", port=5000, debug=True)
