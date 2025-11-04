# app.py
import atexit
from flask import Flask, jsonify, Response

import rclpy
from rclpy.node import Node

from unomas.srv import StatusUpdateService  # response includes .data: StatusUpdatePacket

app = Flask(__name__)
app.config["JSONIFY_PRETTYPRINT_REGULAR"] = False

# ---------------- ROS2 client ----------------
class StatusClient(Node):
    def __init__(self, service_name: str = "update"):
        super().__init__("ui_status_client")
        self._client = self.create_client(StatusUpdateService, service_name)

    def fetch_packet(self, timeout_sec: float = 2.0):
        if not self._client.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError("Status service 'update' is not available.")
        req = StatusUpdateService.Request()  # empty request
        fut = self._client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout_sec)
        if fut.result() is None:
            raise RuntimeError("Status service returned no result.")
        return fut.result().data  # StatusUpdatePacket

def packet_to_dict(p):
    # StatusUpdatePacket.msg:
    # string name
    # bool emergency
    # int32 battery
    # geometry_msgs/Point current_position
    # string current_state
    # geometry_msgs/Point target_position
    return {
        "name": p.name,
        "emergency": bool(p.emergency),
        "battery": int(p.battery),
        "current_state": p.current_state,
        "current_position": {"x": float(p.current_position.x),
                             "y": float(p.current_position.y),
                             "z": float(p.current_position.z)},
        "target_position": {"x": float(p.target_position.x),
                            "y": float(p.target_position.y),
                            "z": float(p.target_position.z)},
    }

# Init ROS once
rclpy.init(args=None)
_status_client = StatusClient()

@atexit.register
def _shutdown_ros():
    try:
        _status_client.destroy_node()
    except Exception:
        pass
    if rclpy.ok():
        rclpy.shutdown()

# ---------------- HTTP routes ----------------
@app.get("/api/status")
def api_status():
    try:
        packet = _status_client.fetch_packet(timeout_sec=2.0)
        return jsonify(packet_to_dict(packet))
    except Exception as e:
        return jsonify({"error": str(e)}), 503
    
from flask import send_file
@app.get("/")
def home():
    return send_file("templates/home.html")
    
# from flask import send_file
# @app.get("/status")
# def status():
#     return send_file("templates/index.html")

# from flask import send_file
# @app.get("/map")
# def map_page():
#     return send_file("templates/map.html")


if __name__ == "__main__":
    # Visit http://127.0.0.1:5000 in your browser
    app.run(host="127.0.0.1", port=5000, debug=True)
