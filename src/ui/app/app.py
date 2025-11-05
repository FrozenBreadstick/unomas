import os
import atexit
from flask import Flask, jsonify, Response, send_file

import rclpy
from rclpy.node import Node
from unomas.srv import StatusUpdateService, TerrainSoilData

app = Flask(__name__)
app.config["JSONIFY_PRETTYPRINT_REGULAR"] = False

# ---------------- ROS2 clients ----------------
class StatusClient(Node):
    def __init__(self, service_name: str = "update"):
        super().__init__("ui_status_client")
        self._client = self.create_client(StatusUpdateService, service_name)

    def fetch_packet(self, timeout_sec: float = 2.0):
        if not self._client.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError("Status service 'update' is not available.")
        req = StatusUpdateService.Request()
        fut = self._client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout_sec)
        if fut.result() is None:
            raise RuntimeError("Status service returned no result.")
        return fut.result().data

class SoilClient(Node):
    def __init__(self, service_name: str = "soil_update"):
        super().__init__("ui_soil_client")
        self._client = self.create_client(TerrainSoilData, service_name)

    def fetch(self, timeout_sec: float = 2.5):
        if not self._client.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError("Soil service 'soil_update' is not available.")
        req = TerrainSoilData.Request()
        fut = self._client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout_sec)
        if fut.result() is None:
            raise RuntimeError("Soil service returned no result.")
        return fut.result().data

def packet_to_dict(p):
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

def soil_to_dict(soil_msg):
    out = {"samples": []}
    for s in soil_msg.samples:
        out["samples"].append({
            "x": float(s.x), "y": float(s.y),
            "moisture": float(s.moisture),
            "ph": float(s.ph),
            "nutrients": float(s.nutrients),
        })
    return out

# --- Initialize ROS exactly once, then create nodes ---
_status_client = None
_soil_client = None

def _init_ros_once():
    global _status_client, _soil_client
    if not rclpy.ok():
        rclpy.init(args=None)
    if _status_client is None:
        _status_client = StatusClient()
    if _soil_client is None:
        _soil_client = SoilClient()

_init_ros_once()

@atexit.register
def _shutdown_ros():
    try:
        if _status_client is not None:
            _status_client.destroy_node()
    except Exception:
        pass
    try:
        if _soil_client is not None:
            _soil_client.destroy_node()
    except Exception:
        pass
    if rclpy.ok():
        rclpy.shutdown()

@app.get("/api/status")
def api_status():
    try:
        packet = _status_client.fetch_packet(timeout_sec=2.0)
        return jsonify(packet_to_dict(packet))
    except Exception as e:
        return jsonify({"error": str(e)}), 503

@app.get("/api/soil")
def api_soil():
    try:
        soil = _soil_client.fetch(timeout_sec=2.5)
        return jsonify(soil_to_dict(soil))
    except Exception as e:
        return jsonify({"samples": [], "error": str(e)}), 200

@app.get("/")
def home():
    return send_file("templates/heatmap.html")

# from flask import send_file
# @app.get("/status")
# def status():
#     return send_file("templates/index.html")

# from flask import send_file
# @app.get("/map")
# def map_page():
#     return send_file("templates/map.html")

if __name__ == "__main__":
    app.run(host="127.0.0.1", port=5000, debug=True, use_reloader=False)
