import atexit
from collections.abc import Iterable
from typing import Dict, Optional
from flask import Flask, jsonify, send_file, request

import rclpy
from rclpy.node import Node
from rclpy import spin_once
from rclpy.subscription import Subscription
from unomas.srv import StatusUpdateService, TerrainSoilData, UpdateMacroPlan
from unomas.msg import (MacroPlan as MacroPlanMsg, FieldPlan, RoutePlan,
                        AddressedPoseArray)
from geometry_msgs.msg import Point

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

class MacroPlanClient(Node):
    def __init__(self, service_name: str = "macro_plan_update"):
        super().__init__("ui_macro_plan_client")
        self._client = self.create_client(UpdateMacroPlan, service_name)

    def submit(self, plan_msg: MacroPlanMsg, timeout_sec: float = 3.0):
        if not self._client.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError("Macro plan service 'macro_plan_update' is not available.")
        req = UpdateMacroPlan.Request()
        req.plan = plan_msg
        fut = self._client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout_sec)
        if fut.result() is None:
            raise RuntimeError("Macro plan service returned no result.")
        return fut.result()


class GoalListener(Node):
    def __init__(self):
        super().__init__("ui_goal_listener")
        self._subs: Dict[str, Subscription] = {}
        self._latest: Dict[str, AddressedPoseArray] = {}

    def ensure_subscription(self, station: str) -> None:
        if station in self._subs:
            return

        topic = f"{station}/goals"
        self._latest.setdefault(station, AddressedPoseArray())

        def _callback(msg: AddressedPoseArray, sta: str = station):
            self._latest[sta] = msg

        self._subs[station] = self.create_subscription(
            AddressedPoseArray,
            topic,
            _callback,
            10,
        )
        self.get_logger().info(f"Subscribed to '{topic}' for route preview data.")

    def latest(self, station: str, robot: Optional[str]) -> Optional[AddressedPoseArray]:
        msg = self._latest.get(station)
        if msg is None or not msg.poses:
            return None
        if robot and msg.address and msg.address != robot:
            return None
        return msg

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

def _point_from_dict(data: dict | None) -> Point:
    pt = data or {}
    point = Point()
    point.x = float(pt.get("x", 0.0))
    point.y = float(pt.get("y", 0.0))
    point.z = float(pt.get("z", 0.0))
    return point

def dict_to_macro_plan(payload: dict, fallback_start: Point | None) -> MacroPlanMsg:
    plan = MacroPlanMsg()
    plan.station_name = str(payload.get("station_name", "")).strip()
    plan.robot_address = str(payload.get("robot_address", "")).strip()
    plan.visit_all_fields = bool(payload.get("visit_all_fields", True))

    start_payload = payload.get("start")
    if isinstance(start_payload, dict):
        plan.start = _point_from_dict(start_payload)
    elif fallback_start is not None:
        plan.start = fallback_start
    else:
        plan.start = Point()

    plan.fields.clear()
    for idx, field in enumerate(payload.get("fields", []), start=1):
        gate = field.get("gate")
        if not isinstance(gate, dict):
            continue
        fp = FieldPlan()
        fp.label = str(field.get("label", f"Field {idx}")).strip() or f"Field {idx}"
        fp.enabled = bool(field.get("enabled", True))
        fp.gate = _point_from_dict(gate)
        outline = field.get("outline", [])
        if isinstance(outline, Iterable):
            fp.outline = [_point_from_dict(pt) for pt in outline if isinstance(pt, dict)]
        plan.fields.append(fp)

    plan.routes.clear()
    for idx, route in enumerate(payload.get("routes", []), start=1):
        waypoints = route.get("waypoints", [])
        if not isinstance(waypoints, Iterable):
            continue
        rp = RoutePlan()
        rp.label = str(route.get("label", f"Route {idx}")).strip() or f"Route {idx}"
        rp.width = float(route.get("width", 0.0) or 0.0)
        rp.waypoints = [_point_from_dict(pt) for pt in waypoints if isinstance(pt, dict)]
        if len(rp.waypoints) >= 2:
            plan.routes.append(rp)

    if not plan.station_name:
        raise ValueError("station_name is required.")
    if not plan.robot_address:
        raise ValueError("robot_address is required.")
    if not plan.routes:
        raise ValueError("at least one route must be provided.")
    if not any(field.enabled for field in plan.fields):
        raise ValueError("at least one gate must be enabled.")

    return plan

# --- Initialize ROS exactly once, then create nodes ---
_status_client = None
_soil_client = None
_plan_client = None
_last_robot_point: Point | None = None
_goal_listener: GoalListener | None = None

def _init_ros_once():
    global _status_client, _soil_client, _plan_client, _goal_listener
    if not rclpy.ok():
        rclpy.init(args=None)
    if _status_client is None:
        _status_client = StatusClient()
    if _soil_client is None:
        _soil_client = SoilClient()
    if _plan_client is None:
        _plan_client = MacroPlanClient()
    if _goal_listener is None:
        _goal_listener = GoalListener()

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
    try:
        if _plan_client is not None:
            _plan_client.destroy_node()
    except Exception:
        pass
    try:
        if _goal_listener is not None:
            _goal_listener.destroy_node()
    except Exception:
        pass
    if rclpy.ok():
        rclpy.shutdown()

@app.get("/api/status")
def api_status():
    try:
        packet = _status_client.fetch_packet(timeout_sec=2.0)
        global _last_robot_point
        _last_robot_point = Point()
        _last_robot_point.x = float(packet.current_position.x)
        _last_robot_point.y = float(packet.current_position.y)
        _last_robot_point.z = float(packet.current_position.z)
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

@app.post("/api/macro-plan")
def api_macro_plan():
    try:
        payload = request.get_json(force=True, silent=False)
    except Exception as exc:
        return jsonify({"accepted": False, "message": f"Invalid JSON: {exc}"}), 400

    if not isinstance(payload, dict):
        return jsonify({"accepted": False, "message": "Request body must be a JSON object."}), 400

    try:
        plan_msg = dict_to_macro_plan(payload, _last_robot_point)
    except Exception as exc:
        return jsonify({"accepted": False, "message": f"Failed to parse plan: {exc}"}), 400

    try:
        result = _plan_client.submit(plan_msg)
        status_code = 200 if result.accepted else 409
        return jsonify({"accepted": bool(result.accepted), "message": result.message}), status_code
    except Exception as exc:
        return jsonify({"accepted": False, "message": str(exc)}), 503


@app.get("/api/route-preview")
def api_route_preview():
    station = (request.args.get("station") or "").strip()
    robot = (request.args.get("robot") or "").strip()

    if not station:
        return jsonify({"error": "station parameter required"}), 400
    if _goal_listener is None:
        return jsonify({"error": "goal listener unavailable"}), 500

    _goal_listener.ensure_subscription(station)
    # Pump callbacks to process any pending goal messages
    for _ in range(3):
        spin_once(_goal_listener, timeout_sec=0.05)

    msg = _goal_listener.latest(station, robot if robot else None)
    if msg is None or not msg.poses:
        return jsonify({"error": "no route data available"}), 404

    goals = [{
        "x": float(p.position.x),
        "y": float(p.position.y),
        "z": float(p.position.z)
    } for p in msg.poses]

    return jsonify({
        "station": station,
        "robot": msg.address,
        "goals": goals
    })

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
