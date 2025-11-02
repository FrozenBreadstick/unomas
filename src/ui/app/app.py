from copy import deepcopy
from threading import Lock
import json
import queue
from flask import Flask, jsonify, render_template, Response
from ros2_bridge import ROS2Bridge

app = Flask(__name__)
app.config["JSONIFY_PRETTYPRINT_REGULAR"] = False

ros = ROS2Bridge()

listeners = []
listeners_lock = Lock()
odom_lock = Lock()
latest_odometry = {
    "topic": "/odom",
    "frame_id": "unknown",
    "stamp": {"sec": 0, "nanosec": 0},
    "position": {"x": 0.0, "y": 0.0, "z": 0.0},
    "orientation": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0},
    "linear_velocity": {"x": 0.0, "y": 0.0, "z": 0.0},
    "angular_velocity": {"x": 0.0, "y": 0.0, "z": 0.0},
}

def push_event(data):
    with listeners_lock:
        for q in list(listeners):
            try:
                q.put_nowait(data)
            except Exception:
                pass

def _handle_odometry(packet):
    global latest_odometry
    with odom_lock:
        latest_odometry = packet
    push_event({"type": "odometry", "payload": packet})

ros.start_listener(_handle_odometry)
app.logger.info("ROS2 odometry listener active.")

@app.route("/")
def index():
    return render_template("index.html")

@app.get("/api/odometry")
def api_odometry():
    with odom_lock:
        payload = deepcopy(latest_odometry)
    return jsonify(payload)

@app.get("/events")
def sse():
    def gen():
        q = queue.Queue()
        with listeners_lock:
            listeners.append(q)
        with odom_lock:
            initial = deepcopy(latest_odometry)
        q.put({"type": "odometry", "payload": initial})
        try:
            while True:
                data = q.get()
                yield f"data: {json.dumps(data)}\n\n"
        except GeneratorExit:
            with listeners_lock:
                if q in listeners:
                    listeners.remove(q)
    return Response(gen(), mimetype="text/event-stream")

if __name__ == "__main__":
    app.run(host="127.0.0.1", port=5000, debug=True)
