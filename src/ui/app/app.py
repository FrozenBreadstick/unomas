import os, json, time, queue
from flask import Flask, request, jsonify, render_template, Response, send_from_directory
from threading import Lock
from sim import SwarmSim
from ros2_bridge import ROS2Bridge

app = Flask(__name__)
app.config["JSONIFY_PRETTYPRINT_REGULAR"] = False

sim = SwarmSim()
ros = ROS2Bridge()

listeners = []
listeners_lock = Lock()

def push_event(data):
    with listeners_lock:
        for q in list(listeners):
            try:
                q.put_nowait(data)
            except Exception:
                pass

@app.route("/")
def index():
    return render_template("index.html")

@app.get("/api/state")
def api_state():
    return jsonify(sim.snapshot())

@app.get("/api/robots")
def api_robots():
    return jsonify(sim.snapshot()["robots"])

@app.post("/api/robots/register")
def api_register_robot():
    data = request.get_json(force=True, silent=True) or {}
    serial = str(data.get("serial","")).strip()
    if not serial:
        return jsonify({"ok": False, "error": "serial required"}), 400
    sim.register_robot(serial)
    push_event({"type":"robots","payload":sim.snapshot()["robots"]})
    return jsonify({"ok": True})

@app.post("/api/mission")
def api_mission():
    data = request.get_json(force=True, silent=True) or {}
    origin = data.get("origin", {"x":0,"y":0})
    size = data.get("size", {"w":50,"h":30})
    spacing = data.get("spacing", 5)
    try:
        sim.set_mission(origin, size, spacing)
        push_event({"type":"mission","payload":sim.snapshot()["mission"]})
        return jsonify({"ok": True})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 400

@app.post("/api/command")
def api_command():
    data = request.get_json(force=True, silent=True) or {}
    target = data.get("target","all")
    cmd_type = data.get("type","").lower()
    params = data.get("params",{})

    if cmd_type == "start":
        sim.start()
    elif cmd_type == "pause":
        sim.pause()
    elif cmd_type == "resume":
        sim.resume()
    elif cmd_type == "return_home":
        serial = None if target == "all" else target
        sim.return_home(serial)
    else:
        return jsonify({"ok": False, "error":"unknown command"}), 400

    push_event({"type":"state","payload":sim.snapshot()})
    return jsonify({"ok": True})

@app.get("/events")
def sse():
    def gen():
        q = queue.Queue()
        with listeners_lock:
            listeners.append(q)
        # send initial state
        q.put({"type":"state","payload":sim.snapshot()})
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
    # Provide a sensible default mission for demo
    sim.set_mission({"x":5,"y":5},{"w":60,"h":40},5)
    # Register a couple of demo robots
    sim.register_robot("R-1001")
    sim.register_robot("R-1002")
    app.run(host="127.0.0.1", port=5000, debug=True)
