# UNO MAS – Simple Swarm Control Web UI (One-Folder App)

This starter web app gives you a single-folder website + Python backend to **control and monitor a swarm** of farm robots for the UNO MAS project.

- **One folder**: everything is inside `unomas_control_ui/`
- **Runs locally** (no external CDNs) with Flask
- **Simulation mode** by default (no ROS2 needed). A ROS2 bridge is stubbed and can be wired up later.
- **Features**:
  - Register robots by serial
  - Define a rectangular field and **grid sampling spacing**
  - Start/Pause/Resume/Return-to-Base
  - Live dashboard + simple canvas map
  - REST API + Server‑Sent Events for live status

> This is intentionally minimal but clean. You can evolve it into your production UI.

---

## Quick Start (Windows)

1) Open **PowerShell** in this folder.
2) Run: `.un_app.bat`
3) Open http://127.0.0.1:5000

## Quick Start (Ubuntu / macOS)

```bash
chmod +x run_app.sh
./run_app.sh
# Then open http://127.0.0.1:5000
```

---

## Files
- `app.py` – Flask server, REST API, SSE, and simulation loop
- `sim.py` – Simple swarm simulator (no ROS2 required)
- `ros2_bridge.py` – Stubs for future ROS2 integration (publishing commands, etc.)
- `templates/index.html` – UI
- `static/css/style.css` – Styling (no external CSS)
- `static/js/app.js` – Front-end logic
- `requirements.txt` – Minimal deps (Flask)

---

## ROS2 (optional, future)
- `ros2_bridge.py` is structured so you can replace the stub methods with real `rclpy` publishers/subscribers.
- Endpoints to consider wiring:
  - `POST /api/command` → publish robot control messages
  - `POST /api/robots/register` → register robot serials
  - `GET /api/robots` → reflect robot status from ROS topics

---

## API (selected)
- `GET /api/state` → `{ mission, robots }`
- `GET /api/robots` → list robots
- `POST /api/robots/register` → `{ serial }`
- `POST /api/mission` → `{ origin:{x,y}, size:{w,h}, spacing }`
- `POST /api/command` → `{ target:"all"|"serial", type:"start|pause|resume|return_home", params:{} }`

Enjoy! – Max Spark
