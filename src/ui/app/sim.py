import threading, time, math, random
from dataclasses import dataclass, field

@dataclass
class Mission:
    origin_x: float = 0.0
    origin_y: float = 0.0
    size_w: float = 50.0
    size_h: float = 30.0
    spacing: float = 5.0
    active: bool = False

    def grid_points(self):
        pts = []
        if self.spacing <= 0:
            return pts
        rows = max(1, int(self.size_h // self.spacing) + 1)
        cols = max(1, int(self.size_w // self.spacing) + 1)
        serpentine = True
        for r in range(rows):
            row = []
            for c in range(cols):
                x = self.origin_x + c * self.spacing
                y = self.origin_y + r * self.spacing
                row.append((x,y))
            if serpentine and r % 2 == 1:
                row.reverse()
            pts.extend(row)
        return pts

@dataclass
class Robot:
    serial: str
    x: float = 0.0
    y: float = 0.0
    heading: float = 0.0
    battery: float = 100.0
    status: str = "idle"  # idle, moving, sampling, returning, paused
    next_goal: tuple = None
    visited: set = field(default_factory=set)

    def to_dict(self):
        return dict(serial=self.serial, x=self.x, y=self.y, heading=self.heading,
                    battery=round(self.battery,1), status=self.status,
                    next_goal=self.next_goal)

class SwarmSim:
    def __init__(self):
        self.mission = Mission()
        self.robots = {}
        self._lock = threading.Lock()
        self._thread = None
        self._running = False
        self._grid = []
        self._idx_assign = {}

    def register_robot(self, serial):
        with self._lock:
            if serial not in self.robots:
                r = Robot(serial=serial, x=0.0, y=0.0, heading=0.0)
                self.robots[serial] = r
                self._idx_assign[serial] = 0
        return True

    def set_mission(self, origin, size, spacing):
        with self._lock:
            self.mission.origin_x = float(origin["x"])
            self.mission.origin_y = float(origin["y"])
            self.mission.size_w = float(size["w"])
            self.mission.size_h = float(size["h"])
            self.mission.spacing = float(spacing)
            self._grid = self.mission.grid_points()
        return True

    def start(self):
        with self._lock:
            self.mission.active = True
        if not self._running:
            self._running = True
            self._thread = threading.Thread(target=self._loop, daemon=True)
            self._thread.start()

    def pause(self):
        with self._lock:
            for r in self.robots.values():
                r.status = "paused"
            self.mission.active = False

    def resume(self):
        with self._lock:
            for r in self.robots.values():
                if r.status == "paused":
                    r.status = "idle"
            self.mission.active = True

    def return_home(self, serial=None):
        with self._lock:
            targets = [self.robots[serial]] if serial and serial in self.robots else self.robots.values()
            for r in targets:
                r.status = "returning"
                r.next_goal = (0.0, 0.0)

    def _assign_next_goal(self, r: Robot):
        if not self._grid:
            r.next_goal = None
            r.status = "idle"
            return
        # assign sequentially per robot index
        idx = self._idx_assign.get(r.serial, 0)
        # find next unvisited
        n = len(self._grid)
        for k in range(n):
            g = self._grid[(idx + k) % n]
            if g not in r.visited:
                r.next_goal = g
                r.status = "moving"
                self._idx_assign[r.serial] = (idx + k + 1) % n
                return
        # all visited for this robot
        r.next_goal = None
        r.status = "idle"

    def _move_towards(self, r: Robot, goal, dt):
        if not goal:
            return
        gx, gy = goal
        dx = gx - r.x
        dy = gy - r.y
        dist = (dx*dx + dy*dy) ** 0.5
        speed = 2.0  # m/s
        if dist < 0.2:
            # sample
            r.status = "sampling"
            r.visited.add(goal)
            # small battery drop
            r.battery = max(0.0, r.battery - 0.1)
            time.sleep(0.05)
            r.status = "idle"
            r.next_goal = None
            return
        # move step
        vx = (dx / max(dist, 1e-6)) * speed
        vy = (dy / max(dist, 1e-6)) * speed
        r.x += vx * dt
        r.y += vy * dt
        r.heading = math.degrees(math.atan2(vy, vx))
        # battery drain
        r.battery = max(0.0, r.battery - 0.01)

    def _loop(self):
        last = time.time()
        while self._running:
            now = time.time()
            dt = now - last
            last = now
            with self._lock:
                if self.mission.active:
                    for r in self.robots.values():
                        if r.status in ("paused",):
                            continue
                        if r.status == "returning":
                            self._move_towards(r, (0.0, 0.0), dt)
                            if abs(r.x) < 0.2 and abs(r.y) < 0.2:
                                r.status = "idle"
                                r.next_goal = None
                            continue
                        if r.next_goal is None:
                            self._assign_next_goal(r)
                        self._move_towards(r, r.next_goal, dt)
                # small sleep outside to avoid lock holding too long
            time.sleep(0.05)

    def snapshot(self):
        with self._lock:
            return {
                "mission": {
                    "origin": {"x": self.mission.origin_x, "y": self.mission.origin_y},
                    "size": {"w": self.mission.size_w, "h": self.mission.size_h},
                    "spacing": self.mission.spacing,
                    "active": self.mission.active,
                },
                "robots": [r.to_dict() for r in self.robots.values()]
            }
