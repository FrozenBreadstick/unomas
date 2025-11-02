const state = { odometry: null };

const setText = (id, value) => {
  const el = document.getElementById(id);
  if (el) el.textContent = value;
};

const fmt = (n, digits = 3) => {
  if (typeof n !== 'number' || !Number.isFinite(n)) return '—';
  return n.toFixed(digits);
};

function render() {
  if (!state.odometry) return;
  const o = state.odometry;
  setText('odom-topic', o.topic || '/odom');
  setText('frame-id', o.frame_id || '—');
  setText('stamp', `${o.stamp?.sec ?? 0}.${String(o.stamp?.nanosec ?? 0).padStart(9, '0')}`);

  setText('pos-x', `${fmt(o.position?.x)}`);
  setText('pos-y', `${fmt(o.position?.y)}`);
  setText('pos-z', `${fmt(o.position?.z)}`);

  setText('orient-roll', `${fmt(o.orientation?.roll)}`);
  setText('orient-pitch', `${fmt(o.orientation?.pitch)}`);
  setText('orient-yaw', `${fmt(o.orientation?.yaw)}`);

  const lin = o.linear_velocity || {};
  const ang = o.angular_velocity || {};
  const linMag = Math.sqrt(
    Math.pow(lin.x || 0, 2) +
    Math.pow(lin.y || 0, 2) +
    Math.pow(lin.z || 0, 2)
  );
  const angMag = Math.sqrt(
    Math.pow(ang.x || 0, 2) +
    Math.pow(ang.y || 0, 2) +
    Math.pow(ang.z || 0, 2)
  );
  setText('vel-linear', fmt(linMag, 2));
  setText('vel-angular', fmt(angMag, 2));
}

async function loadInitial() {
  try {
    const res = await fetch('/api/odometry');
    if (!res.ok) throw new Error(`HTTP ${res.status}`);
    state.odometry = await res.json();
    render();
  } catch (err) {
    console.error('Failed to load initial odometry:', err);
  }
}

function setupEvents() {
  const ev = new EventSource('/events');
  ev.onmessage = (evt) => {
    try {
      const data = JSON.parse(evt.data);
      if (data.type === 'odometry') {
        state.odometry = data.payload;
        render();
      }
    } catch (err) {
      console.error('Bad SSE payload', err);
    }
  };
  ev.onerror = () => {
    console.warn('Lost connection to odometry stream, retrying automatically.');
  };
}

window.addEventListener('load', async () => {
  await loadInitial();
  setupEvents();
});
