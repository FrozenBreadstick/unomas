const state = { packet: null };

const setText = (id, value) => {
  const el = document.getElementById(id);
  if (el) el.textContent = value;
};

const fmtNumber = (value, digits = 2) => {
  if (typeof value !== 'number' || !Number.isFinite(value)) return '—';
  return value.toFixed(digits);
};

const fmtBattery = (value) => {
  if (typeof value !== 'number' || !Number.isFinite(value)) return '—';
  return `${value}%`;
};

const fmtEmergency = (flag) => (flag ? 'ACTIVE' : 'Clear');

function render() {
  if (!state.packet) return;
  const packet = state.packet;

  setText('station-name', packet.name || '—');
  setText('current-state', packet.current_state || '—');
  setText('battery', fmtBattery(packet.battery));
  setText('emergency', fmtEmergency(packet.emergency));

  const current = packet.current_position || {};
  setText('current-x', fmtNumber(current.x));
  setText('current-y', fmtNumber(current.y));
  setText('current-z', fmtNumber(current.z));

  const target = packet.target_position || {};
  setText('target-x', fmtNumber(target.x));
  setText('target-y', fmtNumber(target.y));
  setText('target-z', fmtNumber(target.z));
}

async function refreshStatus() {
  try {
    const res = await fetch('/api/status', { cache: 'no-store' });
    if (!res.ok) throw new Error(`HTTP ${res.status}`);
    const data = await res.json();
    if (data && !data.error) {
      state.packet = data;
      render();
    } else if (data?.error) {
      throw new Error(data.error);
    }
  } catch (err) {
    console.error('Failed to fetch status packet:', err);
  }
}

function startPolling(intervalMs = 1500) {
  refreshStatus();
  setInterval(refreshStatus, intervalMs);
}

window.addEventListener('load', () => {
  startPolling();
});
