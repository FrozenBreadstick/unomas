#!/usr/bin/env bash
set -euo pipefail

# --- Find nearest workspace (looks upward for install/setup.bash) ---
find_workspace_setup() {
  local dir="$PWD"
  while [ "$dir" != "/" ]; do
    if [ -f "$dir/install/setup.bash" ]; then
      echo "$dir/install/setup.bash"
      return 0
    fi
    dir="$(dirname "$dir")"
  done
  return 1
}

# --- 1) Source ROS 2 Humble (temporarily disable -u) ---
if [ -f /opt/ros/humble/setup.bash ]; then
  set +u
  # shellcheck source=/dev/null
  source /opt/ros/humble/setup.bash
  set -u
else
  echo "❌ ROS2 Humble not found at /opt/ros/humble/setup.bash"
  exit 1
fi

# --- 2) Source the current workspace automatically (also with -u disabled) ---
if WS_SETUP="$(find_workspace_setup)"; then
  echo "🔹 Using workspace: $(dirname "$(dirname "$WS_SETUP")")"
  set +u
  # shellcheck source=/dev/null
  source "$WS_SETUP"
  set -u
else
  echo "⚠️  No workspace install/setup.bash found above this directory (continuing)."
fi

# --- 3) Create & activate venv; install deps ---
if [ ! -d "venv" ]; then
  python3 -m venv venv
fi
# shellcheck source=/dev/null
source venv/bin/activate
python -m pip install -U pip wheel >/dev/null
pip install -r requirements.txt

# --- 4) Run the Flask app ---
export SIM_MODE=1
export FLASK_ENV=development
echo "🚀 Launching Flask app..."
exec python app.py
