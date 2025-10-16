#!/usr/bin/env bash
set -e
if [ ! -d "venv" ]; then
  python3 -m venv venv
fi
source venv/bin/activate
python -m pip install --upgrade pip >/dev/null
pip install -r requirements.txt
export SIM_MODE=1
export FLASK_ENV=development
python app.py
