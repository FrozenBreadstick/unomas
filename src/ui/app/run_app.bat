@echo off
setlocal
if not exist venv (
  py -3 -m venv venv
)
call venv\Scripts\activate
python -m pip install --upgrade pip >nul
pip install -r requirements.txt
set SIM_MODE=1
set FLASK_ENV=development
python app.py
