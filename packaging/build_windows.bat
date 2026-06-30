@echo off
setlocal
cd /d "%~dp0\.."
python -m venv .venv
call .venv\Scripts\activate
pip install --upgrade pip
pip install -r requirements.txt -r requirements-dev.txt
pyinstaller packaging\rotoforge_slicer.spec --noconfirm
echo Built: dist\RotoforgeSlicer.exe
