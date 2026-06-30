#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
python3 -m venv .venv
# shellcheck disable=SC1091
source .venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt -r requirements-dev.txt
pyinstaller packaging/rotoforge_slicer.spec --noconfirm
echo "Built: dist/RotoforgeSlicer"
