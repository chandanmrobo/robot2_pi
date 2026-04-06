#!/usr/bin/env python3
"""
00_install.py
"""

import subprocess, sys, os
from pathlib import Path

BASE_DIR = Path.home() / "agri_robot_ml"
BASE_DIR.mkdir(parents=True, exist_ok=True)

YOLO_DIR = BASE_DIR / "yolov5"

def run(cmd):
    print(f"  $ {cmd}")
    result = subprocess.run(cmd, shell=True)
    if result.returncode != 0:
        print(f"  WARNING: command returned {result.returncode}")

print("=" * 60)
print("Agricultural Robot ML — Dependency Setup")
print("=" * 60)

# Core packages
print("\n[1/3] Installing Python packages...")
run(f"{sys.executable} -m pip install -q torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu121")
run(f"{sys.executable} -m pip install -q timm albumentations kaggle seaborn scikit-learn fiftyone opencv-python tqdm Pillow pandas matplotlib")

# YOLOv5
print("\n[2/3] Cloning YOLOv5...")
if not YOLO_DIR.exists():
    run(f"git clone https://github.com/ultralytics/yolov5 {YOLO_DIR}")
    run(f"{sys.executable} -m pip install -q -r {YOLO_DIR}/requirements.txt")
else:
    print("  YOLOv5 already cloned, skipping.")

# Kaggle setup
print("\n[3/3] Setting up Kaggle credentials...")
import json
kaggle_dir = Path.home() / ".kaggle"
kaggle_dir.mkdir(exist_ok=True)
kaggle_json = kaggle_dir / "kaggle.json"

# Use your API token
KAGGLE_USERNAME = "tvhome2"  
KAGGLE_KEY      = "KGAT_5a514e65e65b5894ed8dd24dcf3b4bf8"

if not kaggle_json.exists():
    with open(kaggle_json, "w") as f:
        json.dump({"username": KAGGLE_USERNAME, "key": KAGGLE_KEY}, f)
    os.chmod(kaggle_json, 0o600)
    print("  Kaggle credentials saved.")
else:
    print("  Kaggle credentials already exist, skipping.")

# GPU check
print("\n[GPU CHECK]")
import torch
if torch.cuda.is_available():
    print(f"   GPU: {torch.cuda.get_device_name(0)}")
    print(f"     VRAM: {torch.cuda.get_device_properties(0).total_memory / 1e9:.1f} GB")
    print(f"     CUDA: {torch.version.cuda}")
else:
    print("   No GPU detected — check CUDA install")

print("\n Setup complete. Run 01_download_datasets.py next.")
