#!/usr/bin/env python3
"""
06_export_tflite.py
Exports Bird (YOLOv5n) model to TFLite INT8 for Pi4 deployment.
Plant model (EfficientNetB4 .pth) is kept as PyTorch for Pi4 inference.

Note: Plant model uses torch on Pi4 directly (not TFLite).
Bird model uses TFLite INT8 via YOLOv5 export (faster detection).
"""

import subprocess, sys, shutil
from pathlib import Path

BASE_DIR  = Path.home() / "agri_robot_ml"
YOLO_DIR  = BASE_DIR / "yolov5"
MODEL_DIR = BASE_DIR / "models"
DEPLOY    = BASE_DIR / "deploy"
DEPLOY.mkdir(parents=True, exist_ok=True)

print("=" * 60)
print("Export Models for Pi4 Deployment")
print("=" * 60)

# ─────────────────────────────────────────────────────────────
# BIRD — Export YOLOv5n to TFLite INT8
# ─────────────────────────────────────────────────────────────
bird_pt = MODEL_DIR / "bird" / "bird_yolov5n" / "weights" / "best.pt"

if not bird_pt.exists():
    print(f" Bird model not found: {bird_pt}")
    print("Run 05_train_bird.py first")
else:
    print(f"\n Exporting Bird model to TFLite INT8...")
    print(f"   Source: {bird_pt}")

    cmd = [
        sys.executable, "export.py",
        "--weights", str(bird_pt),
        "--include", "tflite",
        "--img",     "640",
        "--int8",
        "--device",  "0",
    ]
    result = subprocess.run(cmd, cwd=str(YOLO_DIR))

    # Find the exported TFLite file
    tflite_candidates = [
        bird_pt.parent / "best-int8.tflite",
        bird_pt.parent / "best_saved_model" / "best_int8.tflite",
        bird_pt.parent / "best-fp16.tflite",
    ]

    tflite_found = None
    for candidate in tflite_candidates:
        if candidate.exists():
            tflite_found = candidate
            break

    if tflite_found:
        dst = DEPLOY / "bird_model.tflite"
        shutil.copy(tflite_found, dst)
        size_mb = dst.stat().st_size / 1024 / 1024
        print(f" Bird TFLite: {dst} ({size_mb:.1f} MB)")
    else:
        print(" TFLite export failed. Trying FP16...")
        cmd[-1] = "onnx"  # fallback to ONNX
        subprocess.run(cmd, cwd=str(YOLO_DIR))
        print("   Use best.pt directly on Pi4 with PyTorch (slower but works)")
        shutil.copy(bird_pt, DEPLOY / "bird_model.pt")

# ─────────────────────────────────────────────────────────────
# PLANT — Copy PyTorch model + class index
# ─────────────────────────────────────────────────────────────
plant_pt = MODEL_DIR / "plant" / "best_plant_model.pth"
hardfix  = MODEL_DIR / "plant" / "best_plant_model_hardfix.pth"

# Prefer hard-fixed model if exists
source_plant = hardfix if hardfix.exists() else plant_pt

if source_plant.exists():
    shutil.copy(source_plant, DEPLOY / "plant_model.pth")
    size_mb = source_plant.stat().st_size / 1024 / 1024
    print(f"\n Plant model: {DEPLOY / 'plant_model.pth'} ({size_mb:.1f} MB)")
else:
    print(f"\n Plant model not found: {plant_pt}")
    print("Run 04_train_plant.py first")

class_json = BASE_DIR / "plant_class_index.json"
if class_json.exists():
    shutil.copy(class_json, DEPLOY / "plant_class_index.json")
    print(f" Class index: {DEPLOY / 'plant_class_index.json'}")

# ─────────────────────────────────────────────────────────────
# DEPLOY FOLDER SUMMARY
# ─────────────────────────────────────────────────────────────
print("\n" + "=" * 60)
print("DEPLOY folder — copy this to Pi4:")
print(f"  {DEPLOY}")
print()
for f in sorted(DEPLOY.iterdir()):
    size_mb = f.stat().st_size / 1024 / 1024
    print(f"  {f.name:40s} {size_mb:.1f} MB")

print("""
TRANSFER TO Pi4:
  scp -r ~/agri_robot_ml/deploy/ ubuntu@<PI4_IP>:/home/ubuntu/agri_models/

THEN on Pi4:
  Run: python 07_inference_pi4.py
""")
