#!/usr/bin/env python3
"""
05_train_bird.py — Fixed version
Writes augmentation params to hyp.yaml instead of CLI args
"""

import subprocess, sys, os, yaml
import matplotlib.pyplot as plt
import pandas as pd
from pathlib import Path

BASE_DIR    = Path.home() / "agri_robot_ml"
DATASET_DIR = BASE_DIR / "datasets"
YOLO_DIR    = BASE_DIR / "yolov5"
MODEL_DIR   = BASE_DIR / "models" / "bird"
GRAPH_DIR   = BASE_DIR / "graphs"

MODEL_DIR.mkdir(parents=True, exist_ok=True)
GRAPH_DIR.mkdir(parents=True, exist_ok=True)

BIRD_YAML = YOLO_DIR / "bird.yaml"
HYP_YAML  = YOLO_DIR / "data" / "hyps" / "hyp.bird.yaml"

print("=" * 60)
print("Bird Detection Training — YOLOv5n")
print("Binary: Bird / No_Bird + hard negatives (kites/planes)")
print("=" * 60)

if not BIRD_YAML.exists():
    print(f"bird.yaml not found at {BIRD_YAML}")
    print("Run 03_prepare_bird_dataset.py first")
    sys.exit(1)

# ─────────────────────────────────────────────────────────────
# Write hyp.yaml with all augmentation params
# ─────────────────────────────────────────────────────────────
HYP_YAML.parent.mkdir(parents=True, exist_ok=True)

hyp = {
    "lr0":            0.01,
    "lrf":            0.01,
    "momentum":       0.937,
    "weight_decay":   0.0005,
    "warmup_epochs":  3.0,
    "warmup_momentum":0.8,
    "warmup_bias_lr": 0.1,
    "box":            0.05,
    "cls":            0.5,
    "cls_pw":         1.0,
    "obj":            1.0,
    "obj_pw":         1.0,
    "iou_t":          0.20,
    "anchor_t":       4.0,
    "fl_gamma":       2.0,
    "hsv_h":          0.02,
    "hsv_s":          0.70,
    "hsv_v":          0.50,
    "degrees":        10.0,
    "translate":      0.10,
    "scale":          0.80,
    "shear":          0.0,
    "perspective":    0.0,
    "flipud":         0.10,
    "fliplr":         0.50,
    "mosaic":         1.0,
    "mixup":          0.15,
    "copy_paste":     0.0,
}

with open(HYP_YAML, "w") as f:
    yaml.dump(hyp, f, default_flow_style=False)
print(f"Hyperparameters written to {HYP_YAML}")

# ─────────────────────────────────────────────────────────────
# TRAIN — no augmentation CLI args, use hyp file instead
# ─────────────────────────────────────────────────────────────
print("\nStarting YOLOv5n training...")
print("Params: 640px | batch=32 | epochs=150 | patience=25")
print("-" * 60)

env = os.environ.copy()
env["GIT_PYTHON_GIT_EXECUTABLE"] = str(Path.home() / "git-local/usr/bin/git")
env["GIT_PYTHON_REFRESH"]        = "quiet"

cmd = [
    sys.executable, "train.py",
    "--data",           str(BIRD_YAML),
    "--hyp",            str(HYP_YAML),
    "--weights",        "yolov5n.pt",
    "--img",            "640",
    "--batch",          "32",
    "--epochs",         "150",
    "--patience",       "25",
    "--device",         "0",
    "--project",        str(MODEL_DIR),
    "--name",           "bird_yolov5n",
    "--save-period",    "10",
    "--cache",          "ram",
    "--workers",        "8",
    "--exist-ok",
    "--label-smoothing","0.05",
]

result = subprocess.run(cmd, cwd=str(YOLO_DIR), env=env)

if result.returncode != 0:
    print("\nTraining finished with warnings")
else:
    print("\nBird training complete!")

# ─────────────────────────────────────────────────────────────
# EVALUATE
# ─────────────────────────────────────────────────────────────
print("\n" + "=" * 60)
print("Evaluating Bird Model...")

results_csv = MODEL_DIR / "bird_yolov5n" / "results.csv"
if results_csv.exists():
    df = pd.read_csv(results_csv)
    df.columns = df.columns.str.strip()

    fig, axes = plt.subplots(1, 3, figsize=(15, 4))
    axes[0].plot(df["metrics/precision(B)"])
    axes[0].set_title("Precision"); axes[0].set_ylim(0, 1); axes[0].set_xlabel("Epoch")
    axes[1].plot(df["metrics/recall(B)"])
    axes[1].set_title("Recall");    axes[1].set_ylim(0, 1); axes[1].set_xlabel("Epoch")
    axes[2].plot(df["metrics/mAP50(B)"],    label="mAP@0.5")
    axes[2].plot(df["metrics/mAP50-95(B)"], label="mAP@0.5:0.95")
    axes[2].set_title("mAP"); axes[2].set_ylim(0, 1)
    axes[2].legend(); axes[2].set_xlabel("Epoch")
    plt.suptitle("Bird Model — Training Curves", fontsize=14)
    plt.tight_layout()
    graph_path = GRAPH_DIR / "bird_training_curves.png"
    plt.savefig(graph_path, dpi=150)
    print(f"Training curves saved: {graph_path}")
    print(f"\nBird Results:")
    print(f"   Best mAP@0.5  : {df['metrics/mAP50(B)'].max():.3f}")
    print(f"   Best Precision: {df['metrics/precision(B)'].max():.3f}")
    print(f"   Best Recall   : {df['metrics/recall(B)'].max():.3f}")
else:
    print(f"results.csv not found at {results_csv}")

weights_path = MODEL_DIR / "bird_yolov5n" / "weights" / "best.pt"
print(f"\nBird model saved: {weights_path}")
print("Run: python 06_export_tflite.py")
