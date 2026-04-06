#!/usr/bin/env python3


import sys, cv2, random
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from pathlib import Path
from PIL import Image

BASE_DIR     = Path("/home/chandan/Desktop/model_make")
YOLO_DIR    = BASE_DIR / "yolov5"
MODEL_DIR   = BASE_DIR / "models" / "bird" /'bird_yolov5n'/ "weights"
DATASET_DIR = BASE_DIR / "images"
GRAPH_DIR   = BASE_DIR / "graphs"

GRAPH_DIR.mkdir(exist_ok=True)

BEST_PT = MODEL_DIR / "best.pt"

print("=" * 60)
print("Bird Model — Detection Check")
print("=" * 60)

if not BEST_PT.exists():
    print(f"ERROR: Model not found at {BEST_PT}")
    print("Run 05_train_bird.py first")
    exit(1)

sys.path.insert(0, str(YOLO_DIR))

import torch
from models.experimental import attempt_load
from utils.general import non_max_suppression, scale_boxes
from utils.augmentations import letterbox

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print(f"Device: {device}")

model = attempt_load(BEST_PT, device=device)
model.eval()
print(f"Model loaded: {BEST_PT.name}")

IMG_SIZE   = 640
CONF_THRES = 0.40
IOU_THRES  = 0.45

# ─────────────────────────────────────────────────────────────
# Inference function
# ─────────────────────────────────────────────────────────────
def predict(img_path):
    img0 = cv2.imread(str(img_path))
    if img0 is None:
        return None, 0.0, [], None

    img, ratio, pad = letterbox(img0, IMG_SIZE, stride=32, auto=True)
    img_tensor = img.transpose((2, 0, 1))[::-1].copy()
    img_tensor = torch.from_numpy(img_tensor).to(device).float() / 255.0
    if img_tensor.ndimension() == 3:
        img_tensor = img_tensor.unsqueeze(0)

    with torch.no_grad():
        pred = model(img_tensor)[0]

    pred = non_max_suppression(pred, CONF_THRES, IOU_THRES)
    detections = pred[0]

    boxes = []
    if detections is not None and len(detections):
        detections[:, :4] = scale_boxes(img_tensor.shape[2:], detections[:, :4], img0.shape).round()
        bird_dets = detections[detections[:, 5] == 0]
        if len(bird_dets):
            conf = bird_dets[:, 4].max().item()
            for *xyxy, c, cls in bird_dets.tolist():
                boxes.append([int(xyxy[0]), int(xyxy[1]), int(xyxy[2]), int(xyxy[3])])
            return True, conf, boxes, img0

    return False, 0.0, [], img0


def draw_detections(img_bgr, boxes, conf, has_bird, detected):
    img = img_bgr.copy()
    for (x1, y1, x2, y2) in boxes:
        color = (0, 255, 0) if has_bird else (255, 0, 0)
        cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
        label = f"bird {conf*100:.0f}%"
        cv2.putText(img, label, (x1, max(y1 - 6, 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
    return cv2.cvtColor(img, cv2.COLOR_BGR2RGB)



print("\n" + "=" * 60)
print("VAL SET ACCURACY CHECK")
print("=" * 60)

val_bird_dir  = DATASET_DIR 
val_label_dir = DATASET_DIR 

if not val_bird_dir.exists():
    print(f"Val images not found at {val_bird_dir}")
    exit(1)

all_val_images = [
    p for ext in ("*.jpg", "*.jpeg", "*.png")
    for p in val_bird_dir.glob(ext)
]
print(f"Total val images found    : {len(all_val_images)}")

val_images = []   # list of (path, has_bird)
skipped    = 0

for p in all_val_images:
    name = p.stem.lower()
    if "nobird" in name:
        val_images.append((p, False))
    elif "bird" in name:
        val_images.append((p, True))
    else:
        skipped += 1

print(f"Files labelled as BIRD    : {sum(1 for _, h in val_images if h)}")
print(f"Files labelled as NO BIRD : {sum(1 for _, h in val_images if not h)}")
print(f"Files skipped (no match)  : {skipped}")

if len(val_images) == 0:
    print("\nWARNING: No files with 'bird' or 'nobird' in filename found.")
    print("Listing all filenames for inspection:")
    for p in sorted(all_val_images)[:30]:
        print(f"  {p.name}")
    exit(1)
else:
    print("\nMatched files:")
    for p, h in sorted(val_images):
        print(f"  {'[BIRD]  ' if h else '[NOBIRD]'}  {p.name}")
# ─────────────────────────────────────────────────────────────

tp = fp = tn = fn = 0
results = []

print("\nRunning inference on filtered val set...")
for img_path, has_bird in val_images:          # has_bird comes from filename now
    detected, conf, boxes, img_bgr = predict(img_path)
    if detected is None:
        print(f"  SKIP (unreadable): {img_path.name}")
        continue
    results.append((img_path, has_bird, detected, conf, boxes, img_bgr))
    if has_bird and detected:       tp += 1
    elif has_bird and not detected: fn += 1
    elif not has_bird and detected: fp += 1
    else:                           tn += 1

total     = tp + fp + tn + fn
precision = tp / (tp + fp + 1e-8)
recall    = tp / (tp + fn + 1e-8)
f1        = 2 * precision * recall / (precision + recall + 1e-8)
accuracy  = (tp + tn) / (total + 1e-8)

print(f"\nResults on {total} val images (bird-named only):")
print(f"  True Positives  (bird detected correctly) : {tp}")
print(f"  True Negatives  (no bird, not detected)   : {tn}")
print(f"  False Positives (no bird but detected)    : {fp}")
print(f"  False Negatives (bird missed)             : {fn}")
print(f"\n  Accuracy  : {accuracy*100:.1f}%")
print(f"  Precision : {precision*100:.1f}%")
print(f"  Recall    : {recall*100:.1f}%")
print(f"  F1 Score  : {f1*100:.1f}%")

# ─────────────────────────────────────────────────────────────
# [2] HARD NEGATIVE CHECK — kites and planes
# ─────────────────────────────────────────────────────────────
print("\n" + "=" * 60)
print("HARD NEGATIVE CHECK — Kites and Planes")
print("=" * 60)

hard_neg_dir = DATASET_DIR.parent / "hard_negatives_bird"
if hard_neg_dir.exists():
    hard_imgs  = [
        p for ext in ("*.jpg", "*.jpeg", "*.png")
        for p in hard_neg_dir.glob(ext)
    ][:50]
    false_alarms = 0
    for img_path in hard_imgs:
        detected, conf, _, _ = predict(img_path)
        if detected:
            false_alarms += 1
    fa_rate = false_alarms / len(hard_imgs) * 100
    print(f"\n  Hard negatives tested : {len(hard_imgs)}")
    print(f"  False alarms          : {false_alarms}")
    print(f"  False alarm rate      : {fa_rate:.1f}%")
    if fa_rate < 10:
        print("  OK — kites/planes rarely confused as birds")
    elif fa_rate < 20:
        print("  ACCEPTABLE — some kite confusion but manageable")
    else:
        print("  HIGH — model confusing kites/planes with birds")
else:
    print("  Hard negatives folder not found — skipping")

# ─────────────────────────────────────────────────────────────
# [3] SINGLE IMAGE VISUAL TEST
# ─────────────────────────────────────────────────────────────
print("\n" + "=" * 60)
print("SINGLE IMAGE VISUAL TEST")
print("=" * 60)

bird_samples   = [r for r in results if r[1]][:3]
nobird_samples = [r for r in results if not r[1]][:3]
test_samples   = bird_samples + nobird_samples
random.shuffle(test_samples)
test_samples   = test_samples[:6]

fig, axes = plt.subplots(2, 3, figsize=(15, 8))
axes = axes.flatten()

for idx, (img_path, has_bird, detected, conf, boxes, img_bgr) in enumerate(test_samples):
    display_img = draw_detections(img_bgr, boxes, conf, has_bird, detected)
    display_img = cv2.resize(display_img, (320, 320))
    axes[idx].imshow(display_img)
    true_label  = "BIRD" if has_bird else "NO BIRD"
    pred_label  = f"DETECTED ({conf*100:.0f}%)" if detected else "NOT DETECTED"
    correct     = (has_bird == detected)
    axes[idx].set_title(
        f"True: {true_label}\nPred: {pred_label}",
        color="green" if correct else "red", fontsize=9, fontweight="bold"
    )
    axes[idx].axis("off")

for idx in range(len(test_samples), len(axes)):
    axes[idx].axis("off")

plt.suptitle("Bird Model — Sample Predictions (Green=Correct, Red=Wrong)", fontsize=12)
plt.tight_layout()
out = GRAPH_DIR / "bird_sample_predictions.png"
plt.savefig(out, dpi=150)
print(f"Sample predictions saved: {out}")
plt.show()

# ─────────────────────────────────────────────────────────────
# [4] CONFIDENCE DISTRIBUTION
# ─────────────────────────────────────────────────────────────
bird_confs   = [r[3] for r in results if r[1] and r[2]]
nobird_confs = [r[3] for r in results if not r[1] and r[2]]

if bird_confs or nobird_confs:
    fig, ax = plt.subplots(figsize=(10, 4))
    if bird_confs:
        ax.hist(bird_confs,   bins=20, alpha=0.7, color="green", label=f"True Birds ({len(bird_confs)})")
    if nobird_confs:
        ax.hist(nobird_confs, bins=20, alpha=0.7, color="red",   label=f"False Positives ({len(nobird_confs)})")
    ax.axvline(0.5, color="black", linestyle="--", label="0.5 threshold")
    ax.set_xlabel("Confidence Score")
    ax.set_ylabel("Count")
    ax.set_title("Bird Detection Confidence Distribution")
    ax.legend()
    plt.tight_layout()
    conf_out = GRAPH_DIR / "bird_confidence_distribution.png"
    plt.savefig(conf_out, dpi=150)
    print(f"Confidence distribution saved: {conf_out}")
    plt.show()

# ─────────────────────────────────────────────────────────────
# VERDICT
# ─────────────────────────────────────────────────────────────
print("\n" + "=" * 60)
print("VERDICT")
print("=" * 60)
print(f"\n  Accuracy  : {accuracy*100:.1f}%")
print(f"  Precision : {precision*100:.1f}%")
print(f"  Recall    : {recall*100:.1f}%")
print(f"  F1 Score  : {f1*100:.1f}%")

if recall * 100 >= 80 and accuracy * 100 >= 75:
    print(f"\n  MODEL IS GOOD FOR DEPLOYMENT")
    print(f"  ACTION: Copy deploy/ folder to Pi4 via USB")
elif recall * 100 < 70:
    print(f"\n  RECALL TOO LOW — will miss many birds")
    print(f"  ACTION: Add more bird field photos and retrain")
else:
    print(f"\n  ACCEPTABLE — deploy and improve with field data later")
    print(f"  ACTION: Copy deploy/ folder to Pi4 via USB")
print("=" * 60)