#!/usr/bin/env python3
"""
check_plant.py
Run AFTER training the plant model.
Tests plant model on val set — checks detection accuracy
and single image visual test.
"""

import sys, cv2, random
from pathlib import Path
from PIL import Image

# Use non-interactive Agg backend — fixes Qt/xcb platform plugin crash
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import torch
import torch.nn as nn
import timm

# ─────────────────────────────────────────────────────────────
# PATHS
# ─────────────────────────────────────────────────────────────
BASE_DIR    = Path("/home/chandan/Desktop/model_make")
MODEL_PATH  = BASE_DIR / "models" / "plant" / "best_plant_model.pth"
DATASET_DIR = BASE_DIR / "images"
GRAPH_DIR   = BASE_DIR / "graphs"

GRAPH_DIR.mkdir(exist_ok=True)

print("=" * 60)
print("Plant Model — Detection Check (EfficientNet-B4)")
print("=" * 60)

if not MODEL_PATH.exists():
    print(f"ERROR: Model not found at {MODEL_PATH}")
    exit(1)

# ─────────────────────────────────────────────────────────────
# LOAD MODEL
# ─────────────────────────────────────────────────────────────
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print(f"Device: {device}")

checkpoint   = torch.load(MODEL_PATH, map_location=device)
state_dict   = checkpoint["model_state_dict"]
idx_to_class = checkpoint.get("idx_to_class", {0: "noplant", 1: "plant"})
num_classes  = len(idx_to_class)

model = timm.create_model("efficientnet_b4", num_classes=num_classes, pretrained=False)
model.load_state_dict(state_dict)
model.to(device)
model.eval()
print(f"Model loaded: {MODEL_PATH.name}")

CLASS_MAP = {
    int(k): ("plant" in str(v).lower() and "no" not in str(v).lower())
    for k, v in idx_to_class.items()
}

CONF_THRES = 0.40

# ─────────────────────────────────────────────────────────────
# ImageNet normalization constants
# ─────────────────────────────────────────────────────────────
MEAN_T = torch.tensor([0.485, 0.456, 0.406], dtype=torch.float32).view(3, 1, 1)
STD_T  = torch.tensor([0.229, 0.224, 0.225], dtype=torch.float32).view(3, 1, 1)


def pil_to_tensor(pil_img):
    """
    PIL RGB → normalised float32 torch tensor [3, H, W].
    Uses torch.frombuffer on raw PIL bytes — zero numpy involvement.
    """
    pil_img = pil_img.convert("RGB")
    raw     = bytearray(pil_img.tobytes())
    t = torch.frombuffer(raw, dtype=torch.uint8).clone()
    H, W = pil_img.height, pil_img.width
    t = t.reshape(H, W, 3).permute(2, 0, 1)
    return t.float() / 255.0


# ─────────────────────────────────────────────────────────────
# INFERENCE FUNCTION
# ─────────────────────────────────────────────────────────────
def predict(img_path):
    img_bgr = cv2.imread(str(img_path))
    if img_bgr is None:
        return None, 0.0, None

    img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
    pil_img = Image.fromarray(img_rgb).resize((224, 224), Image.BILINEAR)

    t      = pil_to_tensor(pil_img)
    tensor = ((t - MEAN_T) / STD_T).unsqueeze(0).to(device)

    with torch.no_grad():
        logits = model(tensor)
        probs  = torch.softmax(logits, dim=1)[0]

    pred_idx  = probs.argmax().item()
    pred_conf = probs[pred_idx].item()
    detected  = CLASS_MAP[pred_idx] and pred_conf >= CONF_THRES

    return detected, pred_conf, img_bgr


def draw_result(img_bgr, filename, conf):
    """Overlay filename + confidence on image; return PIL RGB."""
    img   = img_bgr.copy()
    label = f"{filename}  {conf*100:.0f}%"
    cv2.putText(img, label, (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
    return Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))


# ─────────────────────────────────────────────────────────────
# [1] COLLECT IMAGES
# ─────────────────────────────────────────────────────────────
print("\n" + "=" * 60)
print("VAL SET ACCURACY CHECK")
print("=" * 60)

if not DATASET_DIR.exists():
    print(f"Dataset folder not found: {DATASET_DIR}")
    exit(1)

all_images = [
    p for ext in ("*.jpg", "*.jpeg", "*.png")
    for p in DATASET_DIR.glob(ext)
]
print(f"Total images found        : {len(all_images)}")

val_images = [p for p in all_images if "maize" in p.stem.lower()]
print(f"Files with 'maize' in name: {len(val_images)}")

if len(val_images) == 0:
    print("\nWARNING: No files with 'maize' in the filename found.")
    print("Listing all filenames for inspection (first 30):")
    for p in sorted(all_images)[:30]:
        print(f"  {p.name}")
    exit(1)
else:
    print("\nMatched files:")
    for p in sorted(val_images):
        print(f"  {p.name}")

# ─────────────────────────────────────────────────────────────
# [2] RUN INFERENCE
# ─────────────────────────────────────────────────────────────
results = []

print(f"\nRunning inference (conf threshold: {CONF_THRES})...")
for img_path in val_images:
    detected, conf, img_bgr = predict(img_path)
    if detected is None:
        print(f"  SKIP (unreadable): {img_path.name}")
        continue
    results.append((img_path, detected, conf, img_bgr))
    print(f"  {img_path.stem:<30}  →  {conf*100:.0f}%  ({'PLANT' if detected else 'NO PLANT'})")

print(f"\nTotal processed : {len(results)}")
print(f"  Detected PLANT   : {sum(1 for r in results if r[1])}")
print(f"  Detected NO PLANT: {sum(1 for r in results if not r[1])}")

# ─────────────────────────────────────────────────────────────
# [3] SAMPLE VISUAL GRID  — labels = filename + confidence
# ─────────────────────────────────────────────────────────────
print("\n" + "=" * 60)
print("SINGLE IMAGE VISUAL TEST")
print("=" * 60)

test_samples = random.sample(results, min(6, len(results)))

n = len(test_samples)
ncols = min(n, 3)
nrows = (n + ncols - 1) // ncols  # only as many rows as needed

fig, axes = plt.subplots(nrows, ncols, figsize=(6 * ncols, 6 * nrows + 1))
axes = axes.flatten() if hasattr(axes, "flatten") else [axes]

for idx, (img_path, detected, conf, img_bgr) in enumerate(test_samples):
    pil_display = draw_result(img_bgr, img_path.stem, conf)
    pil_display = pil_display.resize((320, 320), Image.BILINEAR)
    axes[idx].imshow(pil_display)
    axes[idx].set_title(
        f"{img_path.stem}\n{conf*100:.0f}%",
        fontsize=14, fontweight="bold"
    )
    axes[idx].axis("off")

for idx in range(len(test_samples), len(axes)):
    axes[idx].axis("off")

plt.suptitle("Plant Model — Sample Predictions", fontsize=22, fontweight="bold", y=1.01)
plt.tight_layout(rect=[0, 0, 1, 1])
out = GRAPH_DIR / "plant_sample_predictions.png"
plt.savefig(out, dpi=150, bbox_inches="tight")
plt.close()
print(f"Sample predictions saved : {out}")

# ─────────────────────────────────────────────────────────────
# [4] CONFIDENCE DISTRIBUTION
# ─────────────────────────────────────────────────────────────
detected_confs = [r[2] for r in results if r[1]]
nodetect_confs = [r[2] for r in results if not r[1]]

if detected_confs or nodetect_confs:
    fig, ax = plt.subplots(figsize=(10, 4))
    if detected_confs:
        ax.hist(detected_confs, bins=20, alpha=0.7, color="green",
                label=f"Detected PLANT ({len(detected_confs)})")
    if nodetect_confs:
        ax.hist(nodetect_confs, bins=20, alpha=0.7, color="red",
                label=f"Detected NO PLANT ({len(nodetect_confs)})")
    ax.axvline(CONF_THRES, color="black", linestyle="--",
               label=f"{CONF_THRES} threshold")
    ax.set_xlabel("Confidence Score")
    ax.set_ylabel("Count")
    ax.set_title("Plant Detection Confidence Distribution")
    ax.legend()
    plt.tight_layout()
    conf_out = GRAPH_DIR / "plant_confidence_distribution.png"
    plt.savefig(conf_out, dpi=150)
    plt.close()
    print(f"Confidence distribution saved: {conf_out}")

# ─────────────────────────────────────────────────────────────
# VERDICT
# ─────────────────────────────────────────────────────────────
print("\n" + "=" * 60)
print("VERDICT")
print("=" * 60)
total     = len(results)
n_plant   = sum(1 for r in results if r[1])
n_noplant = total - n_plant
print(f"\n  Total maize images tested : {total}")
print(f"  Predicted PLANT           : {n_plant}")
print(f"  Predicted NO PLANT        : {n_noplant}")
print("=" * 60)
print(f"\nAll graphs saved to: {GRAPH_DIR}")