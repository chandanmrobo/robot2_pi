#!/usr/bin/env python3
"""
check_confusion.py
Run AFTER 04_train_plant.py finishes.
Uses best_plant_model.pth (NOT hardfix — hardfix caused catastrophic forgetting)
"""

import torch, timm, json
import torch.nn.functional as F
import albumentations as A
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from albumentations.pytorch import ToTensorV2
from torch.utils.data import Dataset, DataLoader
from sklearn.metrics import confusion_matrix
from pathlib import Path
from PIL import Image
import random

# ─────────────────────────────────────────────────────────────
# PATHS — always use original model, never hardfix
# ─────────────────────────────────────────────────────────────
BASE_DIR    = Path.home() / "agri_robot_ml"
DATASET_DIR = BASE_DIR / "datasets"
PLANT_TEST  = DATASET_DIR / "plant_split" / "test"
MODEL_DIR   = BASE_DIR / "models" / "plant"
GRAPH_DIR   = BASE_DIR / "graphs"
CLASS_JSON  = BASE_DIR / "plant_class_index.json"

GRAPH_DIR.mkdir(parents=True, exist_ok=True)

# Force original model — hardfix caused catastrophic forgetting
MODEL_PATH = MODEL_DIR / "best_plant_model.pth"

print("=" * 60)
print("Plant Model — Confusion Check")
print(f"Model: {MODEL_PATH.name}")
print("=" * 60)

if not MODEL_PATH.exists():
    print(f"ERROR: Model not found at {MODEL_PATH}")
    print("Run 04_train_plant.py first")
    exit(1)

# ─────────────────────────────────────────────────────────────
# Load model + classes
# ─────────────────────────────────────────────────────────────
with open(CLASS_JSON) as f:
    data = json.load(f)
CLASS_TO_IDX  = data["class_to_idx"]
IDX_TO_CLASS  = data["idx_to_class"]
final_classes = sorted(CLASS_TO_IDX.keys())
NUM_CLASSES   = len(final_classes)

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print(f"Device: {device} | Classes: {NUM_CLASSES}")

ckpt  = torch.load(MODEL_PATH, map_location=device, weights_only=False)
model = timm.create_model("efficientnet_b4", pretrained=False, num_classes=NUM_CLASSES)
model.load_state_dict(ckpt["model_state_dict"])
model = model.to(device)
model.eval()
print(f"Model loaded | Val acc from training: {ckpt.get('val_acc', 0)*100:.2f}%")

# ─────────────────────────────────────────────────────────────
# Transform
# ─────────────────────────────────────────────────────────────
tf = A.Compose([
    A.SmallestMaxSize(256),
    A.CenterCrop(224, 224),
    A.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
    ToTensorV2(),
])

# ─────────────────────────────────────────────────────────────
# Dataset
# ─────────────────────────────────────────────────────────────
class TestDS(Dataset):
    def __init__(self, root):
        self.samples = []
        for cls_dir in Path(root).iterdir():
            if not cls_dir.is_dir() or cls_dir.name not in CLASS_TO_IDX:
                continue
            label = CLASS_TO_IDX[cls_dir.name]
            for img in cls_dir.iterdir():
                if img.suffix.lower() in [".jpg", ".jpeg", ".png"]:
                    self.samples.append((img, label))
    def __len__(self): return len(self.samples)
    def __getitem__(self, idx):
        path, label = self.samples[idx]
        img = np.array(Image.open(path).convert("RGB"))
        return tf(image=img)["image"], label

test_ds = TestDS(PLANT_TEST)
test_dl = DataLoader(test_ds, batch_size=32, shuffle=False, num_workers=4)
print(f"Test images: {len(test_ds)}")

# ─────────────────────────────────────────────────────────────
# Run inference on full test set
# ─────────────────────────────────────────────────────────────
print("\nRunning inference on test set...")
all_preds  = []
all_labels = []
all_confs  = []

with torch.no_grad():
    for imgs, labels in test_dl:
        imgs = imgs.to(device)
        probs = F.softmax(model(imgs), dim=1)
        preds = probs.argmax(1)
        confs = probs.max(1).values
        all_preds.extend(preds.cpu().numpy())
        all_labels.extend(labels.numpy())
        all_confs.extend(confs.cpu().numpy())

all_preds  = np.array(all_preds)
all_labels = np.array(all_labels)
all_confs  = np.array(all_confs)

test_acc = (all_preds == all_labels).mean()
print(f"\nOverall Test Accuracy : {test_acc*100:.2f}%")
print(f"Average Confidence    : {all_confs.mean()*100:.1f}%")

# ─────────────────────────────────────────────────────────────
# [1] CORN SPECIFIC CONFUSION CHECK
# ─────────────────────────────────────────────────────────────
print("\n" + "=" * 60)
print("CORN DISEASE CONFUSION CHECK")
print("=" * 60)

corn_classes = [c for c in final_classes if c.startswith("Corn")]
corn_indices = [CLASS_TO_IDX[c] for c in corn_classes]

corn_mask  = np.isin(all_labels, corn_indices)
corn_preds = all_preds[corn_mask]
corn_true  = all_labels[corn_mask]
corn_confs = all_confs[corn_mask]

corn_acc = (corn_preds == corn_true).mean()
print(f"\nCorn Accuracy: {corn_acc*100:.2f}% ({corn_mask.sum()} test images)")

cm_corn      = confusion_matrix(corn_true, corn_preds, labels=corn_indices)
cm_corn_norm = cm_corn.astype("float") / (cm_corn.sum(axis=1, keepdims=True) + 1e-8)

print("\nCorn Confusion Matrix (%):")
print(f"{'':30s}", end="")
for c in corn_classes:
    print(f"{c.replace('Corn_',''):18s}", end="")
print()
for i, true_cls in enumerate(corn_classes):
    print(f"{true_cls:30s}", end="")
    for j in range(len(corn_classes)):
        val    = cm_corn_norm[i, j] * 100
        marker = " !!!" if (i != j and val > 15) else ("  <<" if i == j else "")
        print(f"{val:6.1f}%{marker:11s}", end="")
    print()

# Check if 04b needed
blight_idx = CLASS_TO_IDX.get("Corn_NorthernBlight")
gray_idx   = CLASS_TO_IDX.get("Corn_GrayLeafSpot")
need_04b   = False

if blight_idx and gray_idx:
    bi = corn_indices.index(blight_idx)
    gi = corn_indices.index(gray_idx)
    blight_as_gray = cm_corn_norm[bi, gi] * 100
    gray_as_blight = cm_corn_norm[gi, bi] * 100
    print(f"\nKey confusion pair:")
    print(f"  NorthernBlight → GrayLeafSpot : {blight_as_gray:.1f}%")
    print(f"  GrayLeafSpot   → NorthernBlight: {gray_as_blight:.1f}%")
    if blight_as_gray > 15 or gray_as_blight > 15:
        need_04b = True
        print("  !!! ABOVE 15%")
    else:
        print("  OK — Below 15%, no fix needed")

# ─────────────────────────────────────────────────────────────
# [2] TOP CONFUSED PAIRS
# ─────────────────────────────────────────────────────────────
print("\n" + "=" * 60)
print("TOP CONFUSED PAIRS — ALL DISEASES")
print("=" * 60)

cm_full      = confusion_matrix(all_labels, all_preds)
cm_full_norm = cm_full.astype("float") / (cm_full.sum(axis=1, keepdims=True) + 1e-8)
np.fill_diagonal(cm_full_norm, 0)

pairs = []
for i in range(NUM_CLASSES):
    for j in range(NUM_CLASSES):
        if cm_full_norm[i, j] > 0.05:
            pairs.append((cm_full_norm[i, j]*100, final_classes[i], final_classes[j]))

pairs.sort(reverse=True)
if pairs:
    print(f"\n  {'True Label':40s} {'Predicted As':40s} {'%':8s}")
    print("  " + "-" * 92)
    for score, true_cls, pred_cls in pairs[:15]:
        flag = "  !!!" if score > 15 else ""
        print(f"  {true_cls:40s} {pred_cls:40s} {score:5.1f}%{flag}")
else:
    print("\n  No confusion pairs above 5% — model is clean!")

# ─────────────────────────────────────────────────────────────
# [3] PER CLASS ACCURACY
# ─────────────────────────────────────────────────────────────
print("\n" + "=" * 60)
print("PER CLASS ACCURACY")
print("=" * 60)
print(f"\n  {'Class':45s} {'Accuracy':10s} {'AvgConf':10s} {'Samples':8s}")
print("  " + "-" * 75)

for cls_name in sorted(final_classes):
    idx  = CLASS_TO_IDX[cls_name]
    mask = all_labels == idx
    if mask.sum() == 0:
        continue
    acc      = (all_preds[mask] == idx).mean() * 100
    avg_conf = all_confs[mask].mean() * 100
    n        = mask.sum()
    flag     = "  <-- LOW" if acc < 80 else ""
    print(f"  {cls_name:45s} {acc:6.1f}%   {avg_conf:6.1f}%   {n:5d}{flag}")

# ─────────────────────────────────────────────────────────────
# [4] SAVE CORN CONFUSION MATRIX IMAGE
# ─────────────────────────────────────────────────────────────
fig, ax = plt.subplots(figsize=(8, 6))
sns.heatmap(
    cm_corn_norm * 100, annot=True, fmt=".1f", cmap="YlOrRd",
    xticklabels=[c.replace("Corn_","") for c in corn_classes],
    yticklabels=[c.replace("Corn_","") for c in corn_classes],
    ax=ax, linewidths=1, vmin=0, vmax=100
)
ax.set_title("Corn Disease Confusion Matrix (%)", fontsize=13)
ax.set_ylabel("True Label")
ax.set_xlabel("Predicted Label")
plt.tight_layout()
out = GRAPH_DIR / "corn_confusion_check.png"
plt.savefig(out, dpi=150)
print(f"\nCorn confusion matrix saved: {out}")

# ─────────────────────────────────────────────────────────────
# [5] SINGLE IMAGE TEST
# ─────────────────────────────────────────────────────────────
print("\n" + "=" * 60)
print("SINGLE IMAGE TEST — Random corn samples")
print("=" * 60)

def predict_image(image_path):
    img_path = Path(image_path)
    if not img_path.exists():
        return
    img    = np.array(Image.open(img_path).convert("RGB"))
    tensor = tf(image=img)["image"].unsqueeze(0).to(device)
    with torch.no_grad():
        probs = F.softmax(model(tensor), dim=1)[0]
    top5_probs, top5_idx = probs.topk(5)
    print(f"\n  Image: {img_path.name}")
    print(f"  {'Rank':<6} {'Disease':40s} {'Confidence':10s}")
    print(f"  {'-'*60}")
    for rank, (p, i) in enumerate(zip(top5_probs, top5_idx), 1):
        cls = IDX_TO_CLASS[str(i.item())]
        bar = "█" * int(p.item() * 20)
        print(f"  #{rank:<5} {cls:40s} {p.item()*100:5.1f}%  {bar}")

for cls_name in corn_classes:
    cls_dir = PLANT_TEST / cls_name
    if not cls_dir.exists():
        continue
    imgs = list(cls_dir.iterdir())
    if imgs:
        predict_image(random.choice(imgs))

# ─────────────────────────────────────────────────────────────
# FINAL VERDICT
# ─────────────────────────────────────────────────────────────
high_confusion = len([p for p in pairs if p[0] > 15])
print("\n" + "=" * 60)
print("VERDICT")
print("=" * 60)
print(f"\n  Overall accuracy           : {test_acc*100:.2f}%")
print(f"  Corn accuracy              : {corn_acc*100:.2f}%")
print(f"  High confusion >15% pairs  : {high_confusion}")

if test_acc * 100 >= 90 and corn_acc * 100 >= 85 and not need_04b:
    print(f"\n  MODEL IS GOOD")
    print(f"  ACTION: Run python3 05_train_bird.py")
elif test_acc * 100 < 80:
    print(f"\n  MODEL ACCURACY TOO LOW")
    print(f"  ACTION: Re-run python3 04_train_plant.py")
elif need_04b:
    print(f"\n  CORN CONFUSION EXISTS BUT DO NOT RUN 04b")
    print(f"  REASON: 04b causes catastrophic forgetting of all other classes")
    print(f"  ACTION: Run python3 05_train_bird.py — corn confusion is minor")
else:
    print(f"\n  ACTION: Run python3 05_train_bird.py")
print("=" * 60)
