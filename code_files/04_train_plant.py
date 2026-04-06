#!/usr/bin/env python3
"""
04_train_plant.py
Trains EfficientNetB4 for plant disease classification.

Why EfficientNetB4 (not YOLOv5 for plant):
- Classification task (not detection) → EfficientNet better
- Pretrained ImageNet features transfer well to leaf texture/color
- Focal Loss fixes Blight vs GrayLeafSpot confusion
- Mixup prevents color-only shortcuts

Two-phase training:
- Phase 1: Freeze backbone, warm up classifier head (5 epochs)
- Phase 2: Full finetune with discriminative LR (25 epochs)

Expected: 90%+ accuracy (vs your previous 65%)
"""

import torch, timm, json, random, time
import torch.nn as nn
import torch.nn.functional as F
import albumentations as A
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from albumentations.pytorch import ToTensorV2
from torch.utils.data import Dataset, DataLoader
from torch.optim import AdamW
from torch.optim.lr_scheduler import OneCycleLR
from torch.cuda.amp import autocast, GradScaler
from sklearn.metrics import classification_report, confusion_matrix
from pathlib import Path
from PIL import Image
from collections import Counter

random.seed(42)
torch.manual_seed(42)

BASE_DIR    = Path.home() / "agri_robot_ml"
DATASET_DIR = BASE_DIR / "datasets"

PLANT_TRAIN = DATASET_DIR / "plant_split" / "train"
PLANT_VAL   = DATASET_DIR / "plant_split" / "val"
PLANT_TEST  = DATASET_DIR / "plant_split" / "test"
MODEL_DIR   = BASE_DIR / "models" / "plant"
GRAPH_DIR   = BASE_DIR / "graphs"

MODEL_DIR.mkdir(parents=True, exist_ok=True)
GRAPH_DIR.mkdir(parents=True, exist_ok=True)

BEST_MODEL_PATH = MODEL_DIR / "best_plant_model.pth"
CLASS_JSON      = BASE_DIR / "plant_class_index.json"

# ─────────────────────────────────────────────────────────────
# Load class index
# ─────────────────────────────────────────────────────────────
with open(CLASS_JSON) as f:
    class_data = json.load(f)
CLASS_TO_IDX = class_data["class_to_idx"]
IDX_TO_CLASS = class_data["idx_to_class"]
final_classes = sorted(CLASS_TO_IDX.keys())
NUM_CLASSES   = len(final_classes)

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print("=" * 60)
print(f"Plant Disease Training — EfficientNetB4")
print(f"Device : {device}")
print(f"Classes: {NUM_CLASSES}")
print("=" * 60)
if torch.cuda.is_available():
    print(f"GPU: {torch.cuda.get_device_name(0)}")
    print(f"VRAM: {torch.cuda.get_device_properties(0).total_memory / 1e9:.1f} GB")

IMG_SIZE   = 224
BATCH_SIZE = 32    # Increase to 64 if VRAM > 12GB
EPOCHS_P1  = 5
EPOCHS_P2  = 25

# ─────────────────────────────────────────────────────────────
# Augmentation transforms
# ─────────────────────────────────────────────────────────────
train_tf = A.Compose([
    A.SmallestMaxSize(256),
    A.RandomCrop(IMG_SIZE, IMG_SIZE),
    A.HorizontalFlip(p=0.5),
    A.VerticalFlip(p=0.3),
    A.RandomRotate90(p=0.5),
    A.OneOf([
        A.RandomBrightnessContrast(0.3, 0.3, p=1.0),   # Harsh Indian sun
        A.RandomGamma(gamma_limit=(70, 130), p=1.0),
        A.CLAHE(clip_limit=4.0, p=1.0),
    ], p=0.8),
    A.HueSaturationValue(20, 30, 20, p=0.6),
    A.RGBShift(15, 15, 15, p=0.4),
    A.OneOf([
        A.GaussianBlur(blur_limit=(3, 7), p=1.0),       # Robot vibration
        A.Sharpen(alpha=(0.2, 0.5), lightness=(0.5, 1.0), p=1.0),
    ], p=0.5),
    A.GaussNoise(var_limit=(10, 50), p=0.4),            # Dust/noise
    A.CoarseDropout(max_holes=8, max_height=32, max_width=32, fill_value=0, p=0.3),
    A.ElasticTransform(alpha=1, sigma=50, alpha_affine=50, p=0.3),
    A.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
    ToTensorV2(),
])

val_tf = A.Compose([
    A.SmallestMaxSize(256),
    A.CenterCrop(IMG_SIZE, IMG_SIZE),
    A.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
    ToTensorV2(),
])

# ─────────────────────────────────────────────────────────────
# Dataset
# ─────────────────────────────────────────────────────────────
class PlantDataset(Dataset):
    def __init__(self, root, class_to_idx, transform):
        self.samples   = []
        self.transform = transform
        for cls_dir in Path(root).iterdir():
            if not cls_dir.is_dir() or cls_dir.name not in class_to_idx:
                continue
            label = class_to_idx[cls_dir.name]
            for img in cls_dir.iterdir():
                if img.suffix.lower() in [".jpg", ".jpeg", ".png"]:
                    self.samples.append((img, label))

    def __len__(self):
        return len(self.samples)

    def __getitem__(self, idx):
        path, label = self.samples[idx]
        img = np.array(Image.open(path).convert("RGB"))
        return self.transform(image=img)["image"], label

train_ds = PlantDataset(PLANT_TRAIN, CLASS_TO_IDX, train_tf)
val_ds   = PlantDataset(PLANT_VAL,   CLASS_TO_IDX, val_tf)
test_ds  = PlantDataset(PLANT_TEST,  CLASS_TO_IDX, val_tf)

# Weighted sampler (handles class imbalance)
label_counts = Counter([s[1] for s in train_ds.samples])
weights      = [1.0 / label_counts[s[1]] for s in train_ds.samples]
sampler      = torch.utils.data.WeightedRandomSampler(weights, len(train_ds), replacement=True)

train_dl = DataLoader(train_ds, batch_size=BATCH_SIZE, sampler=sampler, num_workers=4, pin_memory=True)
val_dl   = DataLoader(val_ds,   batch_size=BATCH_SIZE, shuffle=False,  num_workers=4, pin_memory=True)
test_dl  = DataLoader(test_ds,  batch_size=BATCH_SIZE, shuffle=False,  num_workers=4, pin_memory=True)

print(f"\nDataset split:")
print(f"  Train: {len(train_ds)} images | {len(train_dl)} batches")
print(f"  Val  : {len(val_ds)}   images | {len(val_dl)}   batches")
print(f"  Test : {len(test_ds)}  images | {len(test_dl)}  batches")

# ─────────────────────────────────────────────────────────────
# Model — EfficientNetB4
# ─────────────────────────────────────────────────────────────
model = timm.create_model(
    "efficientnet_b4",
    pretrained=True,
    num_classes=NUM_CLASSES,
    drop_rate=0.4,
    drop_path_rate=0.2
)
model = model.to(device)
print(f"\nEfficientNetB4: {sum(p.numel() for p in model.parameters()) / 1e6:.1f}M params")

# ─────────────────────────────────────────────────────────────
# Focal Loss — forces learning hard pairs (Blight vs GrayLeafSpot)
# ─────────────────────────────────────────────────────────────
class FocalLoss(nn.Module):
    def __init__(self, gamma=2.0, label_smoothing=0.1):
        super().__init__()
        self.gamma = gamma
        self.ls    = label_smoothing

    def forward(self, inputs, targets):
        n = inputs.size(1)
        with torch.no_grad():
            smooth = torch.full_like(inputs, self.ls / (n - 1))
            smooth.scatter_(1, targets.unsqueeze(1), 1.0 - self.ls)
        log_p = F.log_softmax(inputs, dim=1)
        p_t   = (log_p.exp() * smooth).sum(dim=1)
        loss  = (-(smooth * log_p).sum(dim=1)) * ((1 - p_t) ** self.gamma)
        return loss.mean()

criterion = FocalLoss(gamma=2.0, label_smoothing=0.1)

# ─────────────────────────────────────────────────────────────
# Mixup — prevents color-only shortcuts (Blight/GrayLeafSpot both brown-gray)
# ─────────────────────────────────────────────────────────────
def mixup_data(x, y, alpha=0.2):
    lam = np.random.beta(alpha, alpha) if alpha > 0 else 1
    idx = torch.randperm(x.size(0)).to(device)
    return lam * x + (1 - lam) * x[idx], y, y[idx], lam

def mixup_loss(crit, pred, ya, yb, lam):
    return lam * crit(pred, ya) + (1 - lam) * crit(pred, yb)

# ─────────────────────────────────────────────────────────────
# Training loop
# ─────────────────────────────────────────────────────────────
scaler  = GradScaler()
history = {"tl": [], "ta": [], "vl": [], "va": []}
best_acc = 0.0
scheduler = None
optimizer = None

def train_epoch(use_mixup=True):
    model.train()
    tl = correct = total = 0
    for imgs, labels in train_dl:
        imgs, labels = imgs.to(device), labels.to(device)
        if use_mixup and random.random() < 0.5:
            imgs, ya, yb, lam = mixup_data(imgs, labels)
            with autocast():
                out  = model(imgs)
                loss = mixup_loss(criterion, out, ya, yb, lam)
        else:
            with autocast():
                out  = model(imgs)
                loss = criterion(out, labels)

        optimizer.zero_grad()
        scaler.scale(loss).backward()
        scaler.unscale_(optimizer)
        torch.nn.utils.clip_grad_norm_(model.parameters(), 1.0)
        scaler.step(optimizer)
        scaler.update()
        if scheduler:
            scheduler.step()

        tl      += loss.item() * imgs.size(0)
        correct += out.argmax(1).eq(labels).sum().item()
        total   += labels.size(0)
    return tl / total, correct / total

@torch.no_grad()
def val_epoch():
    model.eval()
    vl = correct = total = 0
    for imgs, labels in val_dl:
        imgs, labels = imgs.to(device), labels.to(device)
        with autocast():
            out  = model(imgs)
            loss = criterion(out, labels)
        vl      += loss.item() * imgs.size(0)
        correct += out.argmax(1).eq(labels).sum().item()
        total   += labels.size(0)
    return vl / total, correct / total

# ─────────────────────────────────────────────────────────────
# PHASE 1: Frozen backbone — warm up head
# ─────────────────────────────────────────────────────────────
print("\n" + "=" * 60)
print(f"PHASE 1 ({EPOCHS_P1} epochs): Head only, backbone frozen")
print("=" * 60)

for n, p in model.named_parameters():
    if "classifier" not in n:
        p.requires_grad = False

optimizer = AdamW(
    filter(lambda p: p.requires_grad, model.parameters()),
    lr=1e-3, weight_decay=1e-4
)
scheduler = OneCycleLR(
    optimizer, max_lr=1e-3,
    steps_per_epoch=len(train_dl), epochs=EPOCHS_P1, pct_start=0.3
)

for ep in range(EPOCHS_P1):
    t0 = time.time()
    tl, ta = train_epoch(use_mixup=False)
    vl, va = val_epoch()
    history["tl"].append(tl); history["ta"].append(ta)
    history["vl"].append(vl); history["va"].append(va)
    mark = ""
    if va > best_acc:
        best_acc = va
        torch.save({
            "model_state_dict": model.state_dict(),
            "class_to_idx": CLASS_TO_IDX,
            "idx_to_class": IDX_TO_CLASS,
            "val_acc": va, "epoch": ep
        }, BEST_MODEL_PATH)
        mark = " ✅ BEST"
    print(f"  Ep {ep+1:2d}/{EPOCHS_P1} | Train {ta:.3f} ({tl:.4f}) | Val {va:.3f} ({vl:.4f}) | {time.time()-t0:.0f}s{mark}")

# ─────────────────────────────────────────────────────────────
# PHASE 2: Full finetune with discriminative LR
# ─────────────────────────────────────────────────────────────
print("\n" + "=" * 60)
print(f"PHASE 2 ({EPOCHS_P2} epochs): Full finetune")
print("=" * 60)

for p in model.parameters():
    p.requires_grad = True

backbone_p = [p for n, p in model.named_parameters() if "classifier" not in n]
head_p     = [p for n, p in model.named_parameters() if "classifier" in n]
optimizer  = AdamW([
    {"params": backbone_p, "lr": 1e-5},
    {"params": head_p,     "lr": 1e-4},
], weight_decay=1e-4)
scheduler = OneCycleLR(
    optimizer, max_lr=[1e-4, 5e-4],
    steps_per_epoch=len(train_dl), epochs=EPOCHS_P2,
    pct_start=0.1, anneal_strategy="cos"
)

patience = 8
patience_ctr = 0

for ep in range(EPOCHS_P2):
    t0 = time.time()
    tl, ta = train_epoch(use_mixup=True)
    vl, va = val_epoch()
    history["tl"].append(tl); history["ta"].append(ta)
    history["vl"].append(vl); history["va"].append(va)
    mark = ""
    if va > best_acc:
        best_acc = va
        torch.save({
            "model_state_dict": model.state_dict(),
            "class_to_idx": CLASS_TO_IDX,
            "idx_to_class": IDX_TO_CLASS,
            "val_acc": va, "epoch": EPOCHS_P1 + ep
        }, BEST_MODEL_PATH)
        patience_ctr = 0
        mark = " ... BEST"
    else:
        patience_ctr += 1
        mark = f" ({patience_ctr}/{patience})"
    print(f"  Ep {ep+1:2d}/{EPOCHS_P2} | Train {ta:.3f} ({tl:.4f}) | Val {va:.3f} ({vl:.4f}) | {time.time()-t0:.0f}s{mark}")
    if patience_ctr >= patience:
        print("\n Early stopping")
        break

print(f"\n Best Plant Val Accuracy: {best_acc:.4f} ({best_acc * 100:.2f}%)")

# ─────────────────────────────────────────────────────────────
# EVALUATE on test set
# ─────────────────────────────────────────────────────────────
print("\n" + "=" * 60)
print("Evaluating on test set...")

ckpt = torch.load(BEST_MODEL_PATH)
model.load_state_dict(ckpt["model_state_dict"])
model.eval()

all_preds  = []
all_labels = []
with torch.no_grad():
    for imgs, labels in test_dl:
        imgs = imgs.to(device)
        with autocast():
            out = model(imgs)
        all_preds.extend(out.argmax(1).cpu().numpy())
        all_labels.extend(labels.numpy())

all_preds  = np.array(all_preds)
all_labels = np.array(all_labels)
test_acc   = (all_preds == all_labels).mean()
print(f"\n Plant Test Accuracy: {test_acc * 100:.2f}%")
print(f" Best Val  Accuracy : {best_acc * 100:.2f}%")
print("\nPer-class report:")
print(classification_report(all_labels, all_preds, target_names=final_classes, digits=3))

# ─────────────────────────────────────────────────────────────
# SAVE GRAPHS
# ─────────────────────────────────────────────────────────────
fig, axes = plt.subplots(1, 2, figsize=(14, 5))
axes[0].plot(history["tl"], label="Train")
axes[0].plot(history["vl"], label="Val")
axes[0].axvline(EPOCHS_P1, color="g", linestyle="--", label="Phase 2 start")
axes[0].set_title("Loss"); axes[0].legend()
axes[1].plot([a * 100 for a in history["ta"]], label="Train")
axes[1].plot([a * 100 for a in history["va"]], label="Val")
axes[1].axvline(EPOCHS_P1, color="g", linestyle="--", label="Phase 2 start")
axes[1].set_title("Accuracy (%)"); axes[1].legend()
plt.tight_layout()
plt.savefig(GRAPH_DIR / "plant_training_curves.png", dpi=150, bbox_inches="tight")
print(f"\n Training curves saved: {GRAPH_DIR / 'plant_training_curves.png'}")

cm      = confusion_matrix(all_labels, all_preds)
cm_norm = cm.astype("float") / cm.sum(axis=1)[:, np.newaxis]
sz = max(12, len(final_classes))
fig, ax = plt.subplots(figsize=(sz, sz - 2))
sns.heatmap(cm_norm, annot=True, fmt=".2f", cmap="YlOrRd",
            xticklabels=final_classes, yticklabels=final_classes,
            ax=ax, linewidths=0.5)
ax.set_title("Plant Disease — Normalized Confusion Matrix", fontsize=14)
ax.set_ylabel("True"); ax.set_xlabel("Predicted")
plt.xticks(rotation=45, ha="right"); plt.yticks(rotation=0)
plt.tight_layout()
plt.savefig(GRAPH_DIR / "plant_confusion_matrix.png", dpi=150, bbox_inches="tight")
print(f" Confusion matrix saved: {GRAPH_DIR / 'plant_confusion_matrix.png'}")

# Show worst confused pairs
print("\n Top confused pairs:")
np.fill_diagonal(cm_norm, 0)
pairs = [(cm_norm[i, j], final_classes[i], final_classes[j])
         for i in range(len(final_classes))
         for j in range(len(final_classes))
         if cm_norm[i, j] > 0.05]
for score, true, pred in sorted(pairs, reverse=True)[:8]:
    print(f"  {true} → {pred}: {score * 100:.1f}%")

print(f"\n Plant model saved: {BEST_MODEL_PATH}")
print("Run: python 05_train_bird.py")
