#!/usr/bin/env python3
"""
04b_fix_confusion.py  (Run AFTER 04_train_plant.py if confusion matrix shows issues)
Targeted fine-tune for hard class pairs (Blight vs GrayLeafSpot).
Uses grayscale + CLAHE to force shape/texture learning (not color).
Only run this if confusion matrix from 04_train_plant.py shows >15% confusion
between Corn_NorthernBlight and Corn_GrayLeafSpot.
"""

import torch, timm, json, random, time
import torch.nn as nn
import torch.nn.functional as F
import albumentations as A
import numpy as np
from albumentations.pytorch import ToTensorV2
from torch.utils.data import Dataset, DataLoader
from torch.optim import AdamW
from torch.cuda.amp import autocast, GradScaler
from pathlib import Path
from PIL import Image

random.seed(42)

BASE_DIR    = Path.home() / "agri_robot_ml"
DATASET_DIR = BASE_DIR / "datasets"
PLANT_TRAIN = DATASET_DIR / "plant_split" / "train"
MODEL_DIR   = BASE_DIR / "models" / "plant"
BEST_MODEL  = MODEL_DIR / "best_plant_model.pth"
CLASS_JSON  = BASE_DIR / "plant_class_index.json"

# ─────────────────────────────────────────────────────────────
with open(CLASS_JSON) as f:
    data = json.load(f)
CLASS_TO_IDX = data["class_to_idx"]
IDX_TO_CLASS = data["idx_to_class"]
final_classes = sorted(CLASS_TO_IDX.keys())

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print("=" * 60)
print("Targeted confusion fix: Blight vs GrayLeafSpot")
print(f"Device: {device}")
print("=" * 60)

# ─────────────────────────────────────────────────────────────
# Hard pair transform — removes color cue, forces shape/texture
# ─────────────────────────────────────────────────────────────
hard_tf = A.Compose([
    A.SmallestMaxSize(256),
    A.RandomCrop(224, 224),
    A.HorizontalFlip(p=0.5),
    A.ToGray(p=0.4),                          # Remove color shortcut
    A.Equalize(p=0.3),
    A.CLAHE(clip_limit=6, p=0.5),             # Enhance leaf texture
    A.RandomBrightnessContrast(0.4, 0.4, p=0.7),
    A.Sharpen(alpha=(0.3, 0.7), p=0.6),       # Sharpen vein/lesion edges
    A.GaussNoise(var_limit=(20, 80), p=0.5),
    A.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
    ToTensorV2(),
])

# ─────────────────────────────────────────────────────────────
# Only train on the hard pairs
# ─────────────────────────────────────────────────────────────
HARD_PAIRS = ["Corn_NorthernBlight", "Corn_GrayLeafSpot"]
existing   = [c for c in HARD_PAIRS if c in CLASS_TO_IDX]

if len(existing) < 2:
    print(f"⚠️  Hard pair classes not found in dataset: {HARD_PAIRS}")
    print(f"   Available classes: {[c for c in final_classes if 'Corn' in c]}")
    exit(0)

class HardPairDataset(Dataset):
    def __init__(self, root, classes, class_to_idx, transform):
        self.samples   = []
        self.transform = transform
        for cls_name in classes:
            cls_dir = Path(root) / cls_name
            if not cls_dir.exists():
                continue
            label = class_to_idx[cls_name]
            for img in cls_dir.iterdir():
                if img.suffix.lower() in [".jpg", ".jpeg", ".png"]:
                    self.samples.append((img, label))

    def __len__(self): return len(self.samples)
    def __getitem__(self, idx):
        path, label = self.samples[idx]
        img = np.array(Image.open(path).convert("RGB"))
        return self.transform(image=img)["image"], label

hard_ds = HardPairDataset(PLANT_TRAIN, existing, CLASS_TO_IDX, hard_tf)
hard_dl = DataLoader(hard_ds, batch_size=16, shuffle=True, num_workers=4)
print(f"\nHard pair dataset: {len(hard_ds)} images")
print(f"Classes: {existing}")

# ─────────────────────────────────────────────────────────────
# Load existing model
# ─────────────────────────────────────────────────────────────
ckpt  = torch.load(BEST_MODEL, map_location=device)
model = timm.create_model("efficientnet_b4", pretrained=False, num_classes=len(final_classes))
model.load_state_dict(ckpt["model_state_dict"])
model = model.to(device)
best_acc = ckpt.get("val_acc", 0.0)
print(f"\nLoaded model with val_acc: {best_acc * 100:.2f}%")

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

criterion = FocalLoss(gamma=2.5, label_smoothing=0.05)  # Higher gamma for hard pairs
optimizer = AdamW(model.parameters(), lr=5e-5, weight_decay=1e-4)
scaler    = GradScaler()

# ─────────────────────────────────────────────────────────────
# Fine-tune 5 epochs on hard pairs only
# ─────────────────────────────────────────────────────────────
print("\nFine-tuning on hard pairs...")
for ep in range(5):
    model.train()
    tl = correct = total = 0
    for imgs, labels in hard_dl:
        imgs, labels = imgs.to(device), labels.to(device)
        optimizer.zero_grad()
        with autocast():
            out  = model(imgs)
            loss = criterion(out, labels)
        scaler.scale(loss).backward()
        scaler.step(optimizer)
        scaler.update()
        tl      += loss.item() * imgs.size(0)
        correct += out.argmax(1).eq(labels).sum().item()
        total   += labels.size(0)
    print(f"  Hard pair ep {ep+1}/5 | acc {correct/total:.3f} | loss {tl/total:.4f}")

# Save if improved (always save after hard pair fix)
torch.save({
    "model_state_dict": model.state_dict(),
    "class_to_idx": CLASS_TO_IDX,
    "idx_to_class": IDX_TO_CLASS,
    "val_acc": best_acc,
    "hard_pair_finetuned": True
}, MODEL_DIR / "best_plant_model_hardfix.pth")

print(f"\n✅ Hard-pair fixed model saved: {MODEL_DIR / 'best_plant_model_hardfix.pth'}")
print("Re-run evaluation section of 04_train_plant.py with this model to verify improvement.")
