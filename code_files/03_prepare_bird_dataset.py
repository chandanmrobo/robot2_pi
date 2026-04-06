#!/usr/bin/env python3
"""
03_prepare_bird_dataset.py
Builds binary Bird / No_Bird YOLO dataset.
Includes hard negatives (kites, planes) to prevent false positives.
Also runs synthetic augmentation for field conditions.
"""

import random, shutil, cv2
import numpy as np
import albumentations as A
from pathlib import Path
from PIL import Image
from sklearn.model_selection import train_test_split
from tqdm import tqdm

random.seed(42)

BASE_DIR    = Path.home() / "agri_robot_ml"
DATASET_DIR = BASE_DIR / "datasets"
YOLO_DIR    = BASE_DIR / "yolov5"

BIRD_OUTPUT = DATASET_DIR / "bird_yolo"
for split in ["train", "val"]:
    (BIRD_OUTPUT / "images" / split).mkdir(parents=True, exist_ok=True)
    (BIRD_OUTPUT / "labels" / split).mkdir(parents=True, exist_ok=True)

print("=" * 60)
print("Bird Dataset Preparation")
print("Class 0: Bird  | Class 1: No_Bird")
print("=" * 60)

bird_images    = []   # (Path, yolo_label_dir_or_None)
no_bird_images = []   # Path only

# ─────────────────────────────────────────────────────────────
# Collect bird images
# ─────────────────────────────────────────────────────────────
for src_name, img_sub in [
    ("birds_open_images", "train/images"),
    ("birds_coco",        "train/images"),
]:
    d = DATASET_DIR / src_name / img_sub
    if not d.exists():
        d = DATASET_DIR / src_name / "images"
    if d.exists():
        lbl_dir = d.parent.parent / "labels" / "train"
        imgs = list(d.glob("*.jpg")) + list(d.glob("*.jpeg")) + list(d.glob("*.png"))
        bird_images.extend([(img, lbl_dir) for img in imgs])
        print(f"  Birds from {src_name}: {len(imgs)}")
    else:
        print(f"   {src_name} not found")

# Custom field birds (your photos)
custom_birds = DATASET_DIR / "custom_birds" / "bird"
if custom_birds.exists():
    imgs = list(custom_birds.glob("*.*"))
    bird_images.extend([(img, None) for img in imgs])
    print(f"  Custom field birds: {len(imgs)}")

# iNaturalist manually downloaded birds
inat_dir = DATASET_DIR / "inat_birds"
if inat_dir.exists():
    imgs = list(inat_dir.rglob("*.jpg"))
    bird_images.extend([(img, None) for img in imgs])
    print(f"  iNaturalist birds: {len(imgs)}")

print(f"\n  Total bird images collected: {len(bird_images)}")

# ─────────────────────────────────────────────────────────────
# Collect no-bird images
# ─────────────────────────────────────────────────────────────
sky_dir = DATASET_DIR / "sky_no_birds"
if sky_dir.exists():
    imgs = list(sky_dir.glob("*.jpg"))[:1200]
    no_bird_images.extend(imgs)
    print(f"\n  Sky (no birds): {len(imgs)}")

hard_neg = DATASET_DIR / "hard_negatives_bird"
if hard_neg.exists():
    imgs = list(hard_neg.glob("*.jpg"))
    no_bird_images.extend(imgs)
    print(f"  Hard negatives (kites/planes): {len(imgs)}  ← CONFUSION FIX")

custom_no_bird = DATASET_DIR / "custom_birds" / "no_bird"
if custom_no_bird.exists():
    imgs = list(custom_no_bird.glob("*.*"))
    no_bird_images.extend(imgs)
    print(f"  Custom empty sky: {len(imgs)}")

print(f"  Total no-bird images: {len(no_bird_images)}")

# ─────────────────────────────────────────────────────────────
# Balance classes
# ─────────────────────────────────────────────────────────────
min_count = min(len(bird_images), len(no_bird_images))
min_count = min(min_count, 3000)   # cap at 3k per class before augmentation
bird_images    = random.sample(bird_images, min(min_count, len(bird_images)))
no_bird_images = random.sample(no_bird_images, min(min_count, len(no_bird_images)))
print(f"\n  Balanced: {min_count} per class (before augmentation)")

# ─────────────────────────────────────────────────────────────
# Train/Val split
# ─────────────────────────────────────────────────────────────
all_data = (
    [(img, lbl, 0) for img, lbl in bird_images] +
    [(img, None, 1) for img in no_bird_images]
)
train_data, val_data = train_test_split(
    all_data, test_size=0.2, random_state=42,
    stratify=[x[2] for x in all_data]
)
print(f"  Train: {len(train_data)} | Val: {len(val_data)}")

# ─────────────────────────────────────────────────────────────
# Copy images + labels
# ─────────────────────────────────────────────────────────────
def process_split(data, split):
    ok = skip = 0
    for img_path, lbl_dir, class_id in data:
        try:
            with Image.open(img_path) as img:
                if img.mode != "RGB":
                    img = img.convert("RGB")
                if img.size[0] < 50 or img.size[1] < 50:
                    skip += 1
                    continue
                out_img = BIRD_OUTPUT / "images" / split / f"{img_path.stem}_{class_id}.jpg"
                img.save(str(out_img), quality=90)

            dst_lbl = BIRD_OUTPUT / "labels" / split / f"{img_path.stem}_{class_id}.txt"

            if class_id == 0 and lbl_dir is not None:
                existing = lbl_dir / f"{img_path.stem}.txt"
                if existing.exists():
                    lines = existing.read_text().splitlines()
                    with open(dst_lbl, "w") as f:
                        for line in lines:
                            parts = line.split()
                            if len(parts) >= 5:
                                f.write(f"0 {' '.join(parts[1:])}\n")
                    ok += 1
                    continue
                # No label file → assume whole image is bird
                with open(dst_lbl, "w") as f:
                    f.write("0 0.5 0.5 1.0 1.0\n")
            else:
                # No_Bird → empty label file
                dst_lbl.touch()

            ok += 1
        except Exception as e:
            skip += 1
    print(f"  {split}: {ok} saved, {skip} skipped")

print("\nCopying images...")
process_split(train_data, "train")
process_split(val_data,   "val")

# ─────────────────────────────────────────────────────────────
# SYNTHETIC AUGMENTATION — Indian field conditions
# ─────────────────────────────────────────────────────────────
print("\n" + "=" * 60)
print("Running synthetic augmentation (field conditions)...")
print("Factors: harsh sun, motion blur, vibration, dawn/dusk")

bird_augment = A.Compose([
    # Indian sun: harsh brightness variations
    A.RandomBrightnessContrast(brightness_limit=0.5, contrast_limit=0.4, p=0.95),
    A.HueSaturationValue(hue_shift_limit=15, sat_shift_limit=30, val_shift_limit=25, p=0.7),
    # Atmospheric haze / sun flare
    A.OneOf([
        A.RandomFog(fog_coef_lower=0.05, fog_coef_upper=0.2, p=1.0),
        A.RandomSunFlare(flare_roi=(0, 0, 1, 0.5), angle_lower=0, angle_upper=1, p=1.0),
    ], p=0.35),
    # Motion blur (bird flying + robot moving on bumpy ground)
    A.MotionBlur(blur_limit=17, p=0.7),
    A.OneOf([
        A.GaussianBlur(blur_limit=(3, 9), p=1.0),
        A.Rotate(limit=8, border_mode=cv2.BORDER_CONSTANT, value=0, p=1.0),
    ], p=0.4),
    # JPEG compression artifacts (Pi Camera compression)
    A.ImageCompression(quality_lower=50, quality_upper=90, p=0.25),
])

BIRD_TRAIN_IMG = BIRD_OUTPUT / "images" / "train"
BIRD_TRAIN_LBL = BIRD_OUTPUT / "labels" / "train"
generated = 0

for img_path in tqdm(list(BIRD_TRAIN_IMG.glob("*.jpg")), desc="Synthetic augmentation"):
    try:
        img = cv2.imread(str(img_path))
        if img is None:
            continue
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        orig_lbl = BIRD_TRAIN_LBL / f"{img_path.stem}.txt"
        lbl_content = orig_lbl.read_text() if orig_lbl.exists() else ""

        # Generate 2 synthetic variants per image
        for v in range(2):
            aug = bird_augment(image=img_rgb)["image"]
            out_img = BIRD_TRAIN_IMG / f"{img_path.stem}_syn{v}.jpg"
            cv2.imwrite(str(out_img), cv2.cvtColor(aug, cv2.COLOR_RGB2BGR),
                        [cv2.IMWRITE_JPEG_QUALITY, 90])
            (BIRD_TRAIN_LBL / f"{img_path.stem}_syn{v}.txt").write_text(lbl_content)
            generated += 1
    except:
        continue

total_train = len(list(BIRD_TRAIN_IMG.glob("*.jpg")))
print(f"  Generated {generated} synthetic images")
print(f"  Total bird train: {total_train} (3x augmented)")

# ─────────────────────────────────────────────────────────────
# Write YAML for YOLOv5
# ─────────────────────────────────────────────────────────────
bird_yaml = f"""path: {BIRD_OUTPUT}
train: images/train
val: images/val

nc: 2
names:
  0: Bird
  1: No_Bird
"""
yaml_path = YOLO_DIR / "bird.yaml"
with open(yaml_path, "w") as f:
    f.write(bird_yaml)

print(f"\n Bird YAML written: {yaml_path}")
print(f" Bird dataset ready at: {BIRD_OUTPUT}")
print("\nRun: python 04_train_plant.py")
