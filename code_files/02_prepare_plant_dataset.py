#!/usr/bin/env python3
"""
02_prepare_plant_dataset.py
Organizes all plant datasets into clean per-class folders.
Each plant gets its own disease labels (Corn_Blight, Corn_GrayLeafSpot etc.)
Balances classes and creates train/val/test splits.
"""

import os, shutil, random, json
from pathlib import Path
from collections import defaultdict
from PIL import Image

random.seed(42)

BASE_DIR    = Path.home() / "agri_robot_ml"
DATASET_DIR = BASE_DIR / "datasets"

PLANT_ORGANIZED = DATASET_DIR / "plant_organized"
PLANT_SPLIT     = DATASET_DIR / "plant_split"

PLANT_ORGANIZED.mkdir(parents=True, exist_ok=True)
for split in ["train", "val", "test"]:
    (PLANT_SPLIT / split).mkdir(parents=True, exist_ok=True)

# ─────────────────────────────────────────────────────────────
# CLASS MAP: PlantVillage folder name → clean label
# Each disease is its own class so model learns exactly
# ─────────────────────────────────────────────────────────────
PLANTVILLAGE_MAP = {
    # CORN (your primary crop)
    "corn_(maize)___cercospora_leaf_spot gray_leaf_spot": "Corn_GrayLeafSpot",
    "corn_(maize)___common_rust_":                        "Corn_CommonRust",
    "corn_(maize)___northern_leaf_blight":                "Corn_NorthernBlight",
    "corn_(maize)___healthy":                             "Corn_Healthy",
    # TOMATO
    "tomato___bacterial_spot":                            "Tomato_BacterialSpot",
    "tomato___early_blight":                              "Tomato_EarlyBlight",
    "tomato___late_blight":                               "Tomato_LateBlight",
    "tomato___leaf_mold":                                 "Tomato_LeafMold",
    "tomato___septoria_leaf_spot":                        "Tomato_SeptoriaLeafSpot",
    "tomato___spider_mites two-spotted_spider_mite":      "Tomato_SpiderMites",
    "tomato___target_spot":                               "Tomato_TargetSpot",
    "tomato___tomato_mosaic_virus":                       "Tomato_MosaicVirus",
    "tomato___tomato_yellow_leaf_curl_virus":              "Tomato_YellowCurlVirus",
    "tomato___healthy":                                   "Tomato_Healthy",
    # POTATO
    "potato___early_blight":                              "Potato_EarlyBlight",
    "potato___late_blight":                               "Potato_LateBlight",
    "potato___healthy":                                   "Potato_Healthy",
    # GRAPE
    "grape___black_rot":                                  "Grape_BlackRot",
    "grape___esca_(black_measles)":                       "Grape_BlackMeasles",
    "grape___leaf_blight_(isariopsis_leaf_spot)":         "Grape_LeafBlight",
    "grape___healthy":                                    "Grape_Healthy",
    # APPLE
    "apple___apple_scab":                                 "Apple_Scab",
    "apple___black_rot":                                  "Apple_BlackRot",
    "apple___cedar_apple_rust":                           "Apple_CedarRust",
    "apple___healthy":                                    "Apple_Healthy",
    # PEPPER
    "pepper,_bell___bacterial_spot":                      "Pepper_BacterialSpot",
    "pepper,_bell___healthy":                             "Pepper_Healthy",
    # STRAWBERRY
    "strawberry___leaf_scorch":                           "Strawberry_LeafScorch",
    "strawberry___healthy":                               "Strawberry_Healthy",
    # SQUASH
    "squash___powdery_mildew":                            "Squash_PowderyMildew",
    # PEACH
    "peach___bacterial_spot":                             "Peach_BacterialSpot",
    "peach___healthy":                                    "Peach_Healthy",
    # CHERRY
    "cherry_(including_sour)___powdery_mildew":           "Cherry_PowderyMildew",
    "cherry_(including_sour)___healthy":                  "Cherry_Healthy",
    # ORANGE
    "orange___haunglongbing_(citrus_greening)":           "Orange_CitrusGreening",
    # OTHERS
    "soybean___healthy":                                  "Soybean_Healthy",
    "raspberry___healthy":                                "Raspberry_Healthy",
    "blueberry___healthy":                                "Blueberry_Healthy",
}

KAGGLE_CORN_MAP = {
    "blight":               "Corn_NorthernBlight",
    "northern_leaf_blight": "Corn_NorthernBlight",
    "common_rust":          "Corn_CommonRust",
    "gray_leaf_spot":       "Corn_GrayLeafSpot",
    "healthy":              "Corn_Healthy",
}

# ─────────────────────────────────────────────────────────────
# HELPERS
# ─────────────────────────────────────────────────────────────
class_counts = defaultdict(int)
errors = []

def safe_copy(src, label):
    dst_dir = PLANT_ORGANIZED / label
    dst_dir.mkdir(parents=True, exist_ok=True)
    dst = dst_dir / f"{label}_{class_counts[label]:05d}{Path(src).suffix}"
    try:
        img = Image.open(src)
        img.verify()
        shutil.copy2(src, dst)
        class_counts[label] += 1
    except Exception as e:
        errors.append(str(src))

# ─────────────────────────────────────────────────────────────
# 1. PlantVillage
# ─────────────────────────────────────────────────────────────
print("=" * 60)
print("Processing PlantVillage...")
pv_roots = [
    DATASET_DIR / "plantvillage dataset" / "segmented",
    DATASET_DIR / "PlantVillage" / "segmented",
    DATASET_DIR / "plantvillage_dataset" / "segmented",
]
pv_dir = next((p for p in pv_roots if p.exists()), None)

if pv_dir:
    for cls_dir in sorted(pv_dir.iterdir()):
        if not cls_dir.is_dir():
            continue
        label = PLANTVILLAGE_MAP.get(cls_dir.name.lower())
        if label is None:
            print(f"  No map for: {cls_dir.name}")
            continue
        for img in cls_dir.glob("*.*"):
            if img.suffix.lower() in [".jpg", ".jpeg", ".png"]:
                safe_copy(img, label)
    print(f"  PlantVillage done")
else:
    print("   PlantVillage not found — check datasets folder")

# ─────────────────────────────────────────────────────────────
# 2. Kaggle Corn datasets
# ─────────────────────────────────────────────────────────────
print("\nProcessing Kaggle Corn datasets...")
for search_dir in [
    DATASET_DIR / "corn-or-maize-leaf-disease-dataset",
    DATASET_DIR / "corn_leaf_infection_dataset",
    DATASET_DIR / "Corn_Leaf_Infection_Dataset",
]:
    if not search_dir.exists():
        continue
    for split in ["train", "test", "valid", "Train", "Test", "Valid"]:
        split_dir = search_dir / split
        if not split_dir.exists():
            continue
        for cls_dir in split_dir.iterdir():
            if not cls_dir.is_dir():
                continue
            name = cls_dir.name.lower().replace(" ", "_")
            label = next((v for k, v in KAGGLE_CORN_MAP.items() if k in name), None)
            if label is None:
                print(f"   No map for Kaggle: {cls_dir.name}")
                continue
            for img in cls_dir.glob("*.*"):
                if img.suffix.lower() in [".jpg", ".jpeg", ".png"]:
                    safe_copy(img, label)
    print(f"  {search_dir.name} done")

# ─────────────────────────────────────────────────────────────
# 3. PlantDoc
# ─────────────────────────────────────────────────────────────
print("\nProcessing PlantDoc...")
plantdoc = DATASET_DIR / "PlantDoc"
PLANTDOC_MAP = {
    "corn": {
        "northern leaf blight":  "Corn_NorthernBlight",
        "gray leaf spot":        "Corn_GrayLeafSpot",
        "common rust":           "Corn_CommonRust",
        "healthy":               "Corn_Healthy",
    }
}
if plantdoc.exists():
    for img_path in plantdoc.rglob("*.*"):
        if img_path.suffix.lower() not in [".jpg", ".jpeg", ".png"]:
            continue
        name_lower = img_path.stem.lower()
        for crop, disease_map in PLANTDOC_MAP.items():
            if crop in name_lower:
                for disease_key, label in disease_map.items():
                    if disease_key in name_lower:
                        safe_copy(img_path, label)
                        break
    print("   PlantDoc done")
else:
    print("   PlantDoc not found — skipping")

# ─────────────────────────────────────────────────────────────
# 4. Custom field photos
# ─────────────────────────────────────────────────────────────
print("\nProcessing custom field photos...")
custom_plant = DATASET_DIR / "custom_plants"
if custom_plant.exists():
    for cls_dir in custom_plant.iterdir():
        if not cls_dir.is_dir():
            continue
        for img in cls_dir.glob("*.*"):
            if img.suffix.lower() in [".jpg", ".jpeg", ".png"]:
                safe_copy(img, cls_dir.name)
    print("   Custom plants done")
else:
    print("   No custom_plants folder — add field photos later")

# ─────────────────────────────────────────────────────────────
# SUMMARY
# ─────────────────────────────────────────────────────────────
print("\n" + "=" * 60)
print("Class counts (before balancing):")
total = 0
for label, count in sorted(class_counts.items()):
    print(f"  {label:45s}: {count:5d}")
    total += count
print(f"  {'TOTAL':45s}: {total:5d}")
print(f"  Corrupt skipped: {len(errors)}")

# ─────────────────────────────────────────────────────────────
# BALANCE + SPLIT
# ─────────────────────────────────────────────────────────────
print("\n" + "=" * 60)
print("Balancing classes and creating train/val/test split...")
print("  Max per class: 2000 | Min per class: 50")

MAX_PER_CLASS = 2000
MIN_PER_CLASS = 50

PLANT_TRAIN = PLANT_SPLIT / "train"
PLANT_VAL   = PLANT_SPLIT / "val"
PLANT_TEST  = PLANT_SPLIT / "test"

for d in [PLANT_TRAIN, PLANT_VAL, PLANT_TEST]:
    d.mkdir(parents=True, exist_ok=True)

final_classes = []
for cls_dir in sorted(PLANT_ORGANIZED.iterdir()):
    if not cls_dir.is_dir():
        continue
    images = list(cls_dir.iterdir())
    if len(images) < MIN_PER_CLASS:
        print(f"   Skip {cls_dir.name}: {len(images)} < {MIN_PER_CLASS}")
        shutil.rmtree(cls_dir)
        continue
    if len(images) > MAX_PER_CLASS:
        for f in random.sample(images, len(images) - MAX_PER_CLASS):
            f.unlink()
        images = list(cls_dir.iterdir())

    random.shuffle(images)
    n = len(images)
    n_train = int(n * 0.70)
    n_val   = int(n * 0.15)

    for dst, imgs in [
        (PLANT_TRAIN, images[:n_train]),
        (PLANT_VAL,   images[n_train:n_train + n_val]),
        (PLANT_TEST,  images[n_train + n_val:]),
    ]:
        (dst / cls_dir.name).mkdir(exist_ok=True)
        for img in imgs:
            shutil.copy2(img, dst / cls_dir.name / img.name)

    final_classes.append(cls_dir.name)
    print(f"   {cls_dir.name}: {n_train} train / {n_val} val / {n - n_train - n_val} test")

final_classes = sorted(final_classes)
CLASS_TO_IDX  = {c: i for i, c in enumerate(final_classes)}
IDX_TO_CLASS  = {str(i): c for c, i in CLASS_TO_IDX.items()}

out_json = BASE_DIR / "plant_class_index.json"
with open(out_json, "w") as f:
    json.dump({"class_to_idx": CLASS_TO_IDX, "idx_to_class": IDX_TO_CLASS}, f, indent=2)

print(f"\n {len(final_classes)} final classes saved to {out_json}")
print("\nCorn classes found:")
for c in final_classes:
    if c.startswith("Corn"):
        print(f"  [{CLASS_TO_IDX[c]}] {c}")

print(f"\n Plant dataset ready at: {PLANT_SPLIT}")
print("Run: python 03_prepare_bird_dataset.py")
