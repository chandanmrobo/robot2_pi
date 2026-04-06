#!/usr/bin/env python3
"""
01_download_datasets.py
Downloads all datasets for plant disease + bird detection.
Run on PC with internet. All saved to ~/agri_robot_ml/datasets/
"""

import os, subprocess, sys
from pathlib import Path

BASE_DIR    = Path.home() / "agri_robot_ml"
DATASET_DIR = BASE_DIR / "datasets"
DATASET_DIR.mkdir(parents=True, exist_ok=True)

def run(cmd, cwd=None):
    print(f"  $ {cmd}")
    subprocess.run(cmd, shell=True, cwd=cwd)

print("=" * 60)
print("Downloading ALL Datasets")
print(f"Saving to: {DATASET_DIR}")
print("=" * 60)

# ─────────────────────────────────────────────────────────────
# PLANT DATASETS
# ─────────────────────────────────────────────────────────────
print("\n🌿 [1/5] PlantVillage (54k images, 38 classes)...")
run(f"kaggle datasets download -d abdallahalidev/plantvillage-dataset -p {DATASET_DIR} --unzip -q")

print("\n🌽 [2/5] Kaggle Corn Disease Dataset...")
run(f"kaggle datasets download -d smaranjitghose/corn-or-maize-leaf-disease-dataset -p {DATASET_DIR} --unzip -q")

print("\n🌽 [3/5] Corn Leaf Infection Dataset...")
run(f"kaggle datasets download -d qramkrishna/corn-leaf-infection-dataset -p {DATASET_DIR} --unzip -q")

print("\n🌿 [4/5] PlantDoc (real field conditions)...")
plantdoc = DATASET_DIR / "PlantDoc"
if not plantdoc.exists():
    run(f"git clone https://github.com/pratikkayal/PlantDoc-Dataset {plantdoc}")
else:
    print("  Already exists, skipping.")

# ─────────────────────────────────────────────────────────────
# BIRD DATASETS (via FiftyOne)
# ─────────────────────────────────────────────────────────────
print("\n🐦 [5/5] Bird datasets via FiftyOne...")

import fiftyone as fo
import fiftyone.zoo as foz

def get_label_field(dataset):
    """Auto-detect label field name."""
    sample = dataset.first()
    for field in ["detections", "ground_truth", "predictions", "labels"]:
        try:
            if sample.has_field(field):
                print(f"    Label field: '{field}'")
                return field
        except:
            pass
    for fname, ftype in dataset.get_field_schema().items():
        if "Detection" in str(ftype):
            print(f"    Label field: '{fname}'")
            return fname
    raise ValueError(f"No detection field found. Fields: {list(dataset.get_field_schema().keys())}")

# --- Open Images (birds: flying, distant, sky) ---
print("\n  Downloading Open Images birds...")
try:
    ds_oi = foz.load_zoo_dataset(
        "open-images-v7", split="train",
        label_types=["detections"],
        classes=["Bird"],
        max_samples=2500,
        dataset_name="birds_open_images"
    )
    oi_field = get_label_field(ds_oi)
    ds_oi.export(
        export_dir=str(DATASET_DIR / "birds_open_images"),
        dataset_type=fo.types.YOLOv5Dataset,
        label_field=oi_field
    )
    print(f"  ✅ Open Images birds: {len(ds_oi)} images")
except Exception as e:
    print(f"  ⚠️ Open Images failed: {e}")

# --- COCO birds ---
print("\n  Downloading COCO birds...")
try:
    ds_coco = foz.load_zoo_dataset(
        "coco-2017", split="train",
        label_types=["detections"],
        classes=["bird"],
        max_samples=1500,
        dataset_name="birds_coco"
    )
    coco_field = get_label_field(ds_coco)
    ds_coco.export(
        export_dir=str(DATASET_DIR / "birds_coco"),
        dataset_type=fo.types.YOLOv5Dataset,
        label_field=coco_field
    )
    print(f"  ✅ COCO birds: {len(ds_coco)} images")
except Exception as e:
    print(f"  ⚠️ COCO birds failed: {e}")

# --- Sky / no-bird scenes ---
print("\n  Downloading no-bird sky scenes...")
try:
    ds_sky = foz.load_zoo_dataset(
        "coco-2017", split="train",
        max_samples=2000,
        dataset_name="sky_scenes"
    )
    sky_no_bird = ds_sky.match(
        ~fo.ViewField("detections.detections.label").contains("bird")
    )
    sky_no_bird.export(
        export_dir=str(DATASET_DIR / "sky_no_birds"),
        dataset_type=fo.types.ImageDirectory
    )
    print(f"  ✅ Sky scenes (no birds): {len(sky_no_bird)} images")
except Exception as e:
    print(f"  ⚠️ Sky scenes failed: {e}")

# --- Hard negatives: kites + planes (confusion prevention) ---
print("\n  Downloading hard negatives (kites/planes)...")
try:
    ds_hard = foz.load_zoo_dataset(
        "coco-2017", split="train",
        label_types=["detections"],
        classes=["kite", "airplane"],
        max_samples=800,
        dataset_name="kites_airplanes"
    )
    ds_hard.export(
        export_dir=str(DATASET_DIR / "hard_negatives_bird"),
        dataset_type=fo.types.ImageDirectory
    )
    print(f"  ✅ Hard negatives: {len(ds_hard)} images")
except Exception as e:
    print(f"  ⚠️ Hard negatives failed: {e}")

# ─────────────────────────────────────────────────────────────
# SUMMARY
# ─────────────────────────────────────────────────────────────
print("\n" + "=" * 60)
print("DOWNLOAD SUMMARY")
print("=" * 60)
print(f"  Datasets folder: {DATASET_DIR}")
print("\n  Directories downloaded:")
for d in sorted(DATASET_DIR.iterdir()):
    if d.is_dir():
        count = sum(1 for _ in d.rglob("*.jpg")) + sum(1 for _ in d.rglob("*.png"))
        print(f"    {d.name}: ~{count} images")

print("""
✅ Downloads complete.

NEXT STEPS:
  1. Add your custom field photos:
     Corn healthy/soil    → datasets/custom_plants/Corn_Healthy/
     Corn diseased        → datasets/custom_plants/Corn_Disease/
     Birds in field       → datasets/custom_birds/bird/
     Empty sky            → datasets/custom_birds/no_bird/
     Indian pest birds    → manually from https://www.inaturalist.org

  2. Run: python 02_prepare_plant_dataset.py
""")
