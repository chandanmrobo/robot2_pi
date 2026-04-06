uestuser@mars-4090-22:~/bird_model_make/code_files$ python3 01_download_datasets.py
============================================================
Downloading ALL Datasets
Saving to: /home/guestuser/agri_robot_ml/datasets
============================================================

🌿 [1/5] PlantVillage (54k images, 38 classes)...
  $ kaggle datasets download -d abdallahalidev/plantvillage-dataset -p /home/guestuser/agri_robot_ml/datasets --unzip -q
Dataset URL: https://www.kaggle.com/datasets/abdallahalidev/plantvillage-dataset
License(s): CC-BY-NC-SA-4.0

🌽 [2/5] Kaggle Corn Disease Dataset...
  $ kaggle datasets download -d smaranjitghose/corn-or-maize-leaf-disease-dataset -p /home/guestuser/agri_robot_ml/datasets --unzip -q
Dataset URL: https://www.kaggle.com/datasets/smaranjitghose/corn-or-maize-leaf-disease-dataset
License(s): copyright-authors

🌽 [3/5] Corn Leaf Infection Dataset...
  $ kaggle datasets download -d qramkrishna/corn-leaf-infection-dataset -p /home/guestuser/agri_robot_ml/datasets --unzip -q
Dataset URL: https://www.kaggle.com/datasets/qramkrishna/corn-leaf-infection-dataset
License(s): copyright-authors

🌿 [4/5] PlantDoc (real field conditions)...
  $ git clone https://github.com/pratikkayal/PlantDoc-Dataset /home/guestuser/agri_robot_ml/datasets/PlantDoc
Cloning into '/home/guestuser/agri_robot_ml/datasets/PlantDoc'...
warning: templates not found in /usr/share/git-core/templates
git: 'remote-https' is not a git command. See 'git --help'.

🐦 [5/5] Bird datasets via FiftyOne...

  Downloading Open Images birds...
Downloading split 'train' to '/home/guestuser/fiftyone/open-images-v7/train' if necessary
Downloading 'https://storage.googleapis.com/openimages/2018_04/train/train-images-boxable-with-rotation.csv' to '/home/guestuser/fiftyone/open-images-v7/train/metadata/image_ids.csv'
 100% |████████████████████████████████████████████████████████████████████████████████████████████████████████|    4.8Gb/4.8Gb [1.0m elapsed, 0s remaining, 118.9Mb/s]      
Downloading 'https://storage.googleapis.com/openimages/v5/class-descriptions-boxable.csv' to '/home/guestuser/fiftyone/open-images-v7/train/metadata/classes.csv'
Downloading 'https://storage.googleapis.com/openimages/2018_04/bbox_labels_600_hierarchy.json' to '/tmp/tmpwgv4rdyn/metadata/hierarchy.json'
Downloading 'https://storage.googleapis.com/openimages/v6/oidv6-train-annotations-bbox.csv' to '/home/guestuser/fiftyone/open-images-v7/train/labels/detections.csv'
Downloading 2500 images
 100% |█████████████████████████████████████████████████████████████████████████████████████████████████████████| 2500/2500 [2.5m elapsed, 0s remaining, 5.2 files/s]       
Dataset info written to '/home/guestuser/fiftyone/open-images-v7/info.json'
Loading 'open-images-v7' split 'train'
 100% |███████████████████████████████████████████████████████████████████████████████████████████████████████| 2500/2500 [3.5s elapsed, 0s remaining, 736.6 samples/s]      
Dataset 'birds_open_images' created
    Label field: 'ground_truth'
 100% |███████████████████████████████████████████████████████████████████████████████████████████████████████| 2500/2500 [1.9s elapsed, 0s remaining, 1.4K samples/s]         
  ✅ Open Images birds: 2500 images

  Downloading COCO birds...
Downloading split 'train' to '/home/guestuser/fiftyone/coco-2017/train' if necessary
Downloading annotations to '/home/guestuser/fiftyone/coco-2017/tmp-download/annotations_trainval2017.zip'
 100% |████████████████████████████████████████████████████████████████████████████████████████████████████████|    1.9Gb/1.9Gb [3.4m elapsed, 0s remaining, 11.8Mb/s]        
Extracting annotations to '/home/guestuser/fiftyone/coco-2017/raw/instances_train2017.json'
Downloading 1500 images
   8% |████████|-----------------------------------------------------------------------------------------------|  120/1500 [46.0s elapsed, 8.2m remaining, 16.6 images/s] 
  ⚠️ COCO birds failed: ('Connection aborted.', ConnectionResetError(104, 'Connection reset by peer'))

  Downloading no-bird sky scenes...
Downloading split 'train' to '/home/guestuser/fiftyone/coco-2017/train' if necessary
Found annotations at '/home/guestuser/fiftyone/coco-2017/raw/instances_train2017.json'
128 images found; downloading the remaining 1872
  78% |████████████████████████████████████████████████████████████████████████████████\-----------------------| 1454/1872 [3.5m elapsed, 8.1m remaining, 0.7 images/s]   
  ⚠️ Sky scenes failed: HTTPConnectionPool(host='images.cocodataset.org', port=80): Max retries exceeded with url: /train2017/000000007729.jpg (Caused by NewConnectionError('<urllib3.connection.HTTPConnection object at 0x77d5f1be85e0>: Failed to establish a new connection: [Errno -3] Temporary failure in name resolution'))

  Downloading hard negatives (kites/planes)...
Downloading split 'train' to '/home/guestuser/fiftyone/coco-2017/train' if necessary
Found annotations at '/home/guestuser/fiftyone/coco-2017/raw/instances_train2017.json'
78 images found; downloading the remaining 722
 100% |██████████████████████████████████████████████████████████████████████████████████████████████████████████| 722/722 [1.6m elapsed, 0s remaining, 14.3 images/s]      
Writing annotations for 2351 downloaded samples to '/home/guestuser/fiftyone/coco-2017/train/labels.json'
Dataset info written to '/home/guestuser/fiftyone/coco-2017/info.json'
Loading 'coco-2017' split 'train'
 100% |█████████████████████████████████████████████████████████████████████████████████████████████████████████| 800/800 [1.7s elapsed, 0s remaining, 467.5 samples/s]         
Dataset 'kites_airplanes' created
 100% |█████████████████████████████████████████████████████████████████████████████████████████████████████████| 800/800 [864.4ms elapsed, 0s remaining, 925.4 samples/s]      
  ✅ Hard negatives: 800 images

============================================================
DOWNLOAD SUMMARY
============================================================
  Datasets folder: /home/guestuser/agri_robot_ml/datasets

  Directories downloaded:
    Corn Disease detection: ~4225 images
    birds_open_images: ~2500 images
    data: ~1658 images
    hard_negatives_bird: ~800 images
    plantvillage dataset: ~57308 images

✅ Downloads complete.

NEXT STEPS:
  1. Add your custom field photos:
     Corn healthy/soil    → datasets/custom_plants/Corn_Healthy/
     Corn diseased        → datasets/custom_plants/Corn_Disease/
     Birds in field       → datasets/custom_birds/bird/
     Empty sky            → datasets/custom_birds/no_bird/
     Indian pest birds    → manually from https://www.inaturalist.org

  2. Run: python 02_prepare_plant_dataset.py

guestuser@mars-4090-22:~/bird_model_make/code_files$ 


guestuser@mars-4090-22:~/bird_model_make/code_files$ python3 02_prepare_plant_dataset.py
============================================================
Processing PlantVillage...
  PlantVillage done

Processing Kaggle Corn datasets...

Processing PlantDoc...
   PlantDoc done

Processing custom field photos...
   No custom_plants folder — add field photos later

============================================================
Class counts (before balancing):
  Apple_BlackRot                               :   621
  Apple_CedarRust                              :   275
  Apple_Healthy                                :  1645
  Apple_Scab                                   :   630
  Blueberry_Healthy                            :  1502
  Cherry_Healthy                               :   854
  Cherry_PowderyMildew                         :  1052
  Corn_CommonRust                              :  1192
  Corn_GrayLeafSpot                            :   513
  Corn_Healthy                                 :  1162
  Corn_NorthernBlight                          :   985
  Grape_BlackMeasles                           :  1384
  Grape_BlackRot                               :  1180
  Grape_Healthy                                :   423
  Grape_LeafBlight                             :  1076
  Orange_CitrusGreening                        :  5507
  Peach_BacterialSpot                          :  2297
  Peach_Healthy                                :   360
  Pepper_BacterialSpot                         :   997
  Pepper_Healthy                               :  1478
  Potato_EarlyBlight                           :  1000
  Potato_Healthy                               :   152
  Potato_LateBlight                            :  1000
  Raspberry_Healthy                            :   371
  Soybean_Healthy                              :  5090
  Squash_PowderyMildew                         :  1835
  Strawberry_Healthy                           :   456
  Strawberry_LeafScorch                        :  1109
  Tomato_BacterialSpot                         :  2127
  Tomato_EarlyBlight                           :  1000
  Tomato_Healthy                               :  1591
  Tomato_LateBlight                            :  1909
  Tomato_LeafMold                              :   952
  Tomato_MosaicVirus                           :   373
  Tomato_SeptoriaLeafSpot                      :  1771
  Tomato_SpiderMites                           :  1676
  Tomato_TargetSpot                            :  1404
  Tomato_YellowCurlVirus                       :  5357
  TOTAL                                        : 54306
  Corrupt skipped: 0

============================================================
Balancing classes and creating train/val/test split...
  Max per class: 2000 | Min per class: 50
   Apple_BlackRot: 434 train / 93 val / 94 test
   Apple_CedarRust: 192 train / 41 val / 42 test
   Apple_Healthy: 1151 train / 246 val / 248 test
   Apple_Scab: 441 train / 94 val / 95 test
   Blueberry_Healthy: 1051 train / 225 val / 226 test
   Cherry_Healthy: 597 train / 128 val / 129 test
   Cherry_PowderyMildew: 736 train / 157 val / 159 test
   Corn_CommonRust: 834 train / 178 val / 180 test
   Corn_GrayLeafSpot: 359 train / 76 val / 78 test
   Corn_Healthy: 813 train / 174 val / 175 test
   Corn_NorthernBlight: 689 train / 147 val / 149 test
   Grape_BlackMeasles: 968 train / 207 val / 209 test
   Grape_BlackRot: 826 train / 177 val / 177 test
   Grape_Healthy: 296 train / 63 val / 64 test
   Grape_LeafBlight: 753 train / 161 val / 162 test
   Orange_CitrusGreening: 1400 train / 300 val / 300 test
   Peach_BacterialSpot: 1400 train / 300 val / 300 test
   Peach_Healthy: 251 train / 54 val / 55 test
   Pepper_BacterialSpot: 697 train / 149 val / 151 test
   Pepper_Healthy: 1034 train / 221 val / 223 test
   Potato_EarlyBlight: 700 train / 150 val / 150 test
   Potato_Healthy: 106 train / 22 val / 24 test
   Potato_LateBlight: 700 train / 150 val / 150 test
   Raspberry_Healthy: 259 train / 55 val / 57 test
   Soybean_Healthy: 1400 train / 300 val / 300 test
   Squash_PowderyMildew: 1284 train / 275 val / 276 test
   Strawberry_Healthy: 319 train / 68 val / 69 test
   Strawberry_LeafScorch: 776 train / 166 val / 167 test
   Tomato_BacterialSpot: 1400 train / 300 val / 300 test
   Tomato_EarlyBlight: 700 train / 150 val / 150 test
   Tomato_Healthy: 1113 train / 238 val / 240 test
   Tomato_LateBlight: 1336 train / 286 val / 287 test
   Tomato_LeafMold: 666 train / 142 val / 144 test
   Tomato_MosaicVirus: 261 train / 55 val / 57 test
   Tomato_SeptoriaLeafSpot: 1239 train / 265 val / 267 test
   Tomato_SpiderMites: 1173 train / 251 val / 252 test
   Tomato_TargetSpot: 982 train / 210 val / 212 test
   Tomato_YellowCurlVirus: 1400 train / 300 val / 300 test

 38 final classes saved to /home/guestuser/agri_robot_ml/plant_class_index.json

Corn classes found:
  [7] Corn_CommonRust
  [8] Corn_GrayLeafSpot
  [9] Corn_Healthy
  [10] Corn_NorthernBlight

 Plant dataset ready at: /home/guestuser/agri_robot_ml/datasets/plant_split
Run: python 03_prepare_bird_dataset.py
guestuser@mars-4090-22:~/bird_model_make/code_files$ 
