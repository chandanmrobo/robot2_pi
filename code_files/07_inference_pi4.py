#!/usr/bin/env python3
"""
07_inference_pi4.py — Updated
Pi4 inference script for Robot 2.
- Plant : EfficientNetB4 PyTorch (.pth)
- Bird  : YOLOv5n TFLite INT8 (.tflite)
- Conf threshold raised to 0.72 to fix kite/plane false alarms
- Multi-frame voting for bird — needs 2 detections in 5 frames before alerting
- Model swapping on mode change (3-4 sec delay acceptable)
- Servo pause during camera tilt (no false detections)
Run this on Pi4 (Ubuntu 22.04, ROS2 Humble environment)
"""

import cv2
import json
import time
import threading
import numpy as np
from pathlib import Path
from collections import deque

# ─────────────────────────────────────────────────────────────
# PATHS (on Pi4 after transfer)
# ─────────────────────────────────────────────────────────────
MODEL_DIR = Path.home() / "agri_models"

PLANT_MODEL_PATH = MODEL_DIR / "plant_model.pth"
BIRD_MODEL_PATH  = MODEL_DIR / "bird_model.tflite"
CLASS_JSON       = MODEL_DIR / "plant_class_index.json"

IMG_SIZE = 224    # Plant (EfficientNetB4)
BIRD_IMG = 640    # Bird (YOLOv5n)

# ─────────────────────────────────────────────────────────────
# CONFIDENCE THRESHOLDS — tuned to fix false positives
# ─────────────────────────────────────────────────────────────
BIRD_CONF_THRESHOLD  = 0.72   # Raised from 0.5 — filters kites/planes
PLANT_CONF_THRESHOLD = 0.60   # Minimum confidence to report disease
BIRD_VOTE_WINDOW     = 5      # Frames to check
BIRD_VOTE_MIN        = 2      # Min detections in window to confirm bird

# ─────────────────────────────────────────────────────────────
# LOAD PLANT MODEL
# ─────────────────────────────────────────────────────────────
def load_plant_model():
    """Load EfficientNetB4 for plant disease classification."""
    import torch
    import timm
    import albumentations as A
    from albumentations.pytorch import ToTensorV2

    with open(CLASS_JSON) as f:
        data = json.load(f)
    class_to_idx  = data["class_to_idx"]
    idx_to_class  = data["idx_to_class"]
    final_classes = sorted(class_to_idx.keys())

    device = torch.device("cpu")   # Pi4 — CPU only
    model  = timm.create_model("efficientnet_b4", pretrained=False, num_classes=len(final_classes))
    ckpt   = torch.load(PLANT_MODEL_PATH, map_location="cpu", weights_only=False)
    model.load_state_dict(ckpt["model_state_dict"])
    model.eval()

    inf_tf = A.Compose([
        A.SmallestMaxSize(256),
        A.CenterCrop(IMG_SIZE, IMG_SIZE),
        A.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
        ToTensorV2(),
    ])

    def predict(frame_rgb):
        """Returns (class_name, confidence, top3)"""
        import torch.nn.functional as F
        img   = inf_tf(image=frame_rgb)["image"].unsqueeze(0)
        with torch.no_grad():
            probs = F.softmax(model(img), dim=1)[0]
        top3_p, top3_i = probs.topk(3)
        label = idx_to_class[str(top3_i[0].item())]
        conf  = top3_p[0].item()
        top3  = [(idx_to_class[str(top3_i[i].item())], float(top3_p[i])) for i in range(3)]
        return label, conf, top3

    print(f"Plant model loaded ({len(final_classes)} classes)")
    return predict

# ─────────────────────────────────────────────────────────────
# LOAD BIRD MODEL
# ─────────────────────────────────────────────────────────────
def load_bird_model():
    """Load YOLOv5n TFLite INT8 for bird detection."""
    import tensorflow as tf

    interpreter = tf.lite.Interpreter(
        model_path=str(BIRD_MODEL_PATH),
        experimental_delegates=[]
    )
    interpreter.allocate_tensors()
    input_details  = interpreter.get_input_details()
    output_details = interpreter.get_output_details()

    scale, zero_point = input_details[0]["quantization"]
    input_scale       = scale if scale else 1.0

    def preprocess(frame_bgr):
        img = cv2.resize(frame_bgr, (BIRD_IMG, BIRD_IMG))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = img.astype(np.float32) / 255.0
        return np.expand_dims(img, 0).astype(np.float32)

    def predict(frame_bgr):
        """Returns (detected: bool, confidence: float, boxes: list)"""
        inp = preprocess(frame_bgr)
        if input_details[0]["dtype"] == np.uint8:
            inp = (inp / input_scale + zero_point).astype(np.uint8)
        interpreter.set_tensor(input_details[0]["index"], inp)
        interpreter.invoke()
        output     = interpreter.get_tensor(output_details[0]["index"])
        detections = output[0]

        # Apply higher confidence threshold to reduce false positives
        conf_mask  = detections[:, 4] > BIRD_CONF_THRESHOLD
        bird_dets  = detections[conf_mask]

        # Only keep class 0 (Bird) detections
        if len(bird_dets) > 0:
            cls_mask  = bird_dets[:, 5] < bird_dets[:, 6]  # cls0 score > cls1 score
            bird_dets = bird_dets[cls_mask]

        detected = len(bird_dets) > 0
        conf     = float(bird_dets[:, 4].max()) if detected else 0.0
        return detected, conf, bird_dets.tolist()

    print("Bird TFLite model loaded")
    return predict

# ─────────────────────────────────────────────────────────────
# MULTI-FRAME VOTER — reduces false alarms further
# ─────────────────────────────────────────────────────────────
class BirdVoter:
    """
    Requires BIRD_VOTE_MIN detections in last BIRD_VOTE_WINDOW frames
    before confirming bird presence.
    This eliminates single-frame false positives from kites/planes.
    """
    def __init__(self):
        self.history = deque(maxlen=BIRD_VOTE_WINDOW)

    def update(self, detected, conf):
        self.history.append((detected, conf))
        votes      = sum(1 for d, _ in self.history if d)
        confirmed  = votes >= BIRD_VOTE_MIN
        avg_conf   = np.mean([c for d, c in self.history if d]) if votes > 0 else 0.0
        return confirmed, float(avg_conf), votes

    def reset(self):
        self.history.clear()

# ─────────────────────────────────────────────────────────────
# INFERENCE ENGINE
# ─────────────────────────────────────────────────────────────
class InferenceEngine:
    """
    Handles:
    - Model loading/swapping when mode changes
    - Servo pause during camera tilt
    - Multi-frame bird voting (reduces false alarms)
    - WebRTC frame rate control
    """

    def __init__(self):
        self.mode         = "plant"
        self.predict_fn   = None
        self.is_ready     = False
        self.servo_moving = False
        self.frame_count  = 0
        self.lock         = threading.Lock()
        self.bird_voter   = BirdVoter()
        self._load_model("plant")

    def _load_model(self, mode):
        self.is_ready = False
        print(f"\n[Model] Loading {mode} model...")
        t0 = time.time()
        try:
            if mode == "plant":
                self.predict_fn = load_plant_model()
            else:
                self.predict_fn = load_bird_model()
                self.bird_voter.reset()
            print(f"[Model] Ready in {time.time()-t0:.1f}s")
            self.mode     = mode
            self.is_ready = True
        except Exception as e:
            print(f"[Model] Load failed: {e}")
            self.is_ready = False

    def switch_mode(self, new_mode, servo_controller=None):
        """
        Switch detection mode + move servo.
        Total delay ~3-4 seconds acceptable.
        """
        if new_mode == self.mode and self.is_ready:
            return

        with self.lock:
            print(f"\n[Mode] Switching: {self.mode} -> {new_mode}")
            self.is_ready     = False
            self.servo_moving = True

            if servo_controller:
                if new_mode == "bird":
                    servo_controller.set_tilt(45)    # Upward
                else:
                    servo_controller.set_tilt(-30)   # Downward toward plants

            # Servo settle + camera auto-exposure
            time.sleep(1.2)
            self.servo_moving = False

            del self.predict_fn
            import gc
            gc.collect()

            self._load_model(new_mode)

    def run_inference(self, frame_bgr):
        """
        Run detection on frame.
        Returns result dict or None if not ready / servo moving.
        """
        if not self.is_ready or self.servo_moving:
            return None

        self.frame_count += 1
        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)

        try:
            if self.mode == "plant":
                label, conf, top3 = self.predict_fn(frame_rgb)
                plant_name, *disease_parts = label.split("_")
                disease    = "_".join(disease_parts)
                is_healthy = disease.lower() == "healthy"

                # Only report if confidence above threshold
                reliable = conf >= PLANT_CONF_THRESHOLD

                return {
                    "mode":        "plant",
                    "label":       label,
                    "plant":       plant_name,
                    "disease":     disease,
                    "confidence":  conf,
                    "is_diseased": not is_healthy,
                    "is_healthy":  is_healthy,
                    "reliable":    reliable,
                    "top3":        top3,
                    "send_webrtc": (self.frame_count % 3 == 0),
                }

            else:  # bird
                detected, conf, boxes = self.predict_fn(frame_bgr)

                # Multi-frame voting to confirm bird
                confirmed, avg_conf, votes = self.bird_voter.update(detected, conf)

                return {
                    "mode":        "bird",
                    "detected":    detected,       # Raw single frame
                    "confirmed":   confirmed,      # Voted across frames — use this for alerts
                    "confidence":  conf,
                    "avg_conf":    avg_conf,
                    "votes":       votes,
                    "boxes":       boxes,
                    "send_webrtc": (self.frame_count % 3 == 0),
                }

        except Exception as e:
            print(f"[Inference] Error: {e}")
            return None


# ─────────────────────────────────────────────────────────────
# STANDALONE TEST
# ─────────────────────────────────────────────────────────────
def test_with_webcam():
    """Test inference with Pi camera / webcam."""
    engine = InferenceEngine()

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Camera not found")
        return

    print("\nCamera running. Press:")
    print("  'p' = switch to plant mode")
    print("  'b' = switch to bird mode")
    print("  'q' = quit")

    fps_times = []

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        t0     = time.time()
        result = engine.run_inference(frame)

        if result:
            fps_times.append(time.time() - t0)
            fps = 1.0 / (sum(fps_times[-10:]) / min(len(fps_times), 10))

            if result["mode"] == "plant":
                conf_pct = result["confidence"] * 100
                if result["reliable"]:
                    label = f"Plant: {result['label']} ({conf_pct:.0f}%)"
                    color = (0, 0, 255) if result["is_diseased"] else (0, 255, 0)
                else:
                    label = f"Plant: Low confidence ({conf_pct:.0f}%) - move closer"
                    color = (0, 165, 255)  # Orange

                # Show top 3 predictions
                for i, (cls, p) in enumerate(result["top3"]):
                    cv2.putText(frame, f"  #{i+1} {cls}: {p*100:.0f}%",
                                (10, 110 + i*20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)

            else:  # bird
                conf_pct = result["avg_conf"] * 100
                # Use confirmed (voted) result for display
                if result["confirmed"]:
                    label = f"BIRD DETECTED! ({conf_pct:.0f}%) [{result['votes']}/{BIRD_VOTE_WINDOW} frames]"
                    color = (0, 0, 255)
                    # Draw bounding boxes
                    for box in result["boxes"]:
                        x1, y1, x2, y2 = int(box[0]), int(box[1]), int(box[2]), int(box[3])
                        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                elif result["detected"]:
                    label = f"Possible bird... ({result['confidence']*100:.0f}%) [{result['votes']}/{BIRD_VOTE_WINDOW}]"
                    color = (0, 165, 255)  # Orange — not confirmed yet
                else:
                    label = f"Sky clear ({result['votes']}/{BIRD_VOTE_WINDOW} detections)"
                    color = (0, 255, 0)

            cv2.putText(frame, label, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
            cv2.putText(frame, f"Mode: {result['mode']} | {fps:.1f} FPS",
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
            cv2.putText(frame, f"WebRTC: {'send' if result['send_webrtc'] else 'skip'}",
                        (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)

            print(f"\r  [{result['mode']:5s}] {label:65s} | {fps:.1f} FPS", end="")

        elif engine.servo_moving:
            cv2.putText(frame, "Switching mode...", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 165, 255), 2)

        cv2.imshow("Agri Robot Detection", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        elif key == ord("p"):
            threading.Thread(target=engine.switch_mode, args=("plant",), daemon=True).start()
        elif key == ord("b"):
            threading.Thread(target=engine.switch_mode, args=("bird",), daemon=True).start()

    cap.release()
    cv2.destroyAllWindows()
    print("\nDone")


if __name__ == "__main__":
    if not PLANT_MODEL_PATH.exists():
        print(f"Plant model not found: {PLANT_MODEL_PATH}")
        print("Transfer deploy/ folder from training PC first")
    elif not BIRD_MODEL_PATH.exists():
        print(f"Bird TFLite not found: {BIRD_MODEL_PATH}")
        print("Transfer deploy/ folder from training PC first")
    else:
        test_with_webcam()
