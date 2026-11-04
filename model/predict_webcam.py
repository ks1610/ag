import cv2, threading, time, queue
from collections import deque

import numpy as np
import torch
import open_clip
from PIL import Image
from ultralytics import YOLO

# ------------------------------------------------------------------ settings
MODEL_PATH = "/home/khanhtrinh/trinh/ag/model/cardbox_train/weights/best.pt"
CONF = 0.4              # YOLO threshold (can be lower now, CLIP filters false positives)
SURE_CONF = 0.9         # >= this skips CLIP. 1.01 = always verify (use for testing).
CLIP_THRESH = 0.60      # CLIP probability needed to accept a box
RECHECK_FRAMES = 60     # re-verify each track every N frames (its look can change)
CONFIRM_FRAMES = 3      # must be accepted in 3 of the last 5 frames
SHOW_REJECTED = True    # draw rejected boxes in red (useful while tuning)
IMGSZ = 320

# brightness handling
ENHANCE = True          # automatically brighten dark frames before YOLO and CLIP
TARGET_BRIGHTNESS = 110 # mean gray level (0-255) the enhancer aims for
SHOW_BRIGHTNESS = True  # print the measured brightness on screen (for tuning)

DEVICE = "cuda" if torch.cuda.is_available() else "cpu"
print("Device:", DEVICE)

# ------------------------------------------------------------------ models
det = YOLO(MODEL_PATH)

clip_model, _, preprocess = open_clip.create_model_and_transforms(
    "ViT-B-32", pretrained="laion2b_s34b_b79k")
clip_model = clip_model.to(DEVICE).eval()
tokenizer = open_clip.get_tokenizer("ViT-B-32")

POSITIVE = [
    "a photo of a cardboard box",
    "a photo of a brown cardboard package",
]
NEGATIVE = [
    "a photo of a wooden door",
    "a photo of a picture frame",
    "a photo of a wall",
    "a photo of wooden furniture",
    "a photo of a table or floor",
    "a photo of a window",
    "a photo of a cabinet",
]
LABELS = POSITIVE + NEGATIVE

with torch.no_grad():
    text_feat = clip_model.encode_text(tokenizer(LABELS).to(DEVICE))
    text_feat /= text_feat.norm(dim=-1, keepdim=True)


def clip_box_prob(crop_bgr):
    """Probability that the crop is a cardboard box (sum of positive prompts)."""
    img = preprocess(Image.fromarray(cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2RGB)))
    img = img.unsqueeze(0).to(DEVICE)
    with torch.no_grad():
        f = clip_model.encode_image(img)
        f /= f.norm(dim=-1, keepdim=True)
        probs = (100.0 * f @ text_feat.T).softmax(dim=-1)[0]
    return probs[:len(POSITIVE)].sum().item()


# ------------------------------------------------------------------ brightness normalization
_clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
_smooth_mean = None     # smoothed frame brightness, avoids flicker


def normalize_brightness(frame, target=TARGET_BRIGHTNESS):
    """Brighten dark frames (gamma + CLAHE on lightness). Normal frames pass through.
    Returns (frame, smoothed_mean_brightness_of_the_original)."""
    global _smooth_mean
    mean = float(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY).mean())
    _smooth_mean = mean if _smooth_mean is None else 0.9 * _smooth_mean + 0.1 * mean

    if _smooth_mean >= target - 15:
        return frame, _smooth_mean

    # gamma that maps the current mean brightness to the target
    gamma = np.log(target / 255) / np.log(max(_smooth_mean, 1) / 255)
    gamma = float(np.clip(gamma, 0.35, 1.0))
    lut = (255 * (np.arange(256) / 255) ** gamma).astype(np.uint8)
    out = cv2.LUT(frame, lut)

    # local contrast on the L channel only, so colors are preserved
    lab = cv2.cvtColor(out, cv2.COLOR_BGR2LAB)
    lab[:, :, 0] = _clahe.apply(lab[:, :, 0])
    return cv2.cvtColor(lab, cv2.COLOR_LAB2BGR), _smooth_mean


# ------------------------------------------------------------------ CLIP worker thread
# CLIP runs in the background so the video never freezes while it thinks.
jobs = queue.Queue(maxsize=4)
verdicts = {}        # track_id -> {"p": float, "frame": int}
pending = set()      # track ids currently queued or being checked


def clip_worker():
    while True:
        tid, crop, fid = jobs.get()
        try:
            p = clip_box_prob(crop)
            old = verdicts.get(tid)
            if old is not None:                 # smooth across re-checks
                p = 0.5 * old["p"] + 0.5 * p
            verdicts[tid] = {"p": p, "frame": fid}
        except Exception as e:
            print("CLIP error:", e)
        finally:
            pending.discard(tid)


threading.Thread(target=clip_worker, daemon=True).start()


# ------------------------------------------------------------------ camera thread
class Camera:
    def __init__(self, src=0):
        self.cap = cv2.VideoCapture(src)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.frame, self.ok = None, True
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while self.ok:
            ok, f = self.cap.read()
            if ok:
                self.frame = f

    def read(self):
        return self.frame

    def release(self):
        self.ok = False
        self.cap.release()


def get_crop(frame, box, pad=0.05):
    h, w = frame.shape[:2]
    x1, y1, x2, y2 = box
    px, py = int((x2 - x1) * pad), int((y2 - y1) * pad)
    x1, y1 = max(int(x1) - px, 0), max(int(y1) - py, 0)
    x2, y2 = min(int(x2) + px, w), min(int(y2) + py, h)
    crop = frame[y1:y2, x1:x2]
    return crop if crop.shape[0] >= 24 and crop.shape[1] >= 24 else None


# ------------------------------------------------------------------ main loop
cam = Camera(0)
history = deque(maxlen=5)
last_seen = {}       # track_id -> last frame it was visible (for cleanup)
frame_id = 0
prev = time.time()
brightness = 0.0

while True:
    frame = cam.read()
    if frame is None:
        continue
    frame_id += 1

    # brighten dark scenes; YOLO and CLIP both see the enhanced frame
    if ENHANCE:
        frame, brightness = normalize_brightness(frame)

    r = det.track(frame, persist=True, conf=CONF, imgsz=IMGSZ,
                  tracker="bytetrack.yaml", verbose=False)[0]

    found = False
    if r.boxes is not None and r.boxes.id is not None:
        boxes = r.boxes.xyxy.cpu().numpy()
        ids = r.boxes.id.int().cpu().tolist()
        confs = r.boxes.conf.cpu().numpy()

        for box, tid, conf in zip(boxes, ids, confs):
            last_seen[tid] = frame_id
            v = verdicts.get(tid)

            # ask CLIP when: new track, or verdict is old (and not above SURE_CONF)
            if conf < SURE_CONF:
                stale = v is None or (frame_id - v["frame"]) > RECHECK_FRAMES
                if stale and tid not in pending and not jobs.full():
                    crop = get_crop(frame, box)
                    if crop is not None:
                        pending.add(tid)
                        jobs.put((tid, crop.copy(), frame_id))

            # decide the state of this box
            if conf >= SURE_CONF:
                state, text = "ok", f"cardboard {conf:.2f}"
            elif v is None:
                state, text = "checking", f"checking... det={conf:.2f}"
            elif v["p"] >= CLIP_THRESH:
                state, text = "ok", f"cardboard det={conf:.2f} clip={v['p']:.2f}"
            else:
                state, text = "rejected", f"reject det={conf:.2f} clip={v['p']:.2f}"

            if state == "ok":
                found = True
                color = (0, 255, 0)
            elif state == "checking":
                color = (0, 255, 255)
            else:
                color = (0, 0, 255)
                if not SHOW_REJECTED:
                    continue

            x1, y1, x2, y2 = map(int, box)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(frame, f"#{tid} {text}", (x1, max(y1 - 6, 12)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

    # forget tracks that disappeared long ago
    if frame_id % 100 == 0:
        for tid in [t for t, f in last_seen.items() if frame_id - f > 150]:
            last_seen.pop(tid, None)
            verdicts.pop(tid, None)

    history.append(found)
    confirmed = sum(history) >= CONFIRM_FRAMES
    status = "BOX DETECTED" if confirmed else "searching..."

    now = time.time()
    fps = 1 / max(now - prev, 1e-6)
    prev = now
    cv2.putText(frame, f"{status}  {fps:.1f} FPS", (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
    if SHOW_BRIGHTNESS:
        boosted = ENHANCE and brightness < TARGET_BRIGHTNESS - 15
        cv2.putText(frame, f"brightness {brightness:.0f}  {'(boosted)' if boosted else ''}",
                    (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

    cv2.imshow("Cardboard", frame)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cam.release()
cv2.destroyAllWindows()