#!/usr/bin/env python3
"""
SURDAS Assistive AI Perception Engine - High Performance Edge Architecture
Optimized for Qualcomm QRB2210 (ARM Cortex-A53 quad-core, Arduino UNO Q)
- Throttled pedestrian-paced inference (2.5 FPS)
- CPU-efficient ONNX execution with 2 threads max
- Real-time YOLOv8 object detection + periodic MiDaS depth clearance
"""

import os
import cv2
import numpy as np
import onnxruntime as ort
import time
import threading

COCO_CLASSES = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat",
    "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
    "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack",
    "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball",
    "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket",
    "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
    "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake",
    "chair", "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop",
    "mouse", "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"
]

ALERT_CLASSES = {
    "person", "bicycle", "car", "motorcycle", "bus", "truck", "stop sign", "traffic light",
    "chair", "couch", "dining table", "bed", "toilet", "bottle", "door", "bench"
}

class AIVisionEngine:
    def __init__(self, models_dir="models"):
        self.models_dir = models_dir
        self.yolo_path = os.path.join(models_dir, "yolov8n.onnx")
        self.midas_path = os.path.join(models_dir, "midas_small.onnx")

        self.yolo_session = None
        self.midas_session = None
        self.camera = None
        self.running = False

        self.latest_frame = None
        self.latest_depth_map = None
        self.latest_detections = []
        self.latest_guidance = "SURDAS vision active. Path is clear."
        self.spatial_sectors = {"left": 0.0, "center": 0.0, "right": 0.0}

        self.lock = threading.Lock()
        self.load_models()

    def load_models(self):
        opts = ort.SessionOptions()
        opts.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        # Crucial for embedded SBC: limit to 2 threads so 2 CPU cores remain free for I/O & web
        opts.intra_op_num_threads = 2
        opts.inter_op_num_threads = 1

        if os.path.exists(self.yolo_path):
            try:
                self.yolo_session = ort.InferenceSession(self.yolo_path, opts, providers=['CPUExecutionProvider'])
                print("[SURDAS-AI] YOLOv8n engine loaded (2 threads).", flush=True)
            except Exception as e:
                print(f"[SURDAS-AI] Error loading YOLOv8: {e}", flush=True)

        if os.path.exists(self.midas_path):
            try:
                self.midas_session = ort.InferenceSession(self.midas_path, opts, providers=['CPUExecutionProvider'])
                print("[SURDAS-AI] MiDaS depth engine loaded (2 threads).", flush=True)
            except Exception as e:
                print(f"[SURDAS-AI] Error loading MiDaS: {e}", flush=True)

    def init_camera(self):
        # Logitech C270 is on /dev/video1
        for dev in [1, 0]:
            try:
                cap = cv2.VideoCapture(dev)
                if cap.isOpened():
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                    cap.set(cv2.CAP_PROP_FPS, 10)
                    ret, frame = cap.read()
                    if ret and frame is not None:
                        print(f"[SURDAS-AI] Logitech camera connected on /dev/video{dev}", flush=True)
                        self.camera = cap
                        return True
                    cap.release()
            except Exception:
                pass
        print("[SURDAS-AI] Note: Camera not available, generating status feed.", flush=True)
        return False

    def preprocess_yolo(self, frame):
        # Resize to 640x640 FP16
        img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img = cv2.resize(img, (640, 640))
        img = img.astype(np.float16) / 255.0
        img = np.transpose(img, (2, 0, 1))
        return np.expand_dims(img, axis=0)

    def preprocess_midas(self, frame):
        img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img = cv2.resize(img, (256, 256))
        img = img.astype(np.float32) / 255.0
        mean = np.array([0.485, 0.456, 0.406], dtype=np.float32)
        std = np.array([0.229, 0.224, 0.225], dtype=np.float32)
        img = (img - mean) / std
        img = np.transpose(img, (2, 0, 1))
        return np.expand_dims(img, axis=0)

    def postprocess_yolo(self, output, orig_w, orig_h, conf_thresh=0.35, iou_thresh=0.45):
        predictions = np.squeeze(output[0]).T
        boxes = []
        confidences = []
        class_ids = []

        x_factor = orig_w / 640.0
        y_factor = orig_h / 640.0

        for row in predictions:
            classes_scores = row[4:]
            max_score = np.max(classes_scores)
            if max_score >= conf_thresh:
                class_id = np.argmax(classes_scores)
                cx, cy, w, h = row[0:4]
                left = int((cx - 0.5 * w) * x_factor)
                top = int((cy - 0.5 * h) * y_factor)
                width = int(w * x_factor)
                height = int(h * y_factor)

                boxes.append([left, top, width, height])
                confidences.append(float(max_score))
                class_ids.append(int(class_id))

        indices = cv2.dnn.NMSBoxes(boxes, confidences, conf_thresh, iou_thresh)
        detections = []
        if len(indices) > 0:
            for idx in indices.flatten():
                box = boxes[idx]
                cls_name = COCO_CLASSES[class_ids[idx]] if class_ids[idx] < len(COCO_CLASSES) else "obstacle"
                detections.append({
                    "class": cls_name,
                    "confidence": round(confidences[idx], 2),
                    "box": box,
                    "center_x": box[0] + box[2] // 2,
                    "center_y": box[1] + box[3] // 2,
                    "h_pixel": box[3]
                })
        return detections

    def compute_guidance(self, detections, depth_map, orig_w, orig_h):
        messages = []

        # 1. Depth clearance check (when depth map available)
        if depth_map is not None:
            dh, dw = depth_map.shape
            third = dw // 3
            left_depth = float(np.mean(depth_map[:, :third]))
            center_depth = float(np.mean(depth_map[:, third:2*third]))
            right_depth = float(np.mean(depth_map[:, 2*third:]))

            with self.lock:
                self.spatial_sectors = {
                    "left": round(left_depth / 255.0, 2),
                    "center": round(center_depth / 255.0, 2),
                    "right": round(right_depth / 255.0, 2)
                }

            if center_depth > 175:
                if left_depth < right_depth:
                    messages.append("Obstacle directly ahead. Clear on your left.")
                else:
                    messages.append("Obstacle directly ahead. Clear on your right.")

        # 2. Semantic object positions
        if detections:
            for det in detections[:2]:
                cls = det["class"]
                cx = det["center_x"]
                h_px = det.get("h_pixel", 100)

                if cx < orig_w * 0.33: pos = "on left"
                elif cx > orig_w * 0.66: pos = "on right"
                else: pos = "straight ahead"

                # Geometric distance approximation (relative to human stature 1.7m)
                if h_px > 300: dist_str = "about 1 meter"
                elif h_px > 180: dist_str = "about 2 meters"
                elif h_px > 90: dist_str = "about 3.5 meters"
                else: dist_str = "in distance"

                if cls in ALERT_CLASSES or len(detections) <= 2:
                    messages.append(f"{cls.capitalize()} {pos}, {dist_str}.")

        if not messages:
            return "Path is clear ahead."
        return " ".join(messages[:2])

    def render_overlay(self, frame, detections, depth_map):
        vis = frame.copy()
        h, w = vis.shape[:2]

        for det in detections:
            left, top, bw, bh = det["box"]
            cls = det["class"]
            conf = det["confidence"]
            color = (0, 0, 255) if cls in ["person", "car", "stop sign"] else (0, 255, 0)
            cv2.rectangle(vis, (left, top), (left + bw, top + bh), color, 2)
            label = f"{cls} {int(conf*100)}%"
            cv2.putText(vis, label, (left, max(top - 8, 15)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        # Depth Heatmap thumbnail (140x105)
        if depth_map is not None:
            depth_color = cv2.applyColorMap(depth_map, cv2.COLORMAP_INFERNO)
            depth_thumb = cv2.resize(depth_color, (140, 105))
            vis[10:115, w-150:w-10] = depth_thumb
            cv2.rectangle(vis, (w-150, 10), (w-10, 115), (255, 255, 255), 1)

        # Guidance text bar
        cv2.rectangle(vis, (0, h-35), (w, h), (10, 15, 25), -1)
        cv2.putText(vis, f"SURDAS: {self.latest_guidance[:65]}", (12, h-12), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 229, 255), 1)

        return vis

    def run_worker(self):
        """Paced AI worker loop running at ~2.5 FPS (350ms delay) to keep CPU cool & responsive"""
        self.init_camera()
        frame_idx = 0

        while self.running:
            if self.camera is not None and self.camera.isOpened():
                ret, frame = self.camera.read()
                if not ret or frame is None:
                    time.sleep(0.1)
                    continue
            else:
                frame = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(frame, "SURDAS ASSISTIVE AI", (170, 220), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 229, 255), 2)
                cv2.putText(frame, "Waiting for Camera feed...", (180, 260), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (144, 164, 174), 1)
                time.sleep(0.5)

            h, w = frame.shape[:2]
            detections = []
            depth_norm = None

            # 1. Run YOLOv8 detection
            if self.yolo_session is not None:
                try:
                    inp = self.preprocess_yolo(frame)
                    raw_out = self.yolo_session.run(None, {self.yolo_session.get_inputs()[0].name: inp})
                    detections = self.postprocess_yolo(raw_out, w, h)
                except Exception:
                    pass

            # 2. Run MiDaS Depth every 5th frame (~every 2 seconds) to preserve CPU
            if self.midas_session is not None and (frame_idx % 5 == 0):
                try:
                    inp_midas = self.preprocess_midas(frame)
                    depth_out = self.midas_session.run(None, {self.midas_session.get_inputs()[0].name: inp_midas})
                    raw_depth = np.squeeze(depth_out[0])
                    d_min, d_max = raw_depth.min(), raw_depth.max()
                    if d_max > d_min:
                        depth_norm = ((raw_depth - d_min) / (d_max - d_min) * 255).astype(np.uint8)
                        with self.lock:
                            self.latest_depth_map = depth_norm
                except Exception:
                    pass
            else:
                with self.lock:
                    depth_norm = self.latest_depth_map

            guidance = self.compute_guidance(detections, depth_norm, w, h)
            vis_frame = self.render_overlay(frame, detections, depth_norm)

            with self.lock:
                self.latest_frame = vis_frame
                self.latest_detections = detections
                self.latest_guidance = guidance

            frame_idx += 1
            # Deliberate 300ms pause: gives 2.5-3 FPS, keeping CPU load under 25%
            time.sleep(0.3)

    def start(self):
        self.running = True
        t = threading.Thread(target=self.run_worker, daemon=True)
        t.start()
        print("[SURDAS-AI] Efficient AI perception thread started.", flush=True)

    def stop(self):
        self.running = False
        if self.camera is not None:
            self.camera.release()

    def get_jpeg_stream(self):
        """Yields smooth 6 FPS MJPEG stream with low compression overhead"""
        while self.running:
            with self.lock:
                frame = self.latest_frame

            if frame is not None:
                # 50% JPEG quality is fast to encode on ARM and low bandwidth
                ret, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 50])
                if ret:
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')
            time.sleep(0.15)
