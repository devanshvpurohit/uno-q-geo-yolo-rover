#!/usr/bin/env python3
"""
SURDAS Assistive AI Perception Engine - High Performance Edge Architecture
Optimized for Qualcomm QRB2210 (ARM Cortex-A53 quad-core, Arduino UNO Q)
- Decoupled Camera Pipeline: Dedicated UVC thread for 20-30 FPS low-latency capture
- Background AI Perception Thread: Paced at 3-4 FPS with 2 ONNX threads
- Zero-Lag MJPEG Stream: Live camera feed with smooth bounding box overlay (20+ FPS)
- MiDaS Depth Clearance: Periodic depth mapping for 3-zone spatial awareness
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

        # Live Frame Buffer (Decoupled)
        self.raw_frame = None
        self.raw_frame_lock = threading.Lock()

        # Cached AI Detections & Spatial Awareness
        self.cached_detections = []
        self.cached_depth_map = None
        self.latest_guidance = "SURDAS vision active. Path is clear."
        self.spatial_sectors = {"left": 0.0, "center": 0.0, "right": 0.0}
        self.ai_lock = threading.Lock()

        self.fps_cam = 0.0
        self.fps_ai = 0.0

        self.load_models()

    @property
    def latest_detections(self):
        with self.ai_lock:
            return list(self.cached_detections)

    def load_models(self):
        opts = ort.SessionOptions()
        opts.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        # 2 threads max to preserve 2 cores for OS, telemetry and video streaming
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
        # Logitech C270 is on /dev/video1 or /dev/video0
        for dev in [1, 0]:
            try:
                cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
                if cap.isOpened():
                    # Request hardware MJPEG from Logitech C270 for low USB latency
                    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                    cap.set(cv2.CAP_PROP_FPS, 30)
                    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

                    ret, frame = cap.read()
                    if ret and frame is not None:
                        actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                        actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                        print(f"[SURDAS-AI] Logitech Camera connected on /dev/video{dev} ({actual_w}x{actual_h} MJPG)", flush=True)
                        self.camera = cap
                        return True
                    cap.release()
            except Exception as e:
                print(f"[SURDAS-AI] Device /dev/video{dev} probe failed: {e}", flush=True)

        print("[SURDAS-AI] Note: Camera not available, generating simulated feed.", flush=True)
        return False

    def _camera_worker(self):
        """Dedicated high-speed frame grabber thread ensuring zero V4L2 queue latency"""
        frame_count = 0
        last_t = time.time()

        while self.running:
            if self.camera is not None and self.camera.isOpened():
                ret, frame = self.camera.read()
                if ret and frame is not None:
                    with self.raw_frame_lock:
                        self.raw_frame = frame
                    frame_count += 1
                else:
                    time.sleep(0.005)
            else:
                time.sleep(0.05)

            now = time.time()
            if now - last_t >= 2.0:
                self.fps_cam = round(frame_count / (now - last_t), 1)
                frame_count = 0
                last_t = now

    def preprocess_yolo(self, frame):
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

    def postprocess_yolo(self, raw_output, orig_w, orig_h, conf_thresh=0.35, iou_thresh=0.45):
        output = np.squeeze(raw_output[0])
        if output.shape[0] < output.shape[1]:
            output = output.T

        boxes = []
        confidences = []
        class_ids = []

        x_scale = orig_w / 640.0
        y_scale = orig_h / 640.0

        for row in output:
            scores = row[4:]
            max_idx = np.argmax(scores)
            max_conf = float(scores[max_idx])

            if max_conf >= conf_thresh:
                cx, cy, w, h = row[0:4]
                left = int((cx - w / 2.0) * x_scale)
                top = int((cy - h / 2.0) * y_scale)
                width = int(w * x_scale)
                height = int(h * y_scale)

                boxes.append([left, top, width, height])
                confidences.append(max_conf)
                class_ids.append(max_idx)

        indices = cv2.dnn.NMSBoxes(boxes, confidences, conf_thresh, iou_thresh)
        detections = []
        if len(indices) > 0:
            for i in indices.flatten():
                detections.append({
                    "class": COCO_CLASSES[class_ids[i]] if class_ids[i] < len(COCO_CLASSES) else "object",
                    "confidence": round(confidences[i], 2),
                    "box": boxes[i]
                })
        return detections

    def compute_guidance(self, detections, depth_map, w, h):
        messages = []
        sectors = {"left": 0.0, "center": 0.0, "right": 0.0}

        if depth_map is not None:
            dh, dw = depth_map.shape[:2]
            third = dw // 3
            left_zone = depth_map[:, :third]
            center_zone = depth_map[:, third:2*third]
            right_zone = depth_map[:, 2*third:]

            sectors["left"] = round(float(np.mean(left_zone)) / 255.0, 2)
            sectors["center"] = round(float(np.mean(center_zone)) / 255.0, 2)
            sectors["right"] = round(float(np.mean(right_zone)) / 255.0, 2)

            with self.ai_lock:
                self.spatial_sectors = sectors

            if sectors["center"] > 0.65:
                clearer = "left" if sectors["left"] < sectors["right"] else "right"
                messages.append(f"Obstacle close ahead! Veer {clearer}.")

        for det in detections:
            cls = det["class"]
            left, top, bw, bh = det["box"]
            cx = left + bw / 2.0

            if cls in ALERT_CLASSES or det["confidence"] >= 0.5:
                pos = "Center" if (0.33 * w <= cx <= 0.67 * w) else ("Left" if cx < 0.33 * w else "Right")
                proximity = "very close" if bh > (0.45 * h) else "ahead"
                messages.append(f"{cls.capitalize()} {pos} {proximity}.")

        if not messages:
            return "Path is clear ahead."
        return " ".join(messages[:2])

    def _ai_worker(self):
        """Paced AI perception worker (3-4 FPS) so Cortex-A53 cores stay cool and smooth"""
        frame_idx = 0
        ai_count = 0
        last_t = time.time()

        while self.running:
            with self.raw_frame_lock:
                frame = self.raw_frame.copy() if self.raw_frame is not None else None

            if frame is None:
                time.sleep(0.05)
                continue

            h, w = frame.shape[:2]
            detections = []
            depth_norm = None

            # 1. YOLOv8 object detection
            if self.yolo_session is not None:
                try:
                    inp = self.preprocess_yolo(frame)
                    raw_out = self.yolo_session.run(None, {self.yolo_session.get_inputs()[0].name: inp})
                    detections = self.postprocess_yolo(raw_out, w, h)
                except Exception:
                    pass

            # 2. MiDaS depth estimation every 6th cycle (~every 1.5 - 2s)
            if self.midas_session is not None and (frame_idx % 6 == 0):
                try:
                    inp_midas = self.preprocess_midas(frame)
                    depth_out = self.midas_session.run(None, {self.midas_session.get_inputs()[0].name: inp_midas})
                    raw_depth = np.squeeze(depth_out[0])
                    d_min, d_max = raw_depth.min(), raw_depth.max()
                    if d_max > d_min:
                        depth_norm = ((raw_depth - d_min) / (d_max - d_min) * 255).astype(np.uint8)
                        with self.ai_lock:
                            self.cached_depth_map = depth_norm
                except Exception:
                    pass
            else:
                with self.ai_lock:
                    depth_norm = self.cached_depth_map

            guidance = self.compute_guidance(detections, depth_norm, w, h)

            with self.ai_lock:
                self.cached_detections = detections
                self.latest_guidance = guidance

            frame_idx += 1
            ai_count += 1
            now = time.time()
            if now - last_t >= 2.0:
                self.fps_ai = round(ai_count / (now - last_t), 1)
                ai_count = 0
                last_t = now

            # Pacing pause: ~180ms ensures ~4 FPS inference without CPU saturation
            time.sleep(0.18)

    def start(self):
        self.running = True
        self.init_camera()

        # Dedicated camera thread (20-30 FPS)
        t_cam = threading.Thread(target=self._camera_worker, daemon=True)
        t_cam.start()

        # Dedicated AI thread (3-4 FPS)
        t_ai = threading.Thread(target=self._ai_worker, daemon=True)
        t_ai.start()
        print("[SURDAS-AI] High-FPS camera and decoupled AI worker started.", flush=True)

    def stop(self):
        self.running = False
        if self.camera is not None:
            self.camera.release()

    def get_jpeg_stream(self):
        """Yields buttery smooth 20+ FPS MJPEG stream composited with latest AI bounding boxes"""
        dummy_frame = None

        while self.running:
            with self.raw_frame_lock:
                raw = self.raw_frame.copy() if self.raw_frame is not None else None

            with self.ai_lock:
                detections = list(self.cached_detections)
                depth_map = self.cached_depth_map
                guidance = self.latest_guidance

            if raw is not None:
                vis = raw
            else:
                if dummy_frame is None:
                    dummy_frame = np.zeros((480, 640, 3), dtype=np.uint8)
                    cv2.putText(dummy_frame, "SURDAS ASSISTIVE AI", (170, 220), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 229, 255), 2)
                    cv2.putText(dummy_frame, "Initializing Camera Feed...", (180, 260), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (144, 164, 174), 1)
                vis = dummy_frame.copy()

            h, w = vis.shape[:2]

            # Fast bounding box overlay (takes <0.5ms)
            for det in detections:
                left, top, bw, bh = det["box"]
                cls = det["class"]
                conf = det["confidence"]
                color = (0, 0, 255) if cls in ["person", "car", "stop sign"] else (0, 255, 0)
                cv2.rectangle(vis, (left, top), (left + bw, top + bh), color, 2)
                label = f"{cls} {int(conf*100)}%"
                cv2.putText(vis, label, (left, max(top - 8, 15)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            # Depth Heatmap thumbnail (130x95)
            if depth_map is not None:
                try:
                    depth_color = cv2.applyColorMap(depth_map, cv2.COLORMAP_INFERNO)
                    depth_thumb = cv2.resize(depth_color, (130, 95))
                    vis[10:105, w-140:w-10] = depth_thumb
                    cv2.rectangle(vis, (w-140, 10), (w-10, 105), (255, 255, 255), 1)
                except Exception:
                    pass

            # Guidance banner & Live FPS stats
            cv2.rectangle(vis, (0, h-32), (w, h), (10, 15, 25), -1)
            cv2.putText(vis, f"SURDAS: {guidance[:55]}", (10, h-10), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 229, 255), 1)
            cv2.putText(vis, f"CAM: {self.fps_cam}fps | AI: {self.fps_ai}fps", (w-190, h-10), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (178, 235, 242), 1)

            ret, jpeg = cv2.imencode('.jpg', vis, [cv2.IMWRITE_JPEG_QUALITY, 55])
            if ret:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')

            # 45ms sleep = ~22 FPS smooth live video!
            time.sleep(0.045)
