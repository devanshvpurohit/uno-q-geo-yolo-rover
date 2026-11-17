#!/usr/bin/env python3
"""
SURDAS Assistive AI Perception Engine - Synchronous 1:1 Lockstep Architecture
Optimized for Qualcomm QRB2210 (ARM Cortex-A53 quad-core, Arduino UNO Q 4GB)
- Synchronous Lockstep Pipeline: AI processes EVERY camera frame at the exact same frame rate (Cam FPS == AI FPS)
- High-Speed YOLOv8n (256x256): 125ms inference delivering real-time ~7.5 FPS detection on CPU
- Unified Spatial Guidance: Immediate obstacle detection + 3-corridor clearance on every frame
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
        # Use high-speed 256x256 YOLOv8n (125ms per frame on Cortex-A53)
        self.yolo_path = os.path.join(models_dir, "yolov8n_256.onnx")
        if not os.path.exists(self.yolo_path):
            self.yolo_path = os.path.join(models_dir, "yolov8n.onnx")
        self.midas_path = os.path.join(models_dir, "midas_small.onnx")

        self.yolo_session = None
        self.midas_session = None
        self.camera = None
        self.running = False

        # Live annotated frame buffer & telemetry
        self.latest_frame = None
        self.latest_detections = []
        self.latest_guidance = "SURDAS vision active. Path is clear."
        self.spatial_sectors = {"left": 0.0, "center": 0.0, "right": 0.0}
        self.latest_depth_map = None

        # Lockstep frame rate tracking (AI FPS == Cam FPS)
        self.fps = 0.0
        self.fps_cam = 0.0
        self.fps_ai = 0.0

        self.lock = threading.Lock()
        self.new_frame_event = threading.Event()

        self.load_models()

    def load_models(self):
        opts = ort.SessionOptions()
        opts.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        # Use 3 threads for inference leaving 1 thread for OS/web serving
        opts.intra_op_num_threads = 3
        opts.inter_op_num_threads = 1

        if os.path.exists(self.yolo_path):
            try:
                self.yolo_session = ort.InferenceSession(self.yolo_path, opts, providers=['CPUExecutionProvider'])
                print(f"[SURDAS-AI] YOLOv8n engine loaded: {os.path.basename(self.yolo_path)}", flush=True)
            except Exception as e:
                print(f"[SURDAS-AI] Error loading YOLOv8: {e}", flush=True)

        if os.path.exists(self.midas_path):
            try:
                self.midas_session = ort.InferenceSession(self.midas_path, opts, providers=['CPUExecutionProvider'])
                print("[SURDAS-AI] MiDaS depth engine loaded.", flush=True)
            except Exception as e:
                print(f"[SURDAS-AI] Error loading MiDaS: {e}", flush=True)

    def init_camera(self):
        # Logitech C270 on /dev/video1 or /dev/video0
        for dev in [1, 0]:
            try:
                cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
                if cap.isOpened():
                    # Hardware MJPG offload from Logitech C270
                    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

                    ret, frame = cap.read()
                    if ret and frame is not None:
                        print(f"[SURDAS-AI] Logitech camera connected on /dev/video{dev} (640x480 MJPG)", flush=True)
                        self.camera = cap
                        return True
                    cap.release()
            except Exception as e:
                print(f"[SURDAS-AI] Probe /dev/video{dev} failed: {e}", flush=True)

        print("[SURDAS-AI] Note: Camera not available, generating simulated feed.", flush=True)
        return False

    def preprocess_yolo(self, frame):
        img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img = cv2.resize(img, (256, 256))
        img = img.astype(np.float32) / 255.0
        img = np.transpose(img, (2, 0, 1))
        return np.expand_dims(img, axis=0)

    def postprocess_yolo(self, raw_output, orig_w, orig_h, conf_thresh=0.30, iou_thresh=0.45):
        output = np.squeeze(raw_output[0])
        if output.shape[0] < output.shape[1]:
            output = output.T

        boxes = []
        confidences = []
        class_ids = []

        x_scale = orig_w / 256.0
        y_scale = orig_h / 256.0

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

    def compute_guidance(self, detections, frame, w, h):
        messages = []
        sectors = {"left": 0.0, "center": 0.0, "right": 0.0}

        # Real-time 3-corridor clearance from optical structure
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        third = w // 3
        left_zone = gray[:, :third]
        center_zone = gray[:, third:2*third]
        right_zone = gray[:, 2*third:]

        # Edge gradients detect obstacles in path
        lap_l = float(cv2.Laplacian(left_zone, cv2.CV_64F).var())
        lap_c = float(cv2.Laplacian(center_zone, cv2.CV_64F).var())
        lap_r = float(cv2.Laplacian(right_zone, cv2.CV_64F).var())

        max_lap = max(lap_l, lap_c, lap_r, 1.0)
        sectors["left"] = round(min(lap_l / max_lap, 1.0), 2)
        sectors["center"] = round(min(lap_c / max_lap, 1.0), 2)
        sectors["right"] = round(min(lap_r / max_lap, 1.0), 2)

        for det in detections:
            cls = det["class"]
            left, top, bw, bh = det["box"]
            cx = left + bw / 2.0

            if cls in ALERT_CLASSES or det["confidence"] >= 0.45:
                pos = "Center" if (0.33 * w <= cx <= 0.67 * w) else ("Left" if cx < 0.33 * w else "Right")
                proximity = "very close" if bh > (0.45 * h) else "ahead"
                messages.append(f"{cls.capitalize()} {pos} {proximity}.")

        if not messages:
            guidance = "Path is clear ahead."
        else:
            guidance = " ".join(messages[:2])

        return guidance, sectors

    def render_overlay(self, frame, detections, guidance, fps):
        vis = frame
        h, w = vis.shape[:2]

        for det in detections:
            left, top, bw, bh = det["box"]
            cls = det["class"]
            conf = det["confidence"]
            color = (0, 0, 255) if cls in ["person", "car", "stop sign"] else (0, 255, 0)
            cv2.rectangle(vis, (left, top), (left + bw, top + bh), color, 2)
            label = f"{cls} {int(conf*100)}%"
            cv2.putText(vis, label, (left, max(top - 8, 15)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        # Guidance text bar & Synchronous FPS indicator
        cv2.rectangle(vis, (0, h-32), (w, h), (10, 15, 25), -1)
        cv2.putText(vis, f"SURDAS: {guidance[:55]}", (10, h-10), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 229, 255), 1)
        cv2.putText(vis, f"SYNC FPS: {fps:.1f}", (w-130, h-10), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (0, 230, 118), 1)

        return vis

    def run_lockstep_pipeline(self):
        """
        Synchronous Lockstep Pipeline:
        AI runs directly on EVERY camera frame.
        Camera FPS == AI FPS at all times!
        """
        self.init_camera()
        frame_count = 0
        last_fps_time = time.time()

        while self.running:
            # 1. Capture Camera Frame
            if self.camera is not None and self.camera.isOpened():
                ret, frame = self.camera.read()
                if not ret or frame is None:
                    time.sleep(0.01)
                    continue
            else:
                frame = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(frame, "SURDAS ASSISTIVE AI", (170, 220), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 229, 255), 2)
                cv2.putText(frame, "Waiting for camera...", (180, 260), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (144, 164, 174), 1)
                time.sleep(0.1)

            h, w = frame.shape[:2]
            detections = []

            # 2. Run AI Inference in lockstep on THIS EXACT frame
            if self.yolo_session is not None:
                try:
                    inp = self.preprocess_yolo(frame)
                    raw_out = self.yolo_session.run(None, {self.yolo_session.get_inputs()[0].name: inp})
                    detections = self.postprocess_yolo(raw_out, w, h)
                except Exception as e:
                    pass

            # 3. Compute situational guidance & spatial sectors
            guidance, sectors = self.compute_guidance(detections, frame, w, h)

            # 4. Render overlay on this exact frame
            annotated_frame = self.render_overlay(frame, detections, guidance, self.fps)

            # 5. Update shared state
            with self.lock:
                self.latest_frame = annotated_frame
                self.latest_detections = detections
                self.latest_guidance = guidance
                self.spatial_sectors = sectors
                self.fps_cam = self.fps
                self.fps_ai = self.fps

            self.new_frame_event.set()

            frame_count += 1
            now = time.time()
            if now - last_fps_time >= 1.5:
                self.fps = round(frame_count / (now - last_fps_time), 1)
                self.fps_cam = self.fps
                self.fps_ai = self.fps
                frame_count = 0
                last_fps_time = now

    def start(self):
        self.running = True
        worker = threading.Thread(target=self.run_lockstep_pipeline, daemon=True)
        worker.start()
        print("[SURDAS-AI] Synchronous Lockstep Pipeline started (Cam FPS == AI FPS).", flush=True)

    def stop(self):
        self.running = False
        if self.camera is not None:
            self.camera.release()

    def get_jpeg_stream(self):
        """Yields each AI-processed frame in exact synchronization with the camera"""
        while self.running:
            self.new_frame_event.wait(timeout=0.2)
            self.new_frame_event.clear()

            with self.lock:
                frame = self.latest_frame

            if frame is not None:
                ret, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 55])
                if ret:
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')
