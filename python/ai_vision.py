#!/usr/bin/env python3
"""
SURDAS Assistive AI Perception Engine
Performs real-time YOLOv8 object detection and MiDaS depth estimation
on Qualcomm QRB2210 (ARM Cortex-A53) using ONNX Runtime.
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

# High priority classes for pedestrian mobility & collision avoidance
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
        self.latest_guidance = "Initializing SURDAS AI vision..."
        self.spatial_sectors = {"left": 0.0, "center": 0.0, "right": 0.0}

        self.lock = threading.Lock()
        self.load_models()

    def load_models(self):
        opts = ort.SessionOptions()
        opts.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        opts.intra_op_num_threads = 4

        if os.path.exists(self.yolo_path):
            try:
                self.yolo_session = ort.InferenceSession(self.yolo_path, opts, providers=['CPUExecutionProvider'])
                print("[SURDAS-AI] YOLOv8n ONNX engine loaded successfully.")
            except Exception as e:
                print(f"[SURDAS-AI] Error loading YOLOv8: {e}")

        if os.path.exists(self.midas_path):
            try:
                self.midas_session = ort.InferenceSession(self.midas_path, opts, providers=['CPUExecutionProvider'])
                print("[SURDAS-AI] MiDaS Small depth engine loaded successfully.")
            except Exception as e:
                print(f"[SURDAS-AI] Error loading MiDaS: {e}")

    def init_camera(self):
        # Logitech C270 is typically on /dev/video1 or /dev/video0
        for dev in [1, 0, 2]:
            cap = cv2.VideoCapture(dev)
            if cap.isOpened():
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                cap.set(cv2.CAP_PROP_FPS, 15)
                ret, frame = cap.read()
                if ret and frame is not None:
                    print(f"[SURDAS-AI] Logitech Camera connected on /dev/video{dev}")
                    self.camera = cap
                    return True
                cap.release()
        print("[SURDAS-AI] Warning: No physical camera found, operating in simulated video mode.")
        return False

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
        # MiDaS ImageNet normalization
        mean = np.array([0.485, 0.456, 0.406], dtype=np.float32)
        std = np.array([0.229, 0.224, 0.225], dtype=np.float32)
        img = (img - mean) / std
        img = np.transpose(img, (2, 0, 1))
        return np.expand_dims(img, axis=0)

    def postprocess_yolo(self, output, orig_w, orig_h, conf_thresh=0.35, iou_thresh=0.45):
        # Output shape: (1, 84, 8400)
        predictions = np.squeeze(output[0]).T  # (8400, 84)

        boxes = []
        confidences = []
        class_ids = []

        # Coordinates are cx, cy, w, h
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
                cls_name = COCO_CLASSES[class_ids[idx]] if class_ids[idx] < len(COCO_CLASSES) else "object"
                detections.append({
                    "class": cls_name,
                    "confidence": round(confidences[idx], 2),
                    "box": box, # [left, top, width, height]
                    "center_x": box[0] + box[2] // 2,
                    "center_y": box[1] + box[3] // 2
                })
        return detections

    def compute_guidance(self, detections, depth_map, orig_w, orig_h):
        """Translates detected objects + depth map into clear spatial verbal guidance for blind users"""
        messages = []

        # 1. Analyze depth map sectors (left, center, right)
        if depth_map is not None:
            # depth_map has values 0..255 where higher = CLOSER
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

            # Proximity collision warning (> 170 on 0..255 scale means < 1.0 meter)
            if center_depth > 170:
                if left_depth < right_depth:
                    messages.append("Obstacle directly ahead! Clear path on your left.")
                else:
                    messages.append("Obstacle directly ahead! Clear path on your right.")

        # 2. Add object semantic positions
        if detections:
            for det in detections[:3]: # top 3 most prominent detections
                cls = det["class"]
                cx = det["center_x"]

                # Sector direction
                if cx < orig_w * 0.33:
                    pos = "on your left"
                elif cx > orig_w * 0.66:
                    pos = "on your right"
                else:
                    pos = "straight ahead"

                # Estimate distance from depth map at bounding box center
                dist_str = ""
                if depth_map is not None:
                    norm_y = int(np.clip(det["center_y"] * (depth_map.shape[0] / orig_h), 0, depth_map.shape[0]-1))
                    norm_x = int(np.clip(cx * (depth_map.shape[1] / orig_w), 0, depth_map.shape[1]-1))
                    proximity = depth_map[norm_y, norm_x]
                    # Map inverse depth 0..255 to approximate meters (0.5m to 4m)
                    if proximity > 180:
                        dist_str = "very close, about 0.8 meters"
                    elif proximity > 130:
                        dist_str = "about 1.5 meters"
                    elif proximity > 80:
                        dist_str = "about 2.5 meters"
                    else:
                        dist_str = "further ahead"

                if cls in ALERT_CLASSES or len(detections) <= 2:
                    if dist_str:
                        messages.append(f"{cls.capitalize()} {pos}, {dist_str}.")
                    else:
                        messages.append(f"{cls.capitalize()} {pos}.")

        if not messages:
            return "Path is clear ahead."
        return " ".join(messages[:2])

    def render_overlay(self, frame, detections, depth_map):
        vis = frame.copy()
        h, w = vis.shape[:2]

        # Draw detected bounding boxes
        for det in detections:
            left, top, bw, bh = det["box"]
            cls = det["class"]
            conf = det["confidence"]

            # Visual color scheme
            color = (0, 0, 255) if cls in ["person", "car", "stop sign"] else (0, 255, 0)
            cv2.rectangle(vis, (left, top), (left + bw, top + bh), color, 2)

            label = f"{cls} {int(conf*100)}%"
            cv2.putText(vis, label, (left, max(top - 8, 15)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            cv2.putText(vis, label, (left, max(top - 8, 15)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

        # Inset Depth heatmap in top right corner (160x120)
        if depth_map is not None:
            depth_color = cv2.applyColorMap(depth_map, cv2.COLORMAP_INFERNO)
            depth_thumb = cv2.resize(depth_color, (160, 120))
            vis[10:130, w-170:w-10] = depth_thumb
            cv2.rectangle(vis, (w-170, 10), (w-10, 130), (255, 255, 255), 1)
            cv2.putText(vis, "DEPTH MAP", (w-165, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

        # Navigation Guidance Banner on bottom
        guidance = self.latest_guidance
        cv2.rectangle(vis, (0, h-40), (w, h), (15, 23, 42), -1)
        cv2.putText(vis, f"SURDAS: {guidance[:70]}", (15, h-15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (56, 189, 248), 1)

        return vis

    def run_worker(self):
        """Continuous perception loop running at 10-15 FPS"""
        self.init_camera()
        frame_idx = 0

        while self.running:
            if self.camera is not None and self.camera.isOpened():
                ret, frame = self.camera.read()
                if not ret or frame is None:
                    time.sleep(0.05)
                    continue
            else:
                # Generate synthetic test frame if no physical camera
                frame = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(frame, "SURDAS CAMERA SEARCHING...", (120, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                time.sleep(0.1)

            h, w = frame.shape[:2]
            detections = []
            depth_norm = None

            # 1. Run YOLOv8 every frame
            if self.yolo_session is not None:
                try:
                    inp = self.preprocess_yolo(frame)
                    raw_out = self.yolo_session.run(None, {self.yolo_session.get_inputs()[0].name: inp})
                    detections = self.postprocess_yolo(raw_out, w, h)
                except Exception as e:
                    pass

            # 2. Run MiDaS Depth every 2nd frame (to optimize CPU utilization)
            if self.midas_session is not None and (frame_idx % 2 == 0):
                try:
                    inp_midas = self.preprocess_midas(frame)
                    depth_out = self.midas_session.run(None, {self.midas_session.get_inputs()[0].name: inp_midas})
                    raw_depth = np.squeeze(depth_out[0])
                    # Normalize depth to 0..255 uint8
                    d_min, d_max = raw_depth.min(), raw_depth.max()
                    if d_max > d_min:
                        depth_norm = ((raw_depth - d_min) / (d_max - d_min) * 255).astype(np.uint8)
                        with self.lock:
                            self.latest_depth_map = depth_norm
                except Exception as e:
                    pass
            else:
                with self.lock:
                    depth_norm = self.latest_depth_map

            # 3. Generate verbal spatial guidance
            guidance = self.compute_guidance(detections, depth_norm, w, h)

            # 4. Render visual overlay
            vis_frame = self.render_overlay(frame, detections, depth_norm)

            with self.lock:
                self.latest_frame = vis_frame
                self.latest_detections = detections
                self.latest_guidance = guidance

            frame_idx += 1
            time.sleep(0.03)

    def start(self):
        self.running = True
        t = threading.Thread(target=self.run_worker, daemon=True)
        t.start()
        print("[SURDAS-AI] Perception thread started successfully.")

    def stop(self):
        self.running = False
        if self.camera is not None:
            self.camera.release()

    def get_jpeg_stream(self):
        """Yields MJPEG stream for web dashboard"""
        while self.running:
            with self.lock:
                frame = self.latest_frame

            if frame is not None:
                ret, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
                if ret:
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')
            time.sleep(0.06)
