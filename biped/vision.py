"""Person detection, tracking, and command generation for person-following."""

import cv2
import numpy as np
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Optional, Tuple, List

from .comms import RobotCommand, UDPSender


@dataclass
class TrackedObject:
    object_id: int
    centroid: Tuple[int, int]
    bbox: Tuple[int, int, int, int]
    disappeared: int = 0
    history: deque = field(default_factory=lambda: deque(maxlen=30))

    def bbox_area(self) -> int:
        x1, y1, x2, y2 = self.bbox
        return (x2 - x1) * (y2 - y1)

    def bbox_height_frac(self, frame_h: int) -> float:
        _, y1, _, y2 = self.bbox
        return (y2 - y1) / frame_h


class CentroidTracker:
    """Centroid-based multi-object tracker with identity persistence."""

    def __init__(self, max_disappeared: int = 15, max_distance: float = 100):
        self.next_id = 0
        self.objects: dict[int, TrackedObject] = {}
        self.max_disappeared = max_disappeared
        self.max_distance = max_distance

    def _register(self, centroid, bbox) -> int:
        obj = TrackedObject(self.next_id, centroid, bbox)
        obj.history.append(centroid)
        self.objects[self.next_id] = obj
        self.next_id += 1
        return obj.object_id

    def update(self, detections: List[Tuple[Tuple[int, int], Tuple[int, int, int, int]]]) -> dict:
        if len(detections) == 0:
            for oid in list(self.objects.keys()):
                self.objects[oid].disappeared += 1
                if self.objects[oid].disappeared > self.max_disappeared:
                    del self.objects[oid]
            return self.objects

        if len(self.objects) == 0:
            for c, b in detections:
                self._register(c, b)
            return self.objects

        obj_ids = list(self.objects.keys())
        obj_cents = [self.objects[o].centroid for o in obj_ids]
        det_cents = [d[0] for d in detections]
        det_bboxes = [d[1] for d in detections]

        pairs = []
        for i, oc in enumerate(obj_cents):
            for j, dc in enumerate(det_cents):
                d = np.sqrt((oc[0] - dc[0]) ** 2 + (oc[1] - dc[1]) ** 2)
                pairs.append((d, i, j))
        pairs.sort()

        matched_o, matched_d = set(), set()
        for dist, i, j in pairs:
            if i in matched_o or j in matched_d:
                continue
            if dist > self.max_distance:
                break
            oid = obj_ids[i]
            self.objects[oid].centroid = det_cents[j]
            self.objects[oid].bbox = det_bboxes[j]
            self.objects[oid].disappeared = 0
            self.objects[oid].history.append(det_cents[j])
            matched_o.add(i)
            matched_d.add(j)

        for i in range(len(obj_cents)):
            if i not in matched_o:
                oid = obj_ids[i]
                self.objects[oid].disappeared += 1
                if self.objects[oid].disappeared > self.max_disappeared:
                    del self.objects[oid]

        for j in range(len(det_cents)):
            if j not in matched_d:
                self._register(det_cents[j], det_bboxes[j])

        return self.objects

    def reset(self):
        self.objects.clear()
        self.next_id = 0


class PersonDetector:
    """YOLOv8n person detector."""

    def __init__(self, model_name="yolov8n", input_size=320, confidence=0.5):
        from ultralytics import YOLO

        self.model = YOLO(f"{model_name}.pt")
        self.input_size = input_size
        self.confidence = confidence

    def detect(self, frame: np.ndarray) -> list:
        results = self.model(
            frame, imgsz=self.input_size, conf=self.confidence,
            classes=[0], verbose=False,
        )
        detections = []
        for r in results:
            if r.boxes is None:
                continue
            for box in r.boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)
                detections.append(((cx, cy), (x1, y1, x2, y2)))
        return detections

    def export_onnx(self):
        self.model.export(format="onnx", imgsz=self.input_size)
        print("ONNX model exported.")


class CommandGenerator:
    """Convert tracked person position to robot commands."""

    def __init__(self, params: dict):
        self.p = params
        self.smoothed_yaw = 0.0
        self.smoothed_fwd = 0.0
        self.target_id: Optional[int] = None

    def select_target(self, objects: dict, frame_h: int) -> Optional[TrackedObject]:
        if not objects:
            return None
        if self.target_id is not None and self.target_id in objects:
            obj = objects[self.target_id]
            if obj.disappeared == 0:
                return obj
        best = max(objects.values(), key=lambda o: o.bbox_area())
        self.target_id = best.object_id
        return best

    def generate(self, target: Optional[TrackedObject], fw: int, fh: int) -> RobotCommand:
        cmd = RobotCommand()
        if target is None or target.disappeared > 0:
            self.smoothed_yaw *= 0.9
            self.smoothed_fwd *= 0.9
            cmd.yaw = self.smoothed_yaw
            cmd.forward = self.smoothed_fwd
            self.target_id = None
            return cmd

        cmd.tracking = True
        cmd.target_id = target.object_id
        cx, cy = target.centroid
        nx = (cx - fw / 2) / (fw / 2)
        ny = (cy - fh / 2) / (fh / 2)
        bh = target.bbox_height_frac(fh)

        cmd.head_yaw = -nx
        cmd.head_pitch = -ny * 0.5

        raw_yaw = -self.p["kp_yaw"] * nx if abs(nx) > self.p["deadband_x"] else 0.0
        size_err = self.p["target_bbox_height"] - bh
        raw_fwd = self.p["kp_forward"] * size_err if abs(size_err) > self.p["deadband_size"] else 0.0

        raw_yaw = np.clip(raw_yaw, -self.p["max_yaw_cmd"], self.p["max_yaw_cmd"])
        raw_fwd = np.clip(raw_fwd, -self.p["max_forward_cmd"], self.p["max_forward_cmd"])

        a = self.p["smoothing_alpha"]
        self.smoothed_yaw = a * raw_yaw + (1 - a) * self.smoothed_yaw
        self.smoothed_fwd = a * raw_fwd + (1 - a) * self.smoothed_fwd
        cmd.yaw = self.smoothed_yaw
        cmd.forward = self.smoothed_fwd
        return cmd


def draw_overlay(frame, objects, cmd, fps, params):
    """Draw tracking visualization on frame."""
    h, w = frame.shape[:2]
    out = frame.copy()

    cv2.line(out, (w // 2 - 20, h // 2), (w // 2 + 20, h // 2), (100, 100, 100), 1)
    cv2.line(out, (w // 2, h // 2 - 20), (w // 2, h // 2 + 20), (100, 100, 100), 1)

    db = int(params["deadband_x"] * w / 2)
    cv2.rectangle(out, (w // 2 - db, 0), (w // 2 + db, h), (50, 50, 50), 1)

    for oid, obj in objects.items():
        x1, y1, x2, y2 = obj.bbox
        active = cmd.tracking and oid == cmd.target_id
        color = (0, 255, 0) if active else (0, 200, 255)
        thick = 2 if active else 1
        label = f"TARGET #{oid}" if active else f"#{oid}"

        if obj.disappeared == 0:
            cv2.rectangle(out, (x1, y1), (x2, y2), color, thick)
            cv2.circle(out, obj.centroid, 4, color, -1)
            cv2.putText(out, label, (x1, y1 - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
            for i in range(1, len(obj.history)):
                cv2.line(out, obj.history[i - 1], obj.history[i], color, 1)

    y = 20
    cv2.putText(out, f"FPS: {fps:.1f}", (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    y += 25
    st = "TRACKING" if cmd.tracking else "SEARCHING"
    sc = (0, 255, 0) if cmd.tracking else (0, 0, 255)
    cv2.putText(out, st, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, sc, 2)
    y += 30
    for lbl, val in [("Yaw", cmd.yaw), ("Fwd", cmd.forward), ("H.Yaw", cmd.head_yaw), ("H.Pitch", cmd.head_pitch)]:
        cv2.putText(out, f"{lbl}: {val:+.2f}", (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)
        y += 20

    return out


def run_vision(source=0, params=None, send_udp=False, udp_ip="127.0.0.1", udp_port=8888, export_onnx=False):
    """Run the full vision pipeline."""
    if params is None:
        from .config import load_config
        params = load_config()["vision"]

    detector = PersonDetector(
        params["model_size"], params["input_size"], params["confidence_threshold"],
    )

    if export_onnx:
        detector.export_onnx()
        return

    tracker = CentroidTracker(params["max_disappeared"], params["max_distance"])
    cmd_gen = CommandGenerator(params)
    sender = UDPSender(udp_ip, udp_port) if send_udp else None

    src = int(source) if isinstance(source, str) and source.isdigit() else source
    cap = cv2.VideoCapture(src)
    if not cap.isOpened():
        print(f"Error: cannot open source: {source}")
        return

    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"Source opened: {w}x{h}")
    print("Press 'q' to quit, 'r' to reset tracker\n")

    fps_buf = deque(maxlen=30)
    n = 0

    try:
        while True:
            t0 = time.time()
            ret, frame = cap.read()
            if not ret:
                if isinstance(source, str) and not source.startswith("http"):
                    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    continue
                break

            dets = detector.detect(frame)
            objects = tracker.update(dets)
            fh, fw = frame.shape[:2]
            target = cmd_gen.select_target(objects, fh)
            cmd = cmd_gen.generate(target, fw, fh)

            if sender:
                sender.send(cmd)

            dt = time.time() - t0
            fps_buf.append(dt)
            fps = len(fps_buf) / sum(fps_buf)

            n += 1
            if n % 10 == 0:
                st = "TRACKING" if cmd.tracking else "SEARCHING"
                print(
                    f"[{st}] yaw={cmd.yaw:+.2f} fwd={cmd.forward:+.2f} "
                    f"head=({cmd.head_yaw:+.2f},{cmd.head_pitch:+.2f}) "
                    f"fps={fps:.1f}"
                )

            overlay = draw_overlay(frame, objects, cmd, fps, params)
            cv2.imshow("Robot Vision", overlay)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            elif key == ord("r"):
                tracker.reset()
                cmd_gen.target_id = None
                print("Tracker reset.")
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        cv2.destroyAllWindows()
        if sender:
            sender.close()
