from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import onnxruntime as ort

from lib.runtime_types import Detection2D


def _clamp_box(x1: float, y1: float, x2: float, y2: float, w: int, h: int) -> tuple[float, float, float, float] | None:
    xx1 = max(0.0, min(float(w - 1), float(x1)))
    yy1 = max(0.0, min(float(h - 1), float(y1)))
    xx2 = max(0.0, min(float(w - 1), float(x2)))
    yy2 = max(0.0, min(float(h - 1), float(y2)))
    if xx2 <= xx1 or yy2 <= yy1:
        return None
    return xx1, yy1, xx2, yy2


def _iou(a: Detection2D, b: Detection2D) -> float:
    ix1 = max(a.x1, b.x1)
    iy1 = max(a.y1, b.y1)
    ix2 = min(a.x2, b.x2)
    iy2 = min(a.y2, b.y2)
    iw = max(0.0, ix2 - ix1)
    ih = max(0.0, iy2 - iy1)
    inter = iw * ih
    if inter <= 0.0:
        return 0.0
    area_a = max(0.0, a.x2 - a.x1) * max(0.0, a.y2 - a.y1)
    area_b = max(0.0, b.x2 - b.x1) * max(0.0, b.y2 - b.y1)
    denom = area_a + area_b - inter
    if denom <= 0.0:
        return 0.0
    return inter / denom


class VisionPipeline:
    def run(self, frame: np.ndarray, now_s: float) -> list[Detection2D]:
        raise NotImplementedError


@dataclass(frozen=True)
class _Range:
    class_id: int
    h_min: int
    h_max: int
    s_min: int
    s_max: int
    v_min: int
    v_max: int
    min_area: float
    confidence: float


class ColorPipeline(VisionPipeline):
    def __init__(self, spec: dict[str, Any]):
        self._ranges: list[_Range] = []
        raw = spec.get("ranges", [])
        if not isinstance(raw, list):
            raise ValueError("color pipeline ranges must be a list")
        for item in raw:
            if not isinstance(item, dict):
                continue
            self._ranges.append(
                _Range(
                    class_id=int(item.get("class_id", 0)),
                    h_min=int(item.get("h_min", 0)),
                    h_max=int(item.get("h_max", 179)),
                    s_min=int(item.get("s_min", 0)),
                    s_max=int(item.get("s_max", 255)),
                    v_min=int(item.get("v_min", 0)),
                    v_max=int(item.get("v_max", 255)),
                    min_area=float(item.get("min_area", 120.0)),
                    confidence=float(item.get("confidence", 1.0)),
                )
            )
        self._open_ks = int(spec.get("open_kernel", 3))
        self._close_ks = int(spec.get("close_kernel", 3))
        self._name = str(spec.get("type", "color"))

    def run(self, frame: np.ndarray, now_s: float) -> list[Detection2D]:
        if frame is None or frame.size == 0:
            return []
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        h, w = frame.shape[:2]
        out: list[Detection2D] = []
        for r in self._ranges:
            lower_main = np.array([r.h_min, r.s_min, r.v_min], dtype=np.uint8)
            upper_main = np.array([r.h_max, r.s_max, r.v_max], dtype=np.uint8)
            if r.h_min <= r.h_max:
                mask = cv2.inRange(
                    hsv,
                    lower_main,
                    upper_main,
                )
            else:
                a = cv2.inRange(
                    hsv,
                    np.array([0, r.s_min, r.v_min], dtype=np.uint8),
                    np.array([r.h_max, r.s_max, r.v_max], dtype=np.uint8),
                )
                b = cv2.inRange(
                    hsv,
                    np.array([r.h_min, r.s_min, r.v_min], dtype=np.uint8),
                    np.array([179, r.s_max, r.v_max], dtype=np.uint8),
                )
                mask = cv2.bitwise_or(a, b)
            if self._open_ks >= 2:
                k = np.ones((self._open_ks, self._open_ks), dtype=np.uint8)
                mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
            if self._close_ks >= 2:
                k = np.ones((self._close_ks, self._close_ks), dtype=np.uint8)
                mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in contours:
                area = float(cv2.contourArea(c))
                if area < r.min_area:
                    continue
                x, y, bw, bh = cv2.boundingRect(c)
                box = _clamp_box(float(x), float(y), float(x + bw), float(y + bh), w, h)
                if box is None:
                    continue
                x1, y1, x2, y2 = box
                out.append(
                    Detection2D(
                        class_id=r.class_id,
                        confidence=r.confidence,
                        x1=x1,
                        y1=y1,
                        x2=x2,
                        y2=y2,
                        pipeline=self._name,
                    )
                )
        return out


class ContourPipeline(VisionPipeline):
    def __init__(self, spec: dict[str, Any]):
        self._class_id = int(spec.get("class_id", 0))
        self._confidence = float(spec.get("confidence", 1.0))
        self._min_area = float(spec.get("min_area", 120.0))
        self._canny_low = float(spec.get("canny_low", 60.0))
        self._canny_high = float(spec.get("canny_high", 180.0))
        self._blur_ks = int(spec.get("blur_kernel", 5))
        self._name = str(spec.get("type", "contours"))

    def run(self, frame: np.ndarray, now_s: float) -> list[Detection2D]:
        if frame is None or frame.size == 0:
            return []
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        k = max(1, self._blur_ks)
        if k % 2 == 0:
            k += 1
        gray = cv2.GaussianBlur(gray, (k, k), 0)
        edges = cv2.Canny(gray, self._canny_low, self._canny_high)
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        h, w = frame.shape[:2]
        out: list[Detection2D] = []
        for c in contours:
            area = float(cv2.contourArea(c))
            if area < self._min_area:
                continue
            x, y, bw, bh = cv2.boundingRect(c)
            box = _clamp_box(float(x), float(y), float(x + bw), float(y + bh), w, h)
            if box is None:
                continue
            x1, y1, x2, y2 = box
            out.append(
                Detection2D(
                    class_id=self._class_id,
                    confidence=self._confidence,
                    x1=x1,
                    y1=y1,
                    x2=x2,
                    y2=y2,
                    pipeline=self._name,
                )
            )
        return out


class SegmentationPipeline(VisionPipeline):
    def __init__(self, spec: dict[str, Any]):
        self._name = str(spec.get("type", "segmentation"))
        self._fallback = ColorPipeline(spec)
        self._threshold = float(spec.get("threshold", 0.5))
        self._min_area = float(spec.get("min_area", 120.0))
        self._input_width = int(spec.get("input_width", 640))
        self._input_height = int(spec.get("input_height", 640))
        self._default_class_id = int(spec.get("class_id", 0))
        class_lookup_raw = spec.get("class_lookup", {})
        self._class_lookup: dict[int, int] = {}
        if isinstance(class_lookup_raw, dict):
            for k, v in class_lookup_raw.items():
                self._class_lookup[int(k)] = int(v)
        self._class_values_raw = spec.get("class_values", None)
        model_path = str(spec.get("model_path", "")).strip()
        self._net = None
        if model_path:
            p = Path(model_path)
            if not p.exists():
                raise FileNotFoundError(f"segmentation model not found: {p}")
            self._net = cv2.dnn.readNet(str(p))

    def _contours_from_mask(self, mask: np.ndarray, class_id: int, h: int, w: int) -> list[Detection2D]:
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        out: list[Detection2D] = []
        for c in contours:
            area = float(cv2.contourArea(c))
            if area < self._min_area:
                continue
            x, y, bw, bh = cv2.boundingRect(c)
            box = _clamp_box(float(x), float(y), float(x + bw), float(y + bh), w, h)
            if box is None:
                continue
            x1, y1, x2, y2 = box
            out.append(
                Detection2D(
                    class_id=int(class_id),
                    confidence=1.0,
                    x1=x1,
                    y1=y1,
                    x2=x2,
                    y2=y2,
                    pipeline=self._name,
                )
            )
        return out

    def run(self, frame: np.ndarray, now_s: float) -> list[Detection2D]:
        if frame is None or frame.size == 0:
            return []
        if self._net is None:
            return self._fallback.run(frame, now_s)
        h, w = frame.shape[:2]
        blob = cv2.dnn.blobFromImage(
            frame,
            scalefactor=1.0 / 255.0,
            size=(self._input_width, self._input_height),
            swapRB=True,
            crop=False,
        )
        self._net.setInput(blob)
        raw = self._net.forward()
        arr = np.asarray(raw)
        arr = np.squeeze(arr)
        if arr.ndim == 2:
            prob = arr.astype(np.float32)
            mask = (prob >= self._threshold).astype(np.uint8) * 255
            mask = cv2.resize(mask, (w, h), interpolation=cv2.INTER_NEAREST)
            return self._contours_from_mask(mask, self._default_class_id, h, w)
        if arr.ndim == 3:
            if arr.shape[0] <= arr.shape[-1]:
                cls = np.argmax(arr, axis=0).astype(np.int32)
            else:
                cls = np.argmax(arr, axis=-1).astype(np.int32)
            cls = cv2.resize(cls, (w, h), interpolation=cv2.INTER_NEAREST)
            if self._class_values_raw is None:
                class_values = [int(v) for v in np.unique(cls)]
            else:
                class_values = [int(v) for v in self._class_values_raw]
            out: list[Detection2D] = []
            for v in class_values:
                if v < 0:
                    continue
                mask = (cls == v).astype(np.uint8) * 255
                cid = self._class_lookup.get(v, v if v in self._class_lookup else self._default_class_id)
                out.extend(self._contours_from_mask(mask, cid, h, w))
            return out
        return []


class YoloPipeline(VisionPipeline):
    def __init__(self, spec: dict[str, Any]):
        self._name = str(spec.get("type", "yolo"))
        model_path = str(spec.get("model_path", "")).strip()
        if not model_path:
            raise ValueError("yolo pipeline requires model_path")
        p = Path(model_path)
        if not p.exists():
            raise FileNotFoundError(f"yolo model not found: {p}")
        available = set(ort.get_available_providers())
        providers: list[str] = []
        if "CUDAExecutionProvider" in available:
            providers.append("CUDAExecutionProvider")
        if "CPUExecutionProvider" in available:
            providers.append("CPUExecutionProvider")
        if not providers:
            raise RuntimeError("onnxruntime has no available execution providers")

        session_options = ort.SessionOptions()
        session_options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        self._session = ort.InferenceSession(str(p), sess_options=session_options, providers=providers)
        self._input_name = self._session.get_inputs()[0].name
        self._output_name = self._session.get_outputs()[0].name
        self._input_type = str(self._session.get_inputs()[0].type)
        self._input_width = int(spec.get("input_width", 640))
        self._input_height = int(spec.get("input_height", 640))
        self._conf_threshold = float(spec.get("conf_threshold", 0.25))
        self._nms_threshold = float(spec.get("nms_threshold", 0.45))
        self._output_format = str(spec.get("output_format", "yolov8")).strip().lower()
        self._max_detections = int(spec.get("max_detections", 300))

    def _pred_matrix(self, output: Any) -> np.ndarray:
        arr = np.asarray(output)
        arr = np.squeeze(arr)
        if arr.ndim != 2:
            raise ValueError(f"unsupported yolo output shape: {tuple(np.asarray(output).shape)}")
        if arr.shape[0] < arr.shape[1] and arr.shape[0] <= 128:
            arr = arr.T
        return arr.astype(np.float32)

    def run(self, frame: np.ndarray, now_s: float) -> list[Detection2D]:
        if frame is None or frame.size == 0:
            return []
        h, w = frame.shape[:2]
        resized = cv2.resize(
            frame,
            (self._input_width, self._input_height),
            interpolation=cv2.INTER_LINEAR,
        )
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        chw = np.transpose(rgb, (2, 0, 1))
        if "float16" in self._input_type:
            blob = chw.astype(np.float16, copy=False) / np.float16(255.0)
        else:
            blob = chw.astype(np.float32, copy=False) / np.float32(255.0)
        blob = np.expand_dims(blob, axis=0)
        raw = self._session.run([self._output_name], {self._input_name: blob})[0]
        pred = self._pred_matrix(raw)
        if pred.shape[1] < 5:
            return []
        sx = float(w) / float(self._input_width)
        sy = float(h) / float(self._input_height)

        # End-to-end ONNX export path (x1, y1, x2, y2, score, class_id).
        if pred.shape[1] == 6:
            out: list[Detection2D] = []
            for row in pred:
                x1, y1, x2, y2, score, cls_raw = map(float, row[:6])
                if score < self._conf_threshold:
                    continue
                if max(abs(x1), abs(y1), abs(x2), abs(y2)) <= 2.0:
                    x1 *= float(self._input_width)
                    y1 *= float(self._input_height)
                    x2 *= float(self._input_width)
                    y2 *= float(self._input_height)
                clamped = _clamp_box(x1 * sx, y1 * sy, x2 * sx, y2 * sy, w, h)
                if clamped is None:
                    continue
                cx1, cy1, cx2, cy2 = clamped
                out.append(
                    Detection2D(
                        class_id=max(0, int(round(cls_raw))),
                        confidence=float(score),
                        x1=float(cx1),
                        y1=float(cy1),
                        x2=float(cx2),
                        y2=float(cy2),
                        pipeline=self._name,
                    )
                )
                if len(out) >= self._max_detections:
                    break
            return out

        class_ids: list[int] = []
        confidences: list[float] = []
        boxes_xywh: list[list[float]] = []
        for row in pred:
            cx, cy, bw, bh = map(float, row[:4])
            if max(abs(cx), abs(cy), abs(bw), abs(bh)) <= 2.0:
                cx *= float(self._input_width)
                cy *= float(self._input_height)
                bw *= float(self._input_width)
                bh *= float(self._input_height)
            if self._output_format == "yolov5":
                if row.shape[0] < 6:
                    continue
                obj = float(row[4])
                class_scores = row[5:]
                if class_scores.size == 0:
                    continue
                cls = int(np.argmax(class_scores))
                conf = float(obj * float(class_scores[cls]))
            else:
                class_scores = row[4:]
                if class_scores.size == 0:
                    continue
                cls = int(np.argmax(class_scores))
                conf = float(class_scores[cls])
            if conf < self._conf_threshold:
                continue
            x1 = (cx - 0.5 * bw) * sx
            y1 = (cy - 0.5 * bh) * sy
            x2 = (cx + 0.5 * bw) * sx
            y2 = (cy + 0.5 * bh) * sy
            clamped = _clamp_box(x1, y1, x2, y2, w, h)
            if clamped is None:
                continue
            cx1, cy1, cx2, cy2 = clamped
            boxes_xywh.append([cx1, cy1, cx2 - cx1, cy2 - cy1])
            class_ids.append(cls)
            confidences.append(conf)
        if not boxes_xywh:
            return []
        nms_idx = cv2.dnn.NMSBoxes(boxes_xywh, confidences, self._conf_threshold, self._nms_threshold)
        if nms_idx is None or len(nms_idx) == 0:
            return []
        out: list[Detection2D] = []
        flat_idx = np.asarray(nms_idx, dtype=np.int32).reshape(-1)
        for raw_i in flat_idx[: max(0, self._max_detections)]:
            i = int(raw_i)
            x, y, bw, bh = boxes_xywh[i]
            out.append(
                Detection2D(
                    class_id=int(class_ids[i]),
                    confidence=float(confidences[i]),
                    x1=float(x),
                    y1=float(y),
                    x2=float(x + bw),
                    y2=float(y + bh),
                    pipeline=self._name,
                )
            )
        return out


class MultiPipeline(VisionPipeline):
    def __init__(self, children: list[VisionPipeline], spec: dict[str, Any]):
        self._children = children
        self._dedupe_iou = float(spec.get("dedupe_iou", 0.7))
        self._max_detections = int(spec.get("max_detections", 500))

    def run(self, frame: np.ndarray, now_s: float) -> list[Detection2D]:
        merged: list[Detection2D] = []
        for child in self._children:
            merged.extend(child.run(frame, now_s))
        merged.sort(key=lambda d: float(d.confidence), reverse=True)
        out: list[Detection2D] = []
        for d in merged:
            keep = True
            for e in out:
                if d.class_id == e.class_id and _iou(d, e) >= self._dedupe_iou:
                    keep = False
                    break
            if keep:
                out.append(d)
            if len(out) >= self._max_detections:
                break
        return out


def build_pipeline(spec: dict[str, Any]) -> VisionPipeline:
    if not isinstance(spec, dict):
        raise ValueError("pipeline spec must be an object")
    ptype = str(spec.get("type", "yolo")).strip().lower()
    if ptype == "yolo":
        return YoloPipeline(spec)
    if ptype == "color":
        return ColorPipeline(spec)
    if ptype == "contours":
        return ContourPipeline(spec)
    if ptype == "segmentation":
        return SegmentationPipeline(spec)
    if ptype in ("multi", "ensemble"):
        raw_children = spec.get("pipelines", [])
        if not isinstance(raw_children, list) or not raw_children:
            raise ValueError("multi pipeline requires non-empty pipelines list")
        children = [build_pipeline(x) for x in raw_children if isinstance(x, dict)]
        if not children:
            raise ValueError("multi pipeline has no valid child pipeline")
        return MultiPipeline(children, spec)
    raise ValueError(f"unknown pipeline type: {ptype}")
