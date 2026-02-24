from __future__ import annotations

import argparse
import os
import random
from pathlib import Path

import numpy as np
import torch
from ultralytics import YOLO


def seed_everything(seed: int) -> None:
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    torch.cuda.manual_seed_all(seed)
    torch.backends.cudnn.deterministic = False
    torch.backends.cudnn.benchmark = True


def pick_device(device: str) -> str:
    d = device.strip().lower()
    if d in {"auto", ""}:
        return "0" if torch.cuda.is_available() else "cpu"
    if d == "cpu":
        return "cpu"
    if d.startswith("cuda:"):
        return d.split("cuda:", 1)[1] or "0"
    return d


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--data", type=str, required=True)
    p.add_argument("--model", type=str, default="yolov8n.pt")
    p.add_argument("--imgsz", type=int, default=1024)
    p.add_argument("--epochs", type=int, default=300)
    p.add_argument("--batch", type=int, default=24)
    p.add_argument("--workers", type=int, default=16)
    p.add_argument("--device", type=str, default="0")
    p.add_argument("--seed", type=int, default=42)
    p.add_argument("--project", type=str, default="runs/fuel")
    p.add_argument("--name", type=str, default="train")
    p.add_argument("--resume", action="store_true")

    p.add_argument("--cache", type=str, default="ram", choices=["ram", "disk", "false"])
    p.add_argument("--patience", type=int, default=80)

    p.add_argument("--lr0", type=float, default=0.003)
    p.add_argument("--lrf", type=float, default=0.01)
    p.add_argument("--weight_decay", type=float, default=0.01)
    p.add_argument("--warmup_epochs", type=float, default=3.0)

    p.add_argument("--close_mosaic", type=int, default=10)
    p.add_argument("--mosaic", type=float, default=1.0)
    p.add_argument("--mixup", type=float, default=0.10)
    p.add_argument("--copy_paste", type=float, default=0.05)
    p.add_argument("--degrees", type=float, default=8.0)
    p.add_argument("--translate", type=float, default=0.10)
    p.add_argument("--scale", type=float, default=0.50)
    p.add_argument("--shear", type=float, default=0.0)
    p.add_argument("--perspective", type=float, default=0.0)
    p.add_argument("--fliplr", type=float, default=0.50)
    p.add_argument("--flipud", type=float, default=0.00)
    p.add_argument("--hsv_h", type=float, default=0.015)
    p.add_argument("--hsv_s", type=float, default=0.70)
    p.add_argument("--hsv_v", type=float, default=0.40)
    p.add_argument("--erasing", type=float, default=0.10)

    p.add_argument("--amp", type=str, default="true", choices=["true", "false"])
    p.add_argument("--half", type=str, default="false", choices=["true", "false"])
    p.add_argument("--optimizer", type=str, default="AdamW", choices=["auto", "SGD", "AdamW"])
    p.add_argument("--cos_lr", type=str, default="true", choices=["true", "false"])
    p.add_argument("--freeze", type=int, default=0)

    p.add_argument("--compile", type=str, default="true", choices=["true", "false"])
    p.add_argument("--matmul", type=str, default="highest", choices=["medium", "high", "highest"])

    args = p.parse_args()

    seed_everything(args.seed)

    data_path = Path(args.data).expanduser().resolve()
    if not data_path.exists():
        raise SystemExit(f"data.yaml not found: {data_path}")

    device = pick_device(args.device)

    os.environ.setdefault("ULTRALYTICS_SETTINGS_DIR", str(Path.cwd() / ".ultralytics"))
    os.environ.setdefault(
        "PYTORCH_CUDA_ALLOC_CONF",
        "expandable_segments:True,garbage_collection_threshold:0.8,max_split_size_mb:128",
    )

    if torch.cuda.is_available():
        torch.set_float32_matmul_precision(args.matmul)

    model = YOLO(args.model)

    cache_val = args.cache
    if cache_val == "false":
        cache_val = False

    model.train(
        data=str(data_path),
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        workers=args.workers,
        device=device,
        project=args.project,
        name=args.name,
        resume=args.resume,
        cache=cache_val,
        patience=args.patience,
        lr0=args.lr0,
        lrf=args.lrf,
        weight_decay=args.weight_decay,
        warmup_epochs=args.warmup_epochs,
        close_mosaic=args.close_mosaic,
        optimizer=args.optimizer,
        cos_lr=(args.cos_lr == "true"),
        amp=(args.amp == "true"),
        half=(args.half == "true"),
        freeze=args.freeze,
        mosaic=args.mosaic,
        mixup=args.mixup,
        copy_paste=args.copy_paste,
        degrees=args.degrees,
        translate=args.translate,
        scale=args.scale,
        shear=args.shear,
        perspective=args.perspective,
        fliplr=args.fliplr,
        flipud=args.flipud,
        hsv_h=args.hsv_h,
        hsv_s=args.hsv_s,
        hsv_v=args.hsv_v,
        erasing=args.erasing,
        pretrained=True,
        plots=True,
        save=True,
        val=True,
        compile=(args.compile == "true"),
    )

    model.val(data=str(data_path), imgsz=args.imgsz, device=device)


if __name__ == "__main__":
    main()