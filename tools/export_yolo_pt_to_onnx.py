from __future__ import annotations

import argparse
import os
import shutil
import sys
from pathlib import Path


def _resolve_path(repo_root: Path, raw: str) -> Path:
    p = Path(raw).expanduser()
    if not p.is_absolute():
        p = repo_root / p
    return p.resolve()


def _find_exported_onnx(pt_path: Path, export_result: object) -> Path:
    if isinstance(export_result, str):
        p = Path(export_result).expanduser()
        if p.exists():
            return p.resolve()
    elif isinstance(export_result, Path) and export_result.exists():
        return export_result.resolve()

    candidates = sorted(
        pt_path.parent.glob(f"{pt_path.stem}*.onnx"),
        key=lambda x: x.stat().st_mtime,
        reverse=True,
    )
    if not candidates:
        raise FileNotFoundError(
            f"Could not find exported ONNX near {pt_path}. "
            "Ultralytics export did not produce an .onnx file."
        )
    return candidates[0].resolve()


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Export a YOLO .pt model to ONNX and copy it into vision/models so the "
            "local vision runtime and deploy bundle use it."
        )
    )
    parser.add_argument(
        "--pt",
        required=True,
        help="Path to source YOLO .pt model (absolute or repo-relative).",
    )
    parser.add_argument(
        "--out",
        default="vision/models/yolo.onnx",
        help="Destination ONNX path (default: vision/models/yolo.onnx).",
    )
    parser.add_argument("--imgsz", type=int, default=640, help="Export image size.")
    parser.add_argument("--opset", type=int, default=12, help="ONNX opset version.")
    parser.add_argument("--batch", type=int, default=1, help="ONNX batch size.")
    parser.add_argument("--device", default=None, help="Export device (cpu, 0, 0,1, etc).")
    parser.add_argument(
        "--dynamic",
        action="store_true",
        help="Enable dynamic axes in exported ONNX.",
    )
    parser.add_argument(
        "--simplify",
        action="store_true",
        help="Run ONNX graph simplifier (requires extra deps).",
    )
    parser.add_argument(
        "--half",
        action="store_true",
        help="Export FP16 model (target-dependent).",
    )
    args = parser.parse_args()

    repo_root = Path(__file__).resolve().parents[1]
    pt_path = _resolve_path(repo_root, args.pt)
    out_path = _resolve_path(repo_root, args.out)

    if not pt_path.exists():
        print(f"error: source model not found: {pt_path}", file=sys.stderr)
        return 2
    if pt_path.suffix.lower() != ".pt":
        print(f"error: source model must be a .pt file: {pt_path}", file=sys.stderr)
        return 2

    os.environ.setdefault("ULTRALYTICS_SETTINGS_DIR", str(repo_root / ".ultralytics"))

    try:
        from ultralytics import YOLO  # pyright: ignore[reportMissingImports]
    except Exception as exc:
        print(
            "error: failed to import ultralytics. Install vision deps first "
            "(for example: `pip install -r vision/requirements.txt`).",
            file=sys.stderr,
        )
        print(str(exc), file=sys.stderr)
        return 3

    print(f"[export] source: {pt_path}")
    model = YOLO(str(pt_path))
    export_kwargs: dict[str, object] = {
        "format": "onnx",
        "imgsz": int(args.imgsz),
        "opset": int(args.opset),
        "batch": int(args.batch),
        "dynamic": bool(args.dynamic),
        "simplify": bool(args.simplify),
        "half": bool(args.half),
    }
    if args.device is not None:
        export_kwargs["device"] = str(args.device)

    try:
        export_result = model.export(**export_kwargs)
    except Exception as exc:
        print("error: ONNX export failed.", file=sys.stderr)
        print(str(exc), file=sys.stderr)
        return 4

    exported_onnx = _find_exported_onnx(pt_path, export_result)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(exported_onnx, out_path)

    rel_out = out_path.relative_to(repo_root) if out_path.is_relative_to(repo_root) else out_path
    print(f"[export] onnx:   {exported_onnx}")
    print(f"[install] model copied to: {out_path}")
    print(
        "[install] runtime config uses ../models/yolo.onnx, so this destination is "
        f"ready for local tests and deploy bundling ({rel_out})."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())