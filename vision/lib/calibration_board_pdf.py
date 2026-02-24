from __future__ import annotations

import argparse
import zlib
from pathlib import Path
from typing import Any, cast

import cv2
import numpy as np


def _mm_to_pt(mm: float) -> float:
    return float(mm) * 72.0 / 25.4


def _page_size_pt(name: str) -> tuple[float, float]:
    n = str(name).strip().lower()
    if n == "a4":
        return _mm_to_pt(210.0), _mm_to_pt(297.0)
    if n == "letter":
        return 612.0, 792.0
    raise ValueError(f"unsupported page size: {name}")


def _orient(page_w_pt: float, page_h_pt: float, orientation: str) -> tuple[float, float]:
    o = str(orientation).strip().lower()
    if o == "portrait":
        return (min(page_w_pt, page_h_pt), max(page_w_pt, page_h_pt))
    if o == "landscape":
        return (max(page_w_pt, page_h_pt), min(page_w_pt, page_h_pt))
    if o == "auto":
        return page_w_pt, page_h_pt
    raise ValueError(f"unsupported orientation: {orientation}")


def _dictionary_by_name(name: str):
    if not hasattr(cv2, "aruco"):
        raise RuntimeError("cv2.aruco is required for charuco")
    aruco = cast(Any, cv2.aruco)
    n = str(name).strip().upper()
    code = getattr(aruco, n, None)
    if code is None:
        raise ValueError(f"unsupported aruco dictionary: {name}")
    return aruco.getPredefinedDictionary(int(code))


def _make_charuco_board(squares_x: int, squares_y: int, marker_ratio: float, dictionary):
    aruco = cast(Any, cv2.aruco)
    mr = float(marker_ratio)
    if mr <= 0.0 or mr >= 1.0:
        raise ValueError("marker-ratio must be in (0, 1)")
    if hasattr(aruco, "CharucoBoard"):
        return aruco.CharucoBoard((int(squares_x), int(squares_y)), 1.0, mr, dictionary)
    return aruco.CharucoBoard_create(int(squares_x), int(squares_y), 1.0, mr, dictionary)


def _draw_charuco_image(squares_x: int, squares_y: int, cell_px: int, marker_ratio: float, dict_name: str, invert: bool) -> np.ndarray:
    dictionary = _dictionary_by_name(dict_name)
    board = _make_charuco_board(squares_x, squares_y, marker_ratio, dictionary)
    board_any = cast(Any, board)
    width = int(squares_x) * int(cell_px)
    height = int(squares_y) * int(cell_px)
    if hasattr(board_any, "generateImage"):
        img = board_any.generateImage((width, height), marginSize=0, borderBits=1)
    else:
        img = board_any.draw((width, height), marginSize=0, borderBits=1)
    out = np.asarray(img, dtype=np.uint8)
    if out.ndim == 3:
        out = cv2.cvtColor(out, cv2.COLOR_BGR2GRAY)
    if invert:
        out = 255 - out
    return out


def _draw_checkerboard_image(squares_x: int, squares_y: int, cell_px: int, invert: bool) -> np.ndarray:
    h = int(squares_y) * int(cell_px)
    w = int(squares_x) * int(cell_px)
    out = np.full((h, w), 255, dtype=np.uint8)
    for y in range(int(squares_y)):
        for x in range(int(squares_x)):
            parity = (x + y) % 2
            draw_black = (parity == 0 and not invert) or (parity == 1 and invert)
            if draw_black:
                y0 = y * cell_px
                x0 = x * cell_px
                out[y0 : y0 + cell_px, x0 : x0 + cell_px] = 0
    return out


def _write_pdf_with_image(
    page_w_pt: float,
    page_h_pt: float,
    image_gray: np.ndarray,
    x_pt: float,
    y_pt: float,
    w_pt: float,
    h_pt: float,
    out_path: Path,
) -> None:
    img = np.asarray(image_gray, dtype=np.uint8)
    if img.ndim != 2:
        raise ValueError("image must be grayscale")
    h_px, w_px = img.shape
    compressed = zlib.compress(img.tobytes(), level=9)
    content = f"q\n{w_pt:.6f} 0 0 {h_pt:.6f} {x_pt:.6f} {y_pt:.6f} cm\n/Im0 Do\nQ\n".encode("ascii")

    obj1 = b"1 0 obj\n<< /Type /Catalog /Pages 2 0 R >>\nendobj\n"
    obj2 = b"2 0 obj\n<< /Type /Pages /Kids [3 0 R] /Count 1 >>\nendobj\n"
    obj3 = (
        f"3 0 obj\n<< /Type /Page /Parent 2 0 R /MediaBox [0 0 {page_w_pt:.6f} {page_h_pt:.6f}] "
        f"/Resources << /ProcSet [/PDF /ImageB] /XObject << /Im0 5 0 R >> >> /Contents 4 0 R >>\nendobj\n"
    ).encode("ascii")
    obj4 = f"4 0 obj\n<< /Length {len(content)} >>\nstream\n".encode("ascii") + content + b"endstream\nendobj\n"
    obj5 = (
        f"5 0 obj\n<< /Type /XObject /Subtype /Image /Width {w_px} /Height {h_px} "
        f"/ColorSpace /DeviceGray /BitsPerComponent 8 /Filter /FlateDecode /Length {len(compressed)} >>\nstream\n".encode("ascii")
        + compressed
        + b"\nendstream\nendobj\n"
    )

    parts = [b"%PDF-1.4\n%\x93\x8c\x8b\x9e\n", obj1, obj2, obj3, obj4, obj5]
    offsets = [0]
    cur = len(parts[0])
    for p in parts[1:]:
        offsets.append(cur)
        cur += len(p)
    xref_start = cur
    xref = [b"xref\n0 6\n", b"0000000000 65535 f \n"]
    for off in offsets[1:]:
        xref.append(f"{off:010d} 00000 n \n".encode("ascii"))
    trailer = b"trailer\n<< /Size 6 /Root 1 0 R >>\n"
    startxref = f"startxref\n{xref_start}\n%%EOF\n".encode("ascii")
    out_path.write_bytes(b"".join(parts + xref + [trailer, startxref]))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--pattern", type=str, default="charuco", choices=["charuco", "checkerboard"])
    parser.add_argument("--cols", type=int, default=9)
    parser.add_argument("--rows", type=int, default=6)
    parser.add_argument("--square-mm", type=float, default=25.0)
    parser.add_argument("--marker-ratio", type=float, default=0.70)
    parser.add_argument("--aruco-dict", type=str, default="DICT_5X5_1000")
    parser.add_argument("--page", type=str, default="A4")
    parser.add_argument("--orientation", type=str, default="auto")
    parser.add_argument("--margin-mm", type=float, default=10.0)
    parser.add_argument("--invert", action="store_true")
    parser.add_argument("--out-pdf", type=str, default="calibration_board.pdf")
    parser.add_argument("--out-png", type=str, default="")
    parser.add_argument("--render-dpi", type=int, default=400)
    args = parser.parse_args()

    cols = int(args.cols)
    rows = int(args.rows)
    if cols < 2 or rows < 2:
        raise ValueError("rows and cols must be >= 2")
    square_mm = float(args.square_mm)
    if square_mm <= 0.0:
        raise ValueError("square-mm must be > 0")

    squares_x = cols + 1
    squares_y = rows + 1
    dpi = max(100, int(args.render_dpi))
    cell_px = max(20, int(round((square_mm / 25.4) * float(dpi))))
    pattern = str(args.pattern).strip().lower()
    if pattern == "charuco":
        board = _draw_charuco_image(
            squares_x=squares_x,
            squares_y=squares_y,
            cell_px=cell_px,
            marker_ratio=float(args.marker_ratio),
            dict_name=str(args.aruco_dict),
            invert=bool(args.invert),
        )
    else:
        board = _draw_checkerboard_image(squares_x=squares_x, squares_y=squares_y, cell_px=cell_px, invert=bool(args.invert))

    base_w_pt, base_h_pt = _page_size_pt(str(args.page))
    page_w_pt, page_h_pt = _orient(base_w_pt, base_h_pt, str(args.orientation))
    margin_pt = _mm_to_pt(float(args.margin_mm))
    square_pt = _mm_to_pt(square_mm)
    board_w_pt = float(squares_x) * square_pt
    board_h_pt = float(squares_y) * square_pt
    usable_w = page_w_pt - 2.0 * margin_pt
    usable_h = page_h_pt - 2.0 * margin_pt
    if board_w_pt > usable_w or board_h_pt > usable_h:
        if str(args.orientation).strip().lower() == "auto":
            page_w_pt, page_h_pt = page_h_pt, page_w_pt
            usable_w = page_w_pt - 2.0 * margin_pt
            usable_h = page_h_pt - 2.0 * margin_pt
        if board_w_pt > usable_w or board_h_pt > usable_h:
            raise ValueError(
                f"board does not fit page: board={board_w_pt:.2f}x{board_h_pt:.2f}pt usable={usable_w:.2f}x{usable_h:.2f}pt"
            )

    x_pt = 0.5 * (page_w_pt - board_w_pt)
    y_pt = 0.5 * (page_h_pt - board_h_pt)
    out_pdf = Path(str(args.out_pdf)).resolve()
    _write_pdf_with_image(
        page_w_pt=page_w_pt,
        page_h_pt=page_h_pt,
        image_gray=board,
        x_pt=x_pt,
        y_pt=y_pt,
        w_pt=board_w_pt,
        h_pt=board_h_pt,
        out_path=out_pdf,
    )

    out_png = str(args.out_png).strip()
    if out_png:
        cv2.imwrite(str(Path(out_png).resolve()), board)

    print(str(out_pdf))


if __name__ == "__main__":
    main()
