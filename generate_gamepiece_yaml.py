import argparse
import math
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Optional, Literal, Dict, List

import trimesh
import yaml

AreaMethod = Literal["projected", "bbox_ellipse", "bbox_rect"]

MATERIAL_DENSITY_KG_PER_M3: Dict[str, float] = {
    "foam_low": 20.0,
    "foam_high": 2228.23,
    "rubber": 1100.0,
    "plastic": 950.0,
    "wood": 700.0,
    "aluminum": 2700.0,
    "steel": 7850.0,
    "pvc": 1467.0,  
}

MATERIAL_CHOICES = sorted(list(MATERIAL_DENSITY_KG_PER_M3.keys()) + ["custom"])

UNIT_SCALE_TO_M: Dict[str, float] = {
    "m": 1.0,
    "mm": 0.001,
    "cm": 0.01,
    "in": 0.0254,
    "ft": 0.3048,
}


@dataclass
class GamePieceMetadata:
    source_mesh: str
    mesh_units: str
    scale: float
    volume_m3: float
    bounds_extents_m: List[float]
    flight_axis: str
    density_kg_per_m3: Optional[float]
    mass_kg_override: Optional[float]
    area_method: str
    auto_shape_hint: Optional[str]
    auto_drag_coefficient_hint: Optional[float]
    material: Optional[str]
    estimated_terminal_velocity_mps: float
    alt_cross_section_estimates_m2: Dict[str, float]
    warnings: List[str]


@dataclass
class GamePieceYaml:
    name: str
    mass_kg: float
    drag_coefficient: float
    cross_section_area_m2: float
    air_density_kg_per_m3: float
    metadata: GamePieceMetadata


def load_mesh_any(path: Path, scale: float) -> trimesh.Trimesh:
    raw = trimesh.load_mesh(path, process=True)
    if isinstance(raw, trimesh.Scene):
        if not raw.geometry:
            raise ValueError(f"Scene '{path}' has no geometry")
        mesh = trimesh.util.concatenate(list(raw.geometry.values()))
    else:
        mesh = raw
    if scale != 1.0:
        mesh.apply_scale(scale)
    if mesh.is_empty:
        raise ValueError(f"Mesh '{path}' is empty after processing")
    return mesh


def axis_vector(axis: str) -> tuple[float, float, float]:
    a = axis.lower()
    if a == "x":
        return 1.0, 0.0, 0.0
    if a == "y":
        return 0.0, 1.0, 0.0
    if a == "z":
        return 0.0, 0.0, 1.0
    raise ValueError(f"Invalid flight_axis '{axis}', expected x, y or z")


def compute_mass(
    mesh: trimesh.Trimesh,
    density_kg_per_m3: Optional[float],
    mass_override_kg: Optional[float],
) -> float:
    if mass_override_kg is not None:
        if mass_override_kg <= 0.0:
            raise ValueError("mass_kg override must be > 0")
        return float(mass_override_kg)
    if density_kg_per_m3 is None or density_kg_per_m3 <= 0.0:
        raise ValueError("Either mass_kg override or density_kg_per_m3 > 0 must be provided")
    volume_m3 = float(mesh.volume)
    if volume_m3 <= 0.0:
        raise ValueError("Mesh volume is non-positive, check mesh units or integrity")
    return float(density_kg_per_m3) * volume_m3


def compute_cross_section_area_projected(mesh: trimesh.Trimesh, flight_axis: str) -> float:
    m = mesh
    if not m.is_volume:
        m = m.convex_hull
    ux, uy, uz = axis_vector(flight_axis)
    normals = m.face_normals
    areas = m.area_faces
    if normals.shape[0] != areas.shape[0]:
        raise ValueError("Face normals and areas mismatch")
    dot_abs = abs(normals[:, 0] * ux + normals[:, 1] * uy + normals[:, 2] * uz)
    projected_area = float(0.5 * (areas * dot_abs).sum())
    if projected_area <= 0.0:
        raise ValueError("Projected cross-section area computed as <= 0")
    return projected_area


def compute_cross_section_area_bbox_ellipse(mesh: trimesh.Trimesh, flight_axis: str) -> float:
    extents = mesh.extents
    if extents is None or len(extents) != 3:
        raise ValueError("Mesh extents unavailable or invalid")
    axis_map = {"x": 0, "y": 1, "z": 2}
    axis_idx = axis_map[flight_axis.lower()]
    idx_other1 = (axis_idx + 1) % 3
    idx_other2 = (axis_idx + 2) % 3
    a = float(extents[idx_other1])
    b = float(extents[idx_other2])
    if a <= 0.0 or b <= 0.0:
        raise ValueError("Mesh extents are degenerate, cannot compute ellipse area")
    return math.pi * (a * 0.5) * (b * 0.5)


def compute_cross_section_area_bbox_rect(mesh: trimesh.Trimesh, flight_axis: str) -> float:
    extents = mesh.extents
    if extents is None or len(extents) != 3:
        raise ValueError("Mesh extents unavailable or invalid")
    axis_map = {"x": 0, "y": 1, "z": 2}
    axis_idx = axis_map[flight_axis.lower()]
    idx_other1 = (axis_idx + 1) % 3
    idx_other2 = (axis_idx + 2) % 3
    a = float(extents[idx_other1])
    b = float(extents[idx_other2])
    if a <= 0.0 or b <= 0.0:
        raise ValueError("Mesh extents are degenerate, cannot compute rectangle area")
    return a * b


def guess_shape_and_cd(extents: List[float]) -> tuple[Optional[str], Optional[float]]:
    if len(extents) != 3:
        return None, None
    ex, ey, ez = sorted(float(e) for e in extents)
    if ex <= 0.0 or ey <= 0.0 or ez <= 0.0:
        return None, None
    r1 = ey / ex
    r2 = ez / ex
    if 0.8 <= r1 <= 1.25 and 0.8 <= r2 <= 1.25:
        return "sphere-ish", 0.47
    if r2 > 2.0 and r1 < 0.6:
        return "flat_disc-ish", 1.1
    if r2 > 2.0 and 0.6 <= r1 <= 1.4:
        return "cylinder-ish", 0.8
    return None, None


def compute_area(mesh: trimesh.Trimesh, method: AreaMethod, flight_axis: str) -> float:
    if method == "projected":
        return compute_cross_section_area_projected(mesh, flight_axis)
    if method == "bbox_ellipse":
        return compute_cross_section_area_bbox_ellipse(mesh, flight_axis)
    if method == "bbox_rect":
        return compute_cross_section_area_bbox_rect(mesh, flight_axis)
    raise ValueError(f"Unknown area method: {method}")


def estimate_terminal_velocity(
    mass_kg: float,
    area_m2: float,
    cd: float,
    air_density: float,
) -> float:
    if mass_kg <= 0.0 or area_m2 <= 0.0 or cd <= 0.0 or air_density <= 0.0:
        return 0.0
    return math.sqrt(2.0 * mass_kg * 9.81 / (air_density * area_m2 * cd))


def build_yaml(
    mesh_path: Path,
    name: str,
    flight_axis: str,
    material: Optional[str],
    density_kg_per_m3: Optional[float],
    mass_kg: Optional[float],
    drag_coefficient: Optional[float],
    air_density_kg_per_m3: float,
    scale: float,
    mesh_units: str,
    area_method: AreaMethod,
) -> GamePieceYaml:
    mesh = load_mesh_any(mesh_path, scale)
    volume_m3 = float(mesh.volume)

    eff_density = density_kg_per_m3
    if eff_density is None and material and material in MATERIAL_DENSITY_KG_PER_M3:
        eff_density = MATERIAL_DENSITY_KG_PER_M3[material]

    mass_value = compute_mass(mesh, eff_density, mass_kg)

    alt_areas: Dict[str, float] = {}
    warnings: List[str] = []

    for method in ("projected", "bbox_ellipse", "bbox_rect"):
        try:
            alt_areas[method] = compute_area(mesh, method, flight_axis)
        except Exception as e:
            warnings.append(f"Area estimate failed for {method}: {e!s}")

    if area_method not in alt_areas:
        raise ValueError(f"Requested area_method '{area_method}' did not produce a value")

    area_value = alt_areas[area_method]
    extents = [float(x) for x in mesh.extents]

    shape_hint, cd_hint = guess_shape_and_cd(extents)
    if drag_coefficient is None:
        drag_value = cd_hint if cd_hint is not None else 0.75
    else:
        drag_value = drag_coefficient

    vt = estimate_terminal_velocity(
        mass_kg=mass_value,
        area_m2=area_value,
        cd=drag_value,
        air_density=air_density_kg_per_m3,
    )

    if vt < 3.0:
        warnings.append(f"Very low estimated terminal velocity ({vt:.2f} m/s) - check mass/area/cd")
    if vt > 80.0:
        warnings.append(f"Very high estimated terminal velocity ({vt:.2f} m/s) - check mass/area/cd")

    meta = GamePieceMetadata(
        source_mesh=mesh_path.name,
        mesh_units=mesh_units,
        scale=float(scale),
        volume_m3=volume_m3,
        bounds_extents_m=extents,
        flight_axis=flight_axis,
        density_kg_per_m3=None if eff_density is None else float(eff_density),
        mass_kg_override=None if mass_kg is None else float(mass_kg),
        area_method=area_method,
        auto_shape_hint=shape_hint,
        auto_drag_coefficient_hint=cd_hint,
        material=material,
        estimated_terminal_velocity_mps=vt,
        alt_cross_section_estimates_m2=alt_areas,
        warnings=warnings,
    )

    return GamePieceYaml(
        name=name,
        mass_kg=float(mass_value),
        drag_coefficient=float(drag_value),
        cross_section_area_m2=float(area_value),
        air_density_kg_per_m3=float(air_density_kg_per_m3),
        metadata=meta,
    )


def write_yaml(obj: GamePieceYaml, output_path: Path, pretty: bool) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    data = asdict(obj)
    with output_path.open("w", encoding="utf-8") as f:
        if pretty:
            yaml.safe_dump(data, f, sort_keys=False, indent=2)
        else:
            yaml.safe_dump(data, f, sort_keys=False)


def print_summary(obj: GamePieceYaml) -> None:
    m = obj.metadata
    print(f"Game piece: {obj.name}")
    print(f"  mass_kg                        = {obj.mass_kg:.6f}")
    print(f"  cross_section_area_m2          = {obj.cross_section_area_m2:.6f}")
    print(f"  drag_coefficient               = {obj.drag_coefficient:.4f}")
    print(f"  air_density_kg_per_m3          = {obj.air_density_kg_per_m3:.4f}")
    print(f"  estimated_terminal_velocity_mps= {m.estimated_terminal_velocity_mps:.3f}")
    print("  metadata:")
    print(f"    source_mesh                  = {m.source_mesh}")
    print(f"    mesh_units                   = {m.mesh_units}")
    print(f"    scale                        = {m.scale}")
    print(f"    volume_m3                    = {m.volume_m3:.6f}")
    print(f"    bounds_extents_m             = {m.bounds_extents_m}")
    print(f"    flight_axis                  = {m.flight_axis}")
    print(f"    material                     = {m.material}")
    print(f"    density_kg_per_m3            = {m.density_kg_per_m3}")
    print(f"    mass_kg_override             = {m.mass_kg_override}")
    print(f"    area_method                  = {m.area_method}")
    print(f"    auto_shape_hint              = {m.auto_shape_hint}")
    print(f"    auto_drag_coefficient_hint   = {m.auto_drag_coefficient_hint}")
    print(f"    alt_cross_section_estimates_m2= {m.alt_cross_section_estimates_m2}")
    if m.warnings:
        print("    warnings:")
        for w in m.warnings:
            print(f"      - {w}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Generate FRC game piece physics YAML from a 3D mesh"
    )
    parser.add_argument(
        "--mesh",
        type=Path,
        required=True,
        help="Path to 3D mesh file (e.g. .stl, .obj)",
    )
    parser.add_argument(
        "--name",
        type=str,
        required=True,
        help="Logical game piece name (e.g. note)",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Output YAML path (default: <deploy_root>/gamepieces/<name>.yaml)",
    )
    parser.add_argument(
        "--flight-axis",
        type=str,
        choices=["x", "y", "z"],
        default="x",
        help="Axis along which the piece flies (for drag cross-section)",
    )
    parser.add_argument(
        "--mesh-units",
        type=str,
        choices=list(UNIT_SCALE_TO_M.keys()),
        default="m",
        help="Units of the mesh file (used to derive scale): m, mm, cm, in, ft",
    )
    parser.add_argument(
        "--material",
        type=str,
        choices=MATERIAL_CHOICES,
        default="custom",
        help="Material preset for density (foam_low, foam_high, rubber, plastic, wood, aluminum, steel, custom)",
    )
    parser.add_argument(
        "--density-kg-per-m3",
        type=float,
        default=None,
        help="Material density for mass estimation if mass_kg is not provided. Overrides material preset if set.",
    )
    parser.add_argument(
        "--mass-kg",
        type=float,
        default=None,
        help="Override mass in kg (if known exactly)",
    )
    parser.add_argument(
        "--drag-coefficient",
        type=float,
        default=None,
        help="Optional drag coefficient override; if omitted, a heuristic guess is used",
    )
    parser.add_argument(
        "--air-density-kg-per-m3",
        type=float,
        default=1.225,
        help="Air density in kg/m^3",
    )
    parser.add_argument(
        "--scale",
        type=float,
        default=1.0,
        help="Additional uniform scale factor applied after mesh-units conversion",
    )
    parser.add_argument(
        "--area-method",
        type=str,
        choices=["projected", "bbox_ellipse", "bbox_rect"],
        default="projected",
        help="Method to select for cross_section_area_m2 (all methods are still stored in metadata)",
    )
    parser.add_argument(
        "--deploy-root",
        type=Path,
        default=Path("src/main/deploy"),
        help="Root deploy directory of the robot project",
    )
    parser.add_argument(
        "--stdout",
        action="store_true",
        help="Print YAML to stdout instead of writing to file (still prints summary)",
    )
    parser.add_argument(
        "--no-summary",
        action="store_true",
        help="Do not print human-readable summary, only YAML",
    )
    parser.add_argument(
        "--pretty",
        action="store_true",
        help="Pretty-print YAML with indentation",
        default=True
    )
    args = parser.parse_args()

    unit_scale = UNIT_SCALE_TO_M[args.mesh_units]
    final_scale = unit_scale * args.scale

    material = args.material if args.material != "custom" else None

    obj = build_yaml(
        mesh_path=args.mesh,
        name=args.name,
        flight_axis=args.flight_axis,
        material=material,
        density_kg_per_m3=args.density_kg_per_m3,
        mass_kg=args.mass_kg,
        drag_coefficient=args.drag_coefficient,
        air_density_kg_per_m3=args.air_density_kg_per_m3,
        scale=final_scale,
        mesh_units=args.mesh_units,
        area_method=args.area_method,  # type: ignore[arg-type]
    )

    if not args.no_summary:
        print_summary(obj)

    data = asdict(obj)

    if args.stdout:
        if args.pretty:
            print(yaml.safe_dump(data, sort_keys=False, indent=2))
        else:
            print(yaml.safe_dump(data, sort_keys=False))
    else:
        if args.output is None:
            out_dir = args.deploy_root / "gamepieces"
            output_path = out_dir / f"{args.name}.yaml"
        else:
            output_path = args.output
        write_yaml(obj, output_path, pretty=args.pretty)
        if not args.no_summary:
            print(f"\nWrote YAML to: {output_path}")


if __name__ == "__main__":
    main()
