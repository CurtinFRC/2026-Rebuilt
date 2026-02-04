import argparse
import math
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Optional, Literal, Dict, List, Tuple

import numpy as np
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
    surface_area_m2: float
    bounds_extents_m: List[float]
    flight_axis: str
    density_kg_per_m3: Optional[float]
    mass_kg_override: Optional[float]
    area_method: str
    sphericity: Optional[float]
    equivalent_sphere_diameter_m: Optional[float]
    projected_area_resolution: Optional[int]
    projected_area_rel_error: Optional[float]
    volume_method: str
    voxel_volume_pitch_m: Optional[float]
    voxel_volume_rel_error: Optional[float]
    auto_shape_hint: Optional[str]
    auto_drag_coefficient_hint: Optional[float]
    material: Optional[str]
    estimated_terminal_velocity_mps: float
    air_dynamic_viscosity_pa_s: float
    reynolds_number_at_terminal: Optional[float]
    drag_coefficient_at_terminal: Optional[float]
    drag_model: Optional[str]
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


def compute_surface_area(mesh: trimesh.Trimesh) -> float:
    area = float(mesh.area)
    if area <= 0.0:
        raise ValueError("Mesh surface area is non-positive")
    return area


def compute_volume(
    mesh: trimesh.Trimesh,
    rel_tol: float,
    max_iter: int,
) -> Tuple[float, str, Optional[float], Optional[float]]:
    if mesh.is_volume and mesh.volume > 0.0:
        return float(mesh.volume), "mesh", None, None

    repaired = mesh.copy()
    try:
        repaired.remove_degenerate_faces()
        repaired.remove_duplicate_faces()
        repaired.remove_unreferenced_vertices()
        repaired.fill_holes()
        repaired.process(validate=True)
        if repaired.is_volume and repaired.volume > 0.0:
            return float(repaired.volume), "repaired", None, None
    except Exception:
        pass

    bounds = mesh.bounds
    extents = bounds[1] - bounds[0]
    min_extent = float(np.min(extents))
    if min_extent <= 0.0:
        hull_volume = float(mesh.convex_hull.volume)
        return hull_volume, "convex_hull", None, None

    pitch = min_extent / 64.0
    prev_volume = None
    prev_pitch = None
    rel_err = None

    for _ in range(max_iter):
        vg = mesh.voxelized(pitch)
        vf = vg.fill()
        volume = float(vf.volume)
        if prev_volume is not None:
            rel_err = abs(volume - prev_volume) / max(volume, prev_volume, 1e-12)
            if rel_err <= rel_tol:
                return volume, "voxel", pitch, rel_err
        prev_volume = volume
        prev_pitch = pitch
        pitch *= 0.5

    if prev_volume is None:
        hull_volume = float(mesh.convex_hull.volume)
        return hull_volume, "convex_hull", None, None

    return float(prev_volume), "voxel", prev_pitch, rel_err


def compute_sphericity(volume_m3: float, surface_area_m2: float) -> Optional[float]:
    if volume_m3 <= 0.0 or surface_area_m2 <= 0.0:
        return None
    area_sphere = (math.pi ** (1.0 / 3.0)) * ((6.0 * volume_m3) ** (2.0 / 3.0))
    if area_sphere <= 0.0:
        return None
    sphericity = area_sphere / surface_area_m2
    return float(sphericity)


def equivalent_sphere_diameter(volume_m3: float, area_m2: float) -> Optional[float]:
    if volume_m3 > 0.0:
        return float((6.0 * volume_m3 / math.pi) ** (1.0 / 3.0))
    if area_m2 > 0.0:
        return float(math.sqrt(4.0 * area_m2 / math.pi))
    return None


def drag_coefficient_haider_levenspiel(reynolds: float, sphericity: float) -> float:
    re = max(reynolds, 1e-12)
    phi = min(max(sphericity, 1e-3), 1.0)
    a = math.exp(2.3288 - 6.4581 * phi + 2.4486 * phi * phi)
    b = 0.0964 + 0.5565 * phi
    c = math.exp(4.905 - 13.8944 * phi + 18.4222 * phi * phi - 10.2599 * phi ** 3)
    d = math.exp(1.4681 + 12.2584 * phi - 20.7322 * phi * phi + 15.8855 * phi ** 3)
    term1 = (24.0 / re) * (1.0 + a * (re ** b))
    term2 = c / (1.0 + d / re)
    return float(term1 + term2)


def estimate_terminal_velocity_constant(
    mass_kg: float,
    volume_m3: float,
    area_m2: float,
    cd: float,
    air_density: float,
) -> float:
    if mass_kg <= 0.0 or area_m2 <= 0.0 or cd <= 0.0 or air_density <= 0.0:
        return 0.0
    net_weight = (mass_kg - air_density * volume_m3) * 9.80665
    if net_weight <= 0.0:
        return 0.0
    return math.sqrt(2.0 * net_weight / (air_density * area_m2 * cd))


def solve_terminal_velocity(
    mass_kg: float,
    volume_m3: float,
    area_m2: float,
    air_density: float,
    air_dynamic_viscosity: float,
    sphericity: float,
    equiv_diameter_m: float,
) -> Tuple[float, float, float]:
    if (
        mass_kg <= 0.0
        or area_m2 <= 0.0
        or air_density <= 0.0
        or air_dynamic_viscosity <= 0.0
        or equiv_diameter_m <= 0.0
    ):
        return 0.0, 0.0, 0.0
    net_weight = (mass_kg - air_density * volume_m3) * 9.80665
    if net_weight <= 0.0:
        return 0.0, 0.0, 0.0

    def drag_force(speed: float) -> float:
        re = air_density * speed * equiv_diameter_m / air_dynamic_viscosity
        cd = drag_coefficient_haider_levenspiel(re, sphericity)
        return 0.5 * air_density * speed * speed * cd * area_m2

    hi = 0.1
    for _ in range(200):
        if drag_force(hi) >= net_weight:
            break
        hi *= 2.0
        if hi > 2000.0:
            break
    lo = 0.0
    for _ in range(200):
        mid = 0.5 * (lo + hi)
        if drag_force(mid) >= net_weight:
            hi = mid
        else:
            lo = mid
        if abs(hi - lo) / max(hi, 1e-9) < 1e-8:
            break
    vt = hi
    re = air_density * vt * equiv_diameter_m / air_dynamic_viscosity
    cd = drag_coefficient_haider_levenspiel(re, sphericity)
    return float(vt), float(cd), float(re)


def compute_projected_area(
    mesh: trimesh.Trimesh,
    flight_axis: str,
    rel_tol: float,
    base_resolution: int,
    max_resolution: int,
) -> Tuple[float, int, float]:
    bounds = mesh.bounds
    axis_map = {"x": 0, "y": 1, "z": 2}
    axis_idx = axis_map[flight_axis.lower()]
    if axis_idx == 0:
        u_idx, v_idx = 1, 2
    elif axis_idx == 1:
        u_idx, v_idx = 0, 2
    else:
        u_idx, v_idx = 0, 1

    u0, u1 = float(bounds[0, u_idx]), float(bounds[1, u_idx])
    v0, v1 = float(bounds[0, v_idx]), float(bounds[1, v_idx])
    axis_min, axis_max = float(bounds[0, axis_idx]), float(bounds[1, axis_idx])
    if u1 <= u0 or v1 <= v0 or axis_max <= axis_min:
        raise ValueError("Mesh bounds are degenerate, cannot compute projected area")

    margin = (axis_max - axis_min) * 0.1 + 1e-6
    origin_axis = axis_min - margin
    direction = np.array(axis_vector(flight_axis), dtype=np.float64)
    intersector = mesh.ray

    prev_area = None
    prev_res = None
    area = 0.0
    rel_err = 0.0

    res = base_resolution
    while res <= max_resolution:
        du = (u1 - u0) / res
        dv = (v1 - v0) / res
        u_centers = u0 + (np.arange(res, dtype=np.float64) + 0.5) * du
        v_centers = v0 + (np.arange(res, dtype=np.float64) + 0.5) * dv
        total_hits = 0
        max_rays = 200000
        chunk_v = max(1, int(max_rays // max(u_centers.size, 1)))
        for i in range(0, v_centers.size, chunk_v):
            v_chunk = v_centers[i : i + chunk_v]
            uu = np.tile(u_centers, v_chunk.size)
            vv = np.repeat(v_chunk, u_centers.size)
            origins = np.zeros((uu.size, 3), dtype=np.float64)
            origins[:, axis_idx] = origin_axis
            origins[:, u_idx] = uu
            origins[:, v_idx] = vv
            directions = np.tile(direction, (origins.shape[0], 1))
            hits = intersector.intersects_any(origins, directions)
            total_hits += int(hits.sum())
        area = total_hits * du * dv
        if prev_area is not None:
            rel_err = abs(area - prev_area) / max(area, prev_area, 1e-12)
            if rel_err <= rel_tol:
                return float(area), int(res), float(rel_err)
        prev_area = area
        prev_res = res
        res *= 2

    if prev_area is not None:
        rel_err = abs(area - prev_area) / max(area, prev_area, 1e-12)
    return float(area), int(prev_res if prev_res is not None else res), float(rel_err)


def compute_projected_area_analytic(mesh: trimesh.Trimesh, flight_axis: str) -> float:
    m = mesh.convex_hull
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


def compute_mass(
    volume_m3: float,
    density_kg_per_m3: Optional[float],
    mass_override_kg: Optional[float],
) -> float:
    if mass_override_kg is not None:
        if mass_override_kg <= 0.0:
            raise ValueError("mass_kg override must be > 0")
        return float(mass_override_kg)
    if density_kg_per_m3 is None or density_kg_per_m3 <= 0.0:
        raise ValueError("Either mass_kg override or density_kg_per_m3 > 0 must be provided")
    if volume_m3 <= 0.0:
        raise ValueError("Mesh volume is non-positive, check mesh units or integrity")
    return float(density_kg_per_m3) * volume_m3


def compute_cross_section_area_projected(
    mesh: trimesh.Trimesh,
    flight_axis: str,
    rel_tol: float = 1e-4,
    base_resolution: int = 64,
    max_resolution: int = 4096,
) -> float:
    try:
        area, _, _ = compute_projected_area(
            mesh=mesh,
            flight_axis=flight_axis,
            rel_tol=rel_tol,
            base_resolution=base_resolution,
            max_resolution=max_resolution,
        )
        if area <= 0.0:
            raise ValueError("Projected cross-section area computed as <= 0")
        return area
    except Exception:
        return compute_projected_area_analytic(mesh, flight_axis)


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


def guess_shape_and_cd(
    extents: List[float],
    sphericity: Optional[float],
) -> tuple[Optional[str], Optional[float]]:
    if len(extents) != 3:
        return None, None
    ex, ey, ez = sorted(float(e) for e in extents)
    if ex <= 0.0 or ey <= 0.0 or ez <= 0.0:
        return None, None
    r1 = ey / ex
    r2 = ez / ex
    phi = None if sphericity is None else float(sphericity)
    if phi is not None and phi >= 0.9:
        return "sphere-ish", 0.47
    if r2 > 2.0 and r1 < 0.6:
        return "flat_disc-ish", 1.1
    if r2 > 2.0 and 0.6 <= r1 <= 1.4:
        return "cylinder-ish", 0.8
    if phi is not None and phi >= 0.7:
        return "rounded", 0.6
    if phi is not None and phi >= 0.5:
        return "blocky", 0.9
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
    volume_m3: float,
    area_m2: float,
    cd: float,
    air_density: float,
) -> float:
    return estimate_terminal_velocity_constant(
        mass_kg=mass_kg,
        volume_m3=volume_m3,
        area_m2=area_m2,
        cd=cd,
        air_density=air_density,
    )


def build_yaml(
    mesh_path: Path,
    name: str,
    flight_axis: str,
    material: Optional[str],
    density_kg_per_m3: Optional[float],
    mass_kg: Optional[float],
    drag_coefficient: Optional[float],
    air_density_kg_per_m3: float,
    air_dynamic_viscosity_pa_s: float,
    scale: float,
    mesh_units: str,
    area_method: AreaMethod,
    projected_area_rel_tol: float,
    projected_area_base_resolution: int,
    projected_area_max_resolution: int,
    volume_rel_tol: float,
    volume_max_iter: int,
) -> GamePieceYaml:
    mesh = load_mesh_any(mesh_path, scale)
    volume_m3, volume_method, voxel_pitch, voxel_rel = compute_volume(
        mesh=mesh,
        rel_tol=volume_rel_tol,
        max_iter=volume_max_iter,
    )
    surface_area_m2 = compute_surface_area(mesh)

    eff_density = density_kg_per_m3
    if eff_density is None and material and material in MATERIAL_DENSITY_KG_PER_M3:
        eff_density = MATERIAL_DENSITY_KG_PER_M3[material]

    mass_value = compute_mass(volume_m3, eff_density, mass_kg)

    alt_areas: Dict[str, float] = {}
    warnings: List[str] = []

    if not mesh.is_watertight:
        warnings.append("Mesh is not watertight; volume and projected area may be inaccurate")

    projected_resolution = None
    projected_rel_error = None
    try:
        projected_area, projected_resolution, projected_rel_error = compute_projected_area(
            mesh=mesh,
            flight_axis=flight_axis,
            rel_tol=projected_area_rel_tol,
            base_resolution=projected_area_base_resolution,
            max_resolution=projected_area_max_resolution,
        )
        alt_areas["projected"] = projected_area
    except Exception as e:
        try:
            alt_areas["projected"] = compute_projected_area_analytic(mesh, flight_axis)
            warnings.append(f"Projected area fallback used after ray failure: {e!s}")
        except Exception as e2:
            warnings.append(f"Area estimate failed for projected: {e!s}; fallback failed: {e2!s}")

    for method in ("bbox_ellipse", "bbox_rect"):
        try:
            alt_areas[method] = compute_area(mesh, method, flight_axis)
        except Exception as e:
            warnings.append(f"Area estimate failed for {method}: {e!s}")

    if projected_rel_error is not None and projected_rel_error > projected_area_rel_tol:
        warnings.append(
            f"Projected area did not converge to rel_tol ({projected_rel_error:.2e} > {projected_area_rel_tol:.2e})"
        )
    if volume_method == "voxel" and voxel_rel is not None and voxel_rel > volume_rel_tol:
        warnings.append(
            f"Voxel volume did not converge to rel_tol ({voxel_rel:.2e} > {volume_rel_tol:.2e})"
        )

    if area_method not in alt_areas:
        raise ValueError(f"Requested area_method '{area_method}' did not produce a value")

    area_value = alt_areas[area_method]
    extents = [float(x) for x in mesh.extents]
    sphericity = compute_sphericity(volume_m3, surface_area_m2)
    eq_diameter = equivalent_sphere_diameter(volume_m3, area_value)

    shape_hint, cd_hint = guess_shape_and_cd(extents, sphericity)

    model_cd = None
    model_vt = None
    model_re = None
    drag_model = None

    if sphericity is not None and eq_diameter is not None:
        model_vt, model_cd, model_re = solve_terminal_velocity(
            mass_kg=mass_value,
            volume_m3=volume_m3,
            area_m2=area_value,
            air_density=air_density_kg_per_m3,
            air_dynamic_viscosity=air_dynamic_viscosity_pa_s,
            sphericity=sphericity,
            equiv_diameter_m=eq_diameter,
        )
        if model_cd > 0.0:
            drag_model = "haider_levenspiel"

    auto_cd_hint = model_cd if model_cd and model_cd > 0.0 else cd_hint

    if drag_coefficient is None:
        if model_cd is not None and model_cd > 0.0:
            drag_value = model_cd
            vt = model_vt if model_vt is not None else 0.0
            re_at_vt = model_re
            drag_model_used = drag_model
        else:
            drag_value = cd_hint if cd_hint is not None else 0.75
            vt = estimate_terminal_velocity(
                mass_kg=mass_value,
                volume_m3=volume_m3,
                area_m2=area_value,
                cd=drag_value,
                air_density=air_density_kg_per_m3,
            )
            re_at_vt = None
            drag_model_used = "heuristic"
    else:
        drag_value = drag_coefficient
        vt = estimate_terminal_velocity(
            mass_kg=mass_value,
            volume_m3=volume_m3,
            area_m2=area_value,
            cd=drag_value,
            air_density=air_density_kg_per_m3,
        )
        if eq_diameter is not None and vt > 0.0 and air_dynamic_viscosity_pa_s > 0.0:
            re_at_vt = air_density_kg_per_m3 * vt * eq_diameter / air_dynamic_viscosity_pa_s
        else:
            re_at_vt = None
        drag_model_used = "override"

    if vt < 3.0:
        warnings.append(f"Very low estimated terminal velocity ({vt:.2f} m/s) - check mass/area/cd")
    if vt > 80.0:
        warnings.append(f"Very high estimated terminal velocity ({vt:.2f} m/s) - check mass/area/cd")

    meta = GamePieceMetadata(
        source_mesh=mesh_path.name,
        mesh_units=mesh_units,
        scale=float(scale),
        volume_m3=volume_m3,
        surface_area_m2=surface_area_m2,
        bounds_extents_m=extents,
        flight_axis=flight_axis,
        density_kg_per_m3=None if eff_density is None else float(eff_density),
        mass_kg_override=None if mass_kg is None else float(mass_kg),
        area_method=area_method,
        sphericity=None if sphericity is None else float(sphericity),
        equivalent_sphere_diameter_m=None if eq_diameter is None else float(eq_diameter),
        projected_area_resolution=None if projected_resolution is None else int(projected_resolution),
        projected_area_rel_error=None if projected_rel_error is None else float(projected_rel_error),
        volume_method=volume_method,
        voxel_volume_pitch_m=None if voxel_pitch is None else float(voxel_pitch),
        voxel_volume_rel_error=None if voxel_rel is None else float(voxel_rel),
        auto_shape_hint=shape_hint,
        auto_drag_coefficient_hint=None if auto_cd_hint is None else float(auto_cd_hint),
        material=material,
        estimated_terminal_velocity_mps=vt,
        air_dynamic_viscosity_pa_s=float(air_dynamic_viscosity_pa_s),
        reynolds_number_at_terminal=None if re_at_vt is None else float(re_at_vt),
        drag_coefficient_at_terminal=None if drag_value is None else float(drag_value),
        drag_model=drag_model_used,
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
    print(f"    surface_area_m2              = {m.surface_area_m2:.6f}")
    print(f"    bounds_extents_m             = {m.bounds_extents_m}")
    print(f"    flight_axis                  = {m.flight_axis}")
    print(f"    material                     = {m.material}")
    print(f"    density_kg_per_m3            = {m.density_kg_per_m3}")
    print(f"    mass_kg_override             = {m.mass_kg_override}")
    print(f"    area_method                  = {m.area_method}")
    print(f"    sphericity                   = {m.sphericity}")
    print(f"    equivalent_sphere_diameter_m = {m.equivalent_sphere_diameter_m}")
    print(f"    projected_area_resolution    = {m.projected_area_resolution}")
    print(f"    projected_area_rel_error     = {m.projected_area_rel_error}")
    print(f"    volume_method                = {m.volume_method}")
    print(f"    voxel_volume_pitch_m         = {m.voxel_volume_pitch_m}")
    print(f"    voxel_volume_rel_error       = {m.voxel_volume_rel_error}")
    print(f"    auto_shape_hint              = {m.auto_shape_hint}")
    print(f"    auto_drag_coefficient_hint   = {m.auto_drag_coefficient_hint}")
    print(f"    air_dynamic_viscosity_pa_s   = {m.air_dynamic_viscosity_pa_s}")
    print(f"    reynolds_number_at_terminal  = {m.reynolds_number_at_terminal}")
    print(f"    drag_coefficient_at_terminal = {m.drag_coefficient_at_terminal}")
    print(f"    drag_model                   = {m.drag_model}")
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
        "--air-dynamic-viscosity-pa-s",
        type=float,
        default=1.81e-5,
        help="Air dynamic viscosity in Pa*s",
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
        "--projected-area-rel-tol",
        type=float,
        default=1e-4,
        help="Relative tolerance for projected area convergence",
    )
    parser.add_argument(
        "--projected-area-base-resolution",
        type=int,
        default=64,
        help="Base grid resolution for projected area sampling",
    )
    parser.add_argument(
        "--projected-area-max-resolution",
        type=int,
        default=4096,
        help="Maximum grid resolution for projected area sampling",
    )
    parser.add_argument(
        "--volume-rel-tol",
        type=float,
        default=1e-4,
        help="Relative tolerance for voxel volume convergence",
    )
    parser.add_argument(
        "--volume-max-iter",
        type=int,
        default=8,
        help="Maximum iterations for voxel volume convergence",
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
        air_dynamic_viscosity_pa_s=args.air_dynamic_viscosity_pa_s,
        scale=final_scale,
        mesh_units=args.mesh_units,
        area_method=args.area_method,
        projected_area_rel_tol=args.projected_area_rel_tol,
        projected_area_base_resolution=args.projected_area_base_resolution,
        projected_area_max_resolution=args.projected_area_max_resolution,
        volume_rel_tol=args.volume_rel_tol,
        volume_max_iter=args.volume_max_iter,
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
