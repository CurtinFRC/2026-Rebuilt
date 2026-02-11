/*
 * Copyright (C) 2026 Paul Hodges
 *
 * This file is part of Repulsor.
 *
 * Repulsor is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Repulsor is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Repulsor. If not, see https://www.gnu.org/licenses/.
 */

package org.curtinfrc.frc2026.util.Repulsor.Shooting;

import java.util.List;
import java.util.Map;

public final class GamePiecePhysicsConfig {
  public String name;
  public double mass_kg;
  public double drag_coefficient;
  public double cross_section_area_m2;
  public double air_density_kg_per_m3;
  public Metadata metadata;

  public static final class Metadata {
    public String source_mesh;
    public String mesh_units;
    public double scale;

    public double volume_m3;
    public double surface_area_m2;
    public List<Double> bounds_extents_m;

    public String flight_axis;

    public Double density_kg_per_m3;
    public Double mass_kg_override;

    public String area_method;

    public Double sphericity;
    public Double equivalent_sphere_diameter_m;

    public Integer projected_area_resolution;
    public Double projected_area_rel_error;

    public String volume_method;
    public Double voxel_volume_pitch_m;
    public Double voxel_volume_rel_error;

    public String auto_shape_hint;
    public Double auto_drag_coefficient_hint;

    public String material;

    public double estimated_terminal_velocity_mps;

    public double air_dynamic_viscosity_pa_s;
    public Double reynolds_number_at_terminal;

    public Double drag_coefficient_at_terminal;
    public String drag_model;

    public Map<String, Double> alt_cross_section_estimates_m2;
    public List<String> warnings;
  }
}
