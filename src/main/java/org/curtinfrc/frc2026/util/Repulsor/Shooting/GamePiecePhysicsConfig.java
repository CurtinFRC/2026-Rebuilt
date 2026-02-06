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

public final class GamePiecePhysicsConfig {
  public String name;
  public double mass_kg;
  public double drag_coefficient;
  public double cross_section_area_m2;
  public Double air_density_kg_per_m3;
  public Metadata metadata;

  public static final class Metadata {
    public String source_mesh;
    public String mesh_units;
    public double scale;
    public double volume_m3;
    public List<Double> bounds_extents_m;
    public String flight_axis;
    public double density_kg_per_m3;
    public Double mass_kg_override;
    public String area_method;
    public String auto_shape_hint;
    public double auto_drag_coefficient_hint;
    public String material;
    public double estimated_terminal_velocity_mps;
    public AltCrossSectionEstimates alt_cross_section_estimates_m2;
    public List<String> warnings;
  }

  public static final class AltCrossSectionEstimates {
    public double projected;
    public double bbox_ellipse;
    public double bbox_rect;
  }
}
