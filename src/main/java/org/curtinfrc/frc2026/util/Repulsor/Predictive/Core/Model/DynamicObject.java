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

package org.curtinfrc.frc2026.util.Repulsor.Predictive.Core.Model;

import edu.wpi.first.math.geometry.Translation2d;

public class DynamicObject {
  public final String id;
  public final String type;
  public final Translation2d pos;
  public final Translation2d vel;
  public final double ageS;

  public DynamicObject(String id, String type, Translation2d pos, Translation2d vel, double ageS) {
    this.id = id;
    this.type = type != null ? type : "unknown";
    this.pos = pos != null ? pos : new Translation2d();
    this.vel = vel != null ? vel : new Translation2d();
    this.ageS = Math.max(0.0, ageS);
  }
}
