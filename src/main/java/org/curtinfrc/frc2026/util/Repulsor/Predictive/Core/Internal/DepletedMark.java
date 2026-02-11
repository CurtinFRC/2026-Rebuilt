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
package org.curtinfrc.frc2026.util.Repulsor.Predictive.Core.Internal;

import edu.wpi.first.math.geometry.Translation2d;

public final class DepletedMark {
  public final Translation2d p;
  public double t;
  public double s;
  public double r;
  public double ttl;
  public final boolean ring;
  public final double ringR0;
  public final double ringR1;

  public DepletedMark(Translation2d p, double t, double s, double r, double ttl) {
    this.p = p != null ? p : new Translation2d();
    this.t = t;
    this.s = Math.max(0.0, s);
    this.r = Math.max(0.05, r);
    this.ttl = Math.max(0.1, ttl);
    this.ring = false;
    this.ringR0 = 0.0;
    this.ringR1 = 0.0;
  }

  public DepletedMark(Translation2d p, double t, double s, double r0, double r1, double ttl) {
    this.p = p != null ? p : new Translation2d();
    this.t = t;
    this.s = Math.max(0.0, s);
    this.r = Math.max(0.05, 0.5 * (Math.max(r0, r1) - Math.min(r0, r1)));
    this.ttl = Math.max(0.1, ttl);
    this.ring = true;
    this.ringR0 = Math.max(0.05, Math.min(r0, r1));
    this.ringR1 = Math.max(this.ringR0 + 0.05, Math.max(r0, r1));
  }
}
