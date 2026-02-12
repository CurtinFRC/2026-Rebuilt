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

package org.curtinfrc.frc2026.util.Repulsor.ExtraPathingHelpers;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.List;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.FieldPlanner;

final class ExtraPathingMath {
  private ExtraPathingMath() {}

  static Translation2d trimEnd(Translation2d a, Translation2d b, double trim) {
    Translation2d d = b.minus(a);
    double L = d.getNorm();
    if (L < 1e-9) return b;
    double keep = Math.max(0.0, L - trim);
    Translation2d u = new Translation2d(d.getX() / L, d.getY() / L);
    return a.plus(u.times(keep));
  }

  record DistParam(double dist, double t) {}

  static DistParam pointToSegDistParam(Translation2d p, Translation2d a, Translation2d b) {
    Translation2d ap = p.minus(a);
    Translation2d ab = b.minus(a);
    double ab2 = ab.getNorm() * ab.getNorm();
    if (ab2 < 1e-12) return new DistParam(ap.getNorm(), 0.0);
    double t = Math.max(0.0, Math.min(1.0, FieldPlanner.dot(ap, ab) / ab2));
    Translation2d proj = a.plus(ab.times(t));
    return new DistParam(p.getDistance(proj), t);
  }

  static double paramForYOnSegment(double y, Translation2d a, Translation2d b) {
    double dy = b.getY() - a.getY();
    if (Math.abs(dy) < 1e-12) return Double.NaN;
    return (y - a.getY()) / dy;
  }

  static double paramForXOnSegment(double x, Translation2d a, Translation2d b) {
    double dx = b.getX() - a.getX();
    if (Math.abs(dx) < 1e-12) return Double.NaN;
    return (x - a.getX()) / dx;
  }

  static List<Translation2d> offsetAround(
      Translation2d c, double R, Translation2d start, Translation2d goal) {
    List<Translation2d> out = new ArrayList<>(2);
    Translation2d v = goal.minus(start);
    double vn = v.getNorm();
    if (vn > 1e-9) {
      Translation2d u = new Translation2d(v.getX() / vn, v.getY() / vn);
      Translation2d n = new Translation2d(-u.getY(), u.getX());
      out.add(c.plus(n.times(R)));
      out.add(c.minus(n.times(R)));
    }
    return out;
  }

  static double estimateCurvature(Translation2d a, Translation2d b, Translation2d c) {
    double x1 = a.getX(), y1 = a.getY();
    double x2 = b.getX(), y2 = b.getY();
    double x3 = c.getX(), y3 = c.getY();
    double d = 2 * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2));
    if (Math.abs(d) < 1e-9) return 0.0;
    double ux =
        ((x1 * x1 + y1 * y1) * (y2 - y3)
                + (x2 * x2 + y2 * y2) * (y3 - y1)
                + (x3 * x3 + y3 * y3) * (y1 - y2))
            / d;
    double uy =
        ((x1 * x1 + y1 * y1) * (x3 - x2)
                + (x2 * x2 + y2 * y2) * (x1 - x3)
                + (x3 * x3 + y3 * y3) * (x2 - x1))
            / d;
    double R = Math.hypot(b.getX() - ux, b.getY() - uy);
    return (R < 1e-6) ? 0.0 : 1.0 / R;
  }
}
