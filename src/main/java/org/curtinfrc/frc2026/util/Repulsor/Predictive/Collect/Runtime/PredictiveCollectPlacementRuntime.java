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
package org.curtinfrc.frc2026.util.Repulsor.Predictive.Collect.Runtime;

import edu.wpi.first.math.geometry.Translation2d;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.PredictiveFieldStateOps;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.SpatialDyn;

public final class PredictiveCollectPlacementRuntime {
  private PredictiveCollectPlacementRuntime() {}

  public static double clamp(double x, double lo, double hi) {
    return Math.max(lo, Math.min(hi, x));
  }

  public static double coreRadiusFor(double cellM) {
    double r = 0.55 * Math.max(0.10, cellM);
    return clamp(
        r, PredictiveFieldStateOps.COLLECT_CORE_R_MIN, PredictiveFieldStateOps.COLLECT_CORE_R_MAX);
  }

  public static double snapRadiusFor(double cellM) {
    double r = 4.5 * coreRadiusFor(cellM);
    return clamp(
        r, PredictiveFieldStateOps.COLLECT_SNAP_R_MIN, PredictiveFieldStateOps.COLLECT_SNAP_R_MAX);
  }

  public static double microCentroidRadiusFor(double cellM) {
    double r = 2.6 * coreRadiusFor(cellM);
    return clamp(
        r,
        PredictiveFieldStateOps.COLLECT_MICRO_CENTROID_R_MIN,
        PredictiveFieldStateOps.COLLECT_MICRO_CENTROID_R_MAX);
  }

  public static double jitterRadiusFor(double cellM) {
    double r = 1.15 * coreRadiusFor(cellM);
    return clamp(
        r,
        PredictiveFieldStateOps.COLLECT_JITTER_R_MIN,
        PredictiveFieldStateOps.COLLECT_JITTER_R_MAX);
  }

  public static Translation2d snapToNearestThenMicroCentroid(
      PredictiveFieldStateOps ops,
      Translation2d p,
      SpatialDyn dyn,
      double rSnap,
      double rCentroid,
      double minMass) {
    if (p == null || dyn == null) return null;

    Translation2d nearest = dyn.nearestResourceTo(p, Math.max(0.01, rSnap));
    if (nearest == null) return null;

    Translation2d q = nearest;

    Translation2d c1 =
        dyn.centroidResourcesWithin(q, Math.max(0.05, rCentroid), Math.max(0.0, minMass));
    if (c1 != null) q = c1;

    Translation2d c2 =
        dyn.centroidResourcesWithin(
            q, Math.max(0.05, 0.60 * rCentroid), Math.max(0.0, minMass * 0.65));
    if (c2 != null) q = c2;

    return q;
  }

  public static Translation2d enforceHardStopOnFuel(
      PredictiveFieldStateOps ops,
      SpatialDyn dyn,
      Translation2d p,
      double rCore,
      double rSnap,
      double rCentroid,
      double minMass) {
    if (p == null || dyn == null) return null;

    Translation2d nearestCore = dyn.nearestResourceTo(p, Math.max(0.01, rCore));
    if (nearestCore != null) {
      Translation2d q = nearestCore;
      Translation2d c =
          dyn.centroidResourcesWithin(
              q, Math.max(0.05, Math.min(rCentroid, 0.22)), Math.max(0.0, minMass));
      if (c != null) q = c;
      if (dyn.nearestResourceTo(q, Math.max(0.01, rCore)) != null) return q;
      return nearestCore;
    }

    Translation2d snapped = snapToNearestThenMicroCentroid(ops, p, dyn, rSnap, rCentroid, minMass);
    if (snapped == null) return null;

    Translation2d nearest2 = dyn.nearestResourceTo(snapped, Math.max(0.01, rCore));
    if (nearest2 == null) return null;

    Translation2d q = nearest2;
    Translation2d c =
        dyn.centroidResourcesWithin(
            q, Math.max(0.05, Math.min(rCentroid, 0.22)), Math.max(0.0, minMass));
    if (c != null) q = c;

    return dyn.nearestResourceTo(q, Math.max(0.01, rCore)) != null ? q : nearest2;
  }

  public static double pickupRobustPenalty(
      PredictiveFieldStateOps ops, SpatialDyn dyn, Translation2d p, double rCore, double jitterR) {
    if (dyn == null || p == null) return PredictiveFieldStateOps.COLLECT_NOFUEL_PENALTY;

    int c0 = dyn.countResourcesWithin(p, Math.max(0.01, rCore));
    Translation2d nn0 = dyn.nearestResourceTo(p, Math.max(0.01, rCore));
    double d0 = nn0 != null ? nn0.getDistance(p) : Double.POSITIVE_INFINITY;

    double jr = Math.max(0.0, jitterR);
    if (jr <= 1e-6)
      return (c0 >= 1 || d0 <= rCore) ? 0.0 : PredictiveFieldStateOps.COLLECT_NOFUEL_PENALTY;

    Translation2d[] samp =
        new Translation2d[] {
          p,
          new Translation2d(p.getX() + jr, p.getY()),
          new Translation2d(p.getX() - jr, p.getY()),
          new Translation2d(p.getX(), p.getY() + jr),
          new Translation2d(p.getX(), p.getY() - jr),
          new Translation2d(p.getX() + 0.7071 * jr, p.getY() + 0.7071 * jr),
          new Translation2d(p.getX() + 0.7071 * jr, p.getY() - 0.7071 * jr),
          new Translation2d(p.getX() - 0.7071 * jr, p.getY() + 0.7071 * jr),
          new Translation2d(p.getX() - 0.7071 * jr, p.getY() - 0.7071 * jr)
        };

    int minC = Integer.MAX_VALUE;
    int maxC = 0;

    for (Translation2d s : samp) {
      int c = dyn.countResourcesWithin(s, Math.max(0.01, rCore));
      minC = Math.min(minC, c);
      maxC = Math.max(maxC, c);
    }

    boolean centerOk = (c0 >= 1) || (d0 <= rCore);

    if (centerOk) {
      if (minC >= 1) return 0.0;
      return PredictiveFieldStateOps.COLLECT_EDGE_PENALTY;
    }

    if (maxC >= 1) return PredictiveFieldStateOps.COLLECT_HOLE_PENALTY;
    return PredictiveFieldStateOps.COLLECT_NOFUEL_PENALTY;
  }
}
