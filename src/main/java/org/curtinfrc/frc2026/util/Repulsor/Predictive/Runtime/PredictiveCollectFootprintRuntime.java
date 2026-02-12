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
package org.curtinfrc.frc2026.util.Repulsor.Predictive.Runtime;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Internal.FootprintEval;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Internal.HeadingPick;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.PredictiveFieldStateOps;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.SpatialDyn;

public final class PredictiveCollectFootprintRuntime {
  private PredictiveCollectFootprintRuntime() {}

  public static Rotation2d face(Translation2d from, Translation2d to, Rotation2d fallback) {
    Translation2d d = to.minus(from);
    if (d.getNorm() < 1e-9
        || (d.getMeasureX().baseUnitMagnitude() == 0 && d.getMeasureY().baseUnitMagnitude() == 0)) {
      return fallback;
    }
    return d.getAngle();
  }

  public static Translation2d resolveCollectTouch(
      PredictiveFieldStateOps ops,
      SpatialDyn dyn,
      Translation2d center,
      Rotation2d heading,
      double rCore,
      double rSnap,
      double rCentroid) {
    if (center == null) return null;
    Translation2d front =
        center.plus(
            PredictiveFieldStateOps.COLLECT_INTAKE
                .get()
                .supportPointRobotFrame(new Translation2d(1.0, 0.0))
                .rotateBy(heading));
    return ops.enforceHardStopOnFuel(dyn, front, rCore, rSnap, rCentroid, 0.10);
  }

  public static FootprintEval evalFootprint(
      PredictiveFieldStateOps ops,
      SpatialDyn dyn,
      Translation2d center,
      Rotation2d heading,
      double rCore) {
    FootprintEval e = new FootprintEval();
    if (dyn == null || center == null) return e;
    for (Translation2d sample : PredictiveFieldStateOps.COLLECT_FOOTPRINT_SAMPLES) {
      Translation2d field = center.plus(sample.rotateBy(heading));
      int c = dyn.countResourcesWithin(field, Math.max(0.04, rCore));
      double u = dyn.valueInSquare(field, Math.max(0.08, rCore));
      if (c > e.maxCount) e.maxCount = c;
      e.sumUnits += u;
    }
    return e;
  }

  public static void fillFootprintEvidence(
      PredictiveFieldStateOps ops,
      SpatialDyn dyn,
      Translation2d center,
      Rotation2d heading,
      FootprintEval e) {
    if (e == null || e.hasEvidence || dyn == null || center == null) return;
    double sumEv = 0.0;
    int n = 0;
    for (Translation2d sample : PredictiveFieldStateOps.COLLECT_FOOTPRINT_SAMPLES) {
      Translation2d field = center.plus(sample.rotateBy(heading));
      double ev = dyn.evidenceMassWithin(field, PredictiveFieldStateOps.EVIDENCE_R);
      sumEv += ev;
      n++;
    }
    e.avgEvidence = (n > 0) ? (sumEv / n) : 0.0;
    e.hasEvidence = true;
  }

  public static boolean footprintOk(
      PredictiveFieldStateOps ops,
      SpatialDyn dyn,
      Translation2d center,
      Rotation2d heading,
      double rCore,
      double minUnits) {
    FootprintEval fp = evalFootprint(ops, dyn, center, heading, rCore);
    if (fp.maxCount < 1 || fp.sumUnits < minUnits) return false;
    fillFootprintEvidence(ops, dyn, center, heading, fp);
    return fp.avgEvidence >= minUnits * 0.85;
  }

  public static HeadingPick bestHeadingForFootprint(
      PredictiveFieldStateOps ops,
      SpatialDyn dyn,
      Translation2d desiredCenter,
      Translation2d fuelTouch,
      Rotation2d baseHeading,
      double rCore,
      double minUnits) {
    if (dyn == null || desiredCenter == null || fuelTouch == null) return null;
    double[] deg = new double[] {-60, -40, -20, 0, 20, 40, 60};
    HeadingPick best = null;
    for (double d : deg) {
      Rotation2d h = baseHeading.plus(Rotation2d.fromDegrees(d));
      Translation2d p =
          PredictiveFieldStateOps.COLLECT_INTAKE
              .get()
              .snapCenterSoFootprintTouchesPoint(desiredCenter, h, fuelTouch);
      FootprintEval fp = evalFootprint(ops, dyn, p, h, rCore);
      if (fp.maxCount < 1 || fp.sumUnits < minUnits) continue;
      if (best == null
          || fp.maxCount > best.eval.maxCount
          || (fp.maxCount == best.eval.maxCount && fp.sumUnits > best.eval.sumUnits)
          || (fp.maxCount == best.eval.maxCount
              && Math.abs(fp.sumUnits - best.eval.sumUnits) <= 1e-6
              && compareFootprintEvidence(ops, dyn, p, h, fp, best))) {
        best = new HeadingPick(p, h, fp);
      }
    }
    if (best != null) {
      fillFootprintEvidence(ops, dyn, best.center, best.heading, best.eval);
    }
    return best;
  }

  public static boolean compareFootprintEvidence(
      PredictiveFieldStateOps ops,
      SpatialDyn dyn,
      Translation2d p,
      Rotation2d h,
      FootprintEval fp,
      HeadingPick best) {
    if (best == null) return true;
    fillFootprintEvidence(ops, dyn, p, h, fp);
    fillFootprintEvidence(ops, dyn, best.center, best.heading, best.eval);
    return fp.avgEvidence > best.eval.avgEvidence;
  }

  public static long footprintKey(Translation2d center, double cellM) {
    long kx = Math.round(center.getX() * 1000.0);
    long ky = Math.round(center.getY() * 1000.0);
    long kc = Math.round(cellM * 1000.0);
    long key = kx * 0x9E3779B97F4A7C15L;
    key ^= Long.rotateLeft(ky * 0xC2B2AE3D27D4EB4FL, 1);
    key ^= Long.rotateLeft(kc * 0x165667B19E3779F9L, 2);
    return key;
  }
}
