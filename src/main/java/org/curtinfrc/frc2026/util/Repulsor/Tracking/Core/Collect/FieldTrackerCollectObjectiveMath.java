package org.curtinfrc.frc2026.util.Repulsor.Tracking.Core.Collect;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import org.curtinfrc.frc2026.util.Repulsor.Constants;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Core.Internal.NearestPoint;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Core.Internal._SpatialIndex;

final class FieldTrackerCollectObjectiveMath {
  private static final double STICKY_SAME_M = 0.45;
  private static final double SWITCH_APPROACH_COS_MIN = Math.cos(Math.toRadians(78.0));
  private static final double STICKY_NEAR_DIST_M = 1.40;
  private static final double STICKY_FAR_DIST_M = 4.80;
  private static final double CANONICAL_SNAP_M = 0.45;

  private FieldTrackerCollectObjectiveMath() {}

  static boolean stickySame(Translation2d a, Translation2d b) {
    return samePoint(a, b, STICKY_SAME_M);
  }

  static boolean stickySwitched(Translation2d prev, Translation2d next) {
    if (prev == null && next != null) return true;
    if (prev == null || next == null) return false;
    return !stickySame(prev, next);
  }

  static Translation2d canonicalizeCollectPoint(Translation2d p, Translation2d[] setpoints) {
    if (p == null || setpoints == null || setpoints.length == 0) return p;

    Translation2d best = null;
    double bestD2 = Double.POSITIVE_INFINITY;

    for (Translation2d s : setpoints) {
      if (s == null) continue;
      double dx = p.getX() - s.getX();
      double dy = p.getY() - s.getY();
      double d2 = dx * dx + dy * dy;
      if (d2 < bestD2) {
        bestD2 = d2;
        best = s;
      }
    }

    if (best == null) return p;
    double snap2 = CANONICAL_SNAP_M * CANONICAL_SNAP_M;
    return bestD2 <= snap2 ? best : p;
  }

  static int sideSignX(double x) {
    return sideSignXBand(x, 0.10);
  }

  static int sideSignXBand(double x, double band) {
    double mid = Constants.FIELD_LENGTH * 0.5;
    double b = Math.max(0.0, band);
    if (x < mid - b) return -1;
    if (x > mid + b) return 1;
    return 0;
  }

  static int sideSign(Translation2d p) {
    if (p == null) return 0;
    return sideSignX(p.getX());
  }

  static boolean samePoint(Translation2d a, Translation2d b, double tol) {
    if (a == null || b == null) return false;
    return a.getDistance(b) <= tol;
  }

  static double dot(Translation2d a, Translation2d b) {
    return a.getX() * b.getX() + a.getY() * b.getY();
  }

  static Translation2d unit(Translation2d v) {
    double n = v.getNorm();
    if (n <= 1e-9) return new Translation2d(1.0, 0.0);
    return v.div(n);
  }

  static boolean movedEnough(Translation2d a, Translation2d b, double minMove) {
    if (a == null || b == null) return true;
    return a.getDistance(b) >= minMove;
  }

  static boolean violatesApproach(Translation2d approachHat, Translation2d toCandHat) {
    if (approachHat == null || toCandHat == null) return false;
    return dot(approachHat, toCandHat) < SWITCH_APPROACH_COS_MIN;
  }

  static Translation2d unitOrDefault(Translation2d v, Translation2d def) {
    double n = v.getNorm();
    if (n <= 1e-9) return def;
    return v.div(n);
  }

  static NearestPoint nearestPointTo(Translation2d q, Translation2d[] pts) {
    if (q == null || pts == null || pts.length == 0)
      return new NearestPoint(null, Double.POSITIVE_INFINITY);
    Translation2d best = null;
    double bestD = Double.POSITIVE_INFINITY;
    for (Translation2d p : pts) {
      if (p == null) continue;
      double d = q.getDistance(p);
      if (d < bestD) {
        bestD = d;
        best = p;
      }
    }
    return new NearestPoint(best, bestD);
  }

  static double dist2(Translation2d a, Translation2d b) {
    double dx = a.getX() - b.getX();
    double dy = a.getY() - b.getY();
    return dx * dx + dy * dy;
  }

  static double centerPenalty(Translation2d p) {
    double dx = p.getX() - (Constants.FIELD_LENGTH * 0.5);
    double dy = p.getY() - (Constants.FIELD_WIDTH * 0.5);
    return Math.sqrt(dx * dx + dy * dy);
  }

  static _SpatialIndex buildIndex(Translation2d[] pts, double cellM) {
    _SpatialIndex idx = new _SpatialIndex(cellM);
    for (Translation2d p : pts) {
      if (p != null) idx.add(p);
    }
    return idx;
  }

  static Translation2d perp(Translation2d v) {
    return new Translation2d(-v.getY(), v.getX());
  }

  static double nowSFromNs(long ns) {
    return ns / 1e9;
  }

  static double holdSForDist(double d) {
    double t = clamp01((d - STICKY_NEAR_DIST_M) / (STICKY_FAR_DIST_M - STICKY_NEAR_DIST_M));
    double min = lerp(0.70, 0.25, t);
    double max = lerp(3.80, 1.80, t);
    return Math.max(min, Math.min(max, max));
  }

  static double switchMarginForDist(double d) {
    double t = clamp01((d - STICKY_NEAR_DIST_M) / (STICKY_FAR_DIST_M - STICKY_NEAR_DIST_M));
    return lerp(1.45, 0.50, t);
  }

  static double lerp(double a, double b, double t) {
    return a + (b - a) * t;
  }

  static double clamp01(double x) {
    if (x < 0.0) return 0.0;
    if (x > 1.0) return 1.0;
    return x;
  }
}
