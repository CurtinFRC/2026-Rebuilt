package org.curtinfrc.frc2026.util.Repulsor.fieldplanner;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.curtinfrc.frc2026.util.Repulsor.fieldplanner.FieldPlanner;
import org.curtinfrc.frc2026.util.Repulsor.fieldplanner.Obstacle;
import org.curtinfrc.frc2026.util.Repulsor.Force;

public class SquareObstacle extends Obstacle {
  public final Translation2d center;
  public final double halfSize;
  public final double maxRange;

  private static final double X_AXIS_ANGLE_BIAS_RAD = Math.toRadians(18.0);

  private static final double CORNER_RANGE_M = 1.0;
  private static final double CORNER_FORCE_SCALE = 22.0;
  private static final double CORNER_FORCE_SOFTEN = 0.08;

  public SquareObstacle(
      Translation2d center, double sizeMeters, double strength, double maxRange) {
    super(strength, true);
    this.center = center;
    this.halfSize = Math.max(0.0, sizeMeters * 0.5);
    this.maxRange = Math.max(0.0, maxRange);
  }

  @Override
  public Force getForceAtPosition(Translation2d position, Translation2d target) {
    double minX = center.getX() - halfSize;
    double maxX = center.getX() + halfSize;
    double minY = center.getY() - halfSize;
    double maxY = center.getY() + halfSize;

    double px = position.getX();
    double py = position.getY();

    double cx = Math.max(minX, Math.min(px, maxX));
    double cy = Math.max(minY, Math.min(py, maxY));

    double dx = px - cx;
    double dy = py - cy;

    double dist = Math.hypot(dx, dy);
    boolean inside = (px >= minX && px <= maxX && py >= minY && py <= maxY);

    double margin = 0.12;
    boolean occludes =
        segmentIntersectsAabb(
            position, target, minX - margin, maxX + margin, minY - margin, maxY + margin);

    double dC1 = Math.hypot(px - minX, py - minY);
    double dC2 = Math.hypot(px - minX, py - maxY);
    double dC3 = Math.hypot(px - maxX, py - minY);
    double dC4 = Math.hypot(px - maxX, py - maxY);
    double dCorner = Math.min(Math.min(dC1, dC2), Math.min(dC3, dC4));
    boolean nearCorner = dCorner <= CORNER_RANGE_M;

    if (!inside && dist > maxRange && !occludes && !nearCorner) return new Force();

    Translation2d awayLocal;
    double effectiveDist;

    if (!inside) {
      if (dist < EPS) return new Force();
      awayLocal = new Translation2d(dx, dy);

      if (Math.abs(dx) > Math.abs(dy)) {
        double sign = Math.signum(target.getY() - position.getY());
        if (sign == 0.0) sign = 1.0;
        Rotation2d biased =
            awayLocal.getAngle().rotateBy(Rotation2d.fromRadians(sign * X_AXIS_ANGLE_BIAS_RAD));
        double norm = awayLocal.getNorm();
        awayLocal = new Translation2d(norm, biased);
      } else if (Math.abs(dy) > Math.abs(dx)) {
        double sign = Math.signum(target.getX() - position.getX());
        if (sign == 0.0) sign = 1.0;
        Rotation2d biased =
            awayLocal.getAngle().rotateBy(Rotation2d.fromRadians(sign * X_AXIS_ANGLE_BIAS_RAD));
        double norm = awayLocal.getNorm();
        awayLocal = new Translation2d(norm, biased);
      }

      effectiveDist = dist;
    } else {
      double dL = px - minX;
      double dR = maxX - px;
      double dB = py - minY;
      double dT = maxY - py;

      double min = dL;
      double nx = -1, ny = 0;

      if (dR < min) {
        min = dR;
        nx = 1;
        ny = 0;
      }
      if (dB < min) {
        min = dB;
        nx = 0;
        ny = -1;
      }
      if (dT < min) {
        min = dT;
        nx = 0;
        ny = 1;
      }

      awayLocal = new Translation2d(nx, ny);
      effectiveDist = -Math.max(EPS, min);
    }

    double primaryMag = distToForceMag(effectiveDist, Math.max(EPS, maxRange));
    Translation2d primary =
        (Math.abs(primaryMag) < EPS || awayLocal.getNorm() < EPS)
            ? Translation2d.kZero
            : new Translation2d(Math.abs(primaryMag), awayLocal.getAngle());

    Translation2d escape = Translation2d.kZero;

    if (occludes) {
      double engageR = Math.max(0.9, maxRange + halfSize + 1.1);
      if (position.getDistance(center) <= engageR) {
        Translation2d outward = position.minus(center);
        if (outward.getNorm() < EPS) outward = new Translation2d(1.0, 0.0);
        Translation2d outwardU = outward.div(Math.max(EPS, outward.getNorm()));

        Translation2d toC = center.minus(position);
        if (toC.getNorm() < EPS) toC = new Translation2d(1.0, 0.0);
        Translation2d tCCW = new Translation2d(-toC.getY(), toC.getX());
        Translation2d tCW = new Translation2d(toC.getY(), -toC.getX());
        double n1 = Math.max(EPS, tCCW.getNorm());
        double n2 = Math.max(EPS, tCW.getNorm());
        tCCW = tCCW.div(n1);
        tCW = tCW.div(n2);

        double pad = Math.max(0.55, Math.min(2.0, maxRange * 0.85));
        double expMinX = minX - pad;
        double expMaxX = maxX + pad;
        double expMinY = minY - pad;
        double expMaxY = maxY + pad;

        Translation2d chosenT =
            chooseTangent(
                position, target, outwardU, tCW, tCCW, expMinX, expMaxX, expMinY, expMaxY);

        double d = Math.max(0.15, position.getDistance(center) - halfSize);
        double followMag = (strength * 9.0) / (0.35 + d * d);
        double pushOutMag = (strength * 2.6) / (0.45 + d * d);

        Translation2d follow = chosenT.times(followMag);
        Translation2d pushOut = outwardU.times(pushOutMag);

        Translation2d g = target.minus(position);
        if (g.getNorm() > EPS) {
          double along = dot(chosenT, g.div(g.getNorm()));
          if (along < 0.10) {
            follow = follow.times(1.35);
          }
        }

        escape = escape.plus(follow).plus(pushOut);
      }
    }

    Translation2d cornerBoost = Translation2d.kZero;
    if (nearCorner) {
      Translation2d outward = position.minus(center);
      double outN = outward.getNorm();
      if (outN < EPS) {
        double cdx = 0.0;
        double cdy = 0.0;
        double best = dC1;
        cdx = minX - center.getX();
        cdy = minY - center.getY();
        if (dC2 < best) {
          best = dC2;
          cdx = minX - center.getX();
          cdy = maxY - center.getY();
        }
        if (dC3 < best) {
          best = dC3;
          cdx = maxX - center.getX();
          cdy = minY - center.getY();
        }
        if (dC4 < best) {
          cdx = maxX - center.getX();
          cdy = maxY - center.getY();
        }
        outward = new Translation2d(cdx, cdy);
        outN = outward.getNorm();
        if (outN < EPS) outward = new Translation2d(1.0, 0.0);
        outN = outward.getNorm();
      }

      Translation2d outwardU = outward.div(Math.max(EPS, outN));
      double w = 1.0 - Math.min(1.0, dCorner / CORNER_RANGE_M);
      double w2 = w * w;
      double mag =
          (strength * CORNER_FORCE_SCALE) * w2 / (CORNER_FORCE_SOFTEN + dCorner * dCorner);
      cornerBoost = outwardU.times(mag);
    }

    Translation2d sum = primary.plus(escape).plus(cornerBoost);

    double sumN = sum.getNorm();
    Translation2d g = target.minus(position);
    double gN = g.getNorm();
    if (gN > EPS) {
      Translation2d gU = g.div(gN);
      double engageR2 = Math.max(1.15, maxRange + halfSize + 1.35);
      if (position.getDistance(center) <= engageR2 || nearCorner) {
        if (sumN > EPS) {
          double align = dot(sum.div(sumN), gU);
          if (align < 0.18) {
            double d = Math.max(0.18, position.getDistance(center) - halfSize);
            double pushThroughMag = (strength * 2.3) / (0.70 + d * d);
            sum = sum.plus(gU.times(pushThroughMag * (0.18 - align)));
            sumN = sum.getNorm();
          }
        } else {
          double d = Math.max(0.18, position.getDistance(center) - halfSize);
          double pushThroughMag = (strength * 2.3) / (0.70 + d * d);
          sum = sum.plus(gU.times(pushThroughMag));
          sumN = sum.getNorm();
        }
      }
    }

    if (sumN < EPS) return new Force();
    if (sum.getX() == 0 && sum.getY() == 0) {
      return new Force(0, 0);
    }
    return new Force(sumN, sum.getAngle());
  }

  private static Translation2d chooseTangent(
      Translation2d pos,
      Translation2d goal,
      Translation2d outwardU,
      Translation2d tCW,
      Translation2d tCCW,
      double minX,
      double maxX,
      double minY,
      double maxY) {

    double stepT = 0.55;
    double stepO = 0.22;

    Translation2d pCW = pos.plus(tCW.times(stepT)).plus(outwardU.times(stepO));
    Translation2d pCCW = pos.plus(tCCW.times(stepT)).plus(outwardU.times(stepO));

    double base = pos.getDistance(goal);

    double sCW = scoreCandidate(pos, goal, pCW, base, minX, maxX, minY, maxY);
    double sCCW = scoreCandidate(pos, goal, pCCW, base, minX, maxX, minY, maxY);

    return (sCW <= sCCW) ? tCW : tCCW;
  }

  private static double scoreCandidate(
      Translation2d pos,
      Translation2d goal,
      Translation2d cand,
      double baseDist,
      double minX,
      double maxX,
      double minY,
      double maxY) {

    double d = cand.getDistance(goal);

    boolean stillOccluding = segmentIntersectsAabb(cand, goal, minX, maxX, minY, maxY);
    double occPenalty = stillOccluding ? 0.65 : 0.0;

    double progressPenalty = (d >= baseDist - 0.01) ? 0.35 : 0.0;

    double wallPenalty = 0.0;
    double edge = 0.20;
    if (cand.getX() < minX - edge
        || cand.getX() > maxX + edge
        || cand.getY() < minY - edge
        || cand.getY() > maxY + edge) {
      wallPenalty += 0.0;
    }

    return d + occPenalty + progressPenalty + wallPenalty;
  }

  @Override
  public boolean intersectsRectangle(Translation2d[] rectCorners) {
    double minX = center.getX() - halfSize;
    double maxX = center.getX() + halfSize;
    double minY = center.getY() - halfSize;
    double maxY = center.getY() + halfSize;

    for (Translation2d c : rectCorners) {
      double x = c.getX(), y = c.getY();
      if (x >= minX && x <= maxX && y >= minY && y <= maxY) return true;
    }

    Translation2d[] square =
        new Translation2d[] {
          new Translation2d(minX, minY),
          new Translation2d(maxX, minY),
          new Translation2d(maxX, maxY),
          new Translation2d(minX, maxY)
        };

    for (Translation2d s : square) if (FieldPlanner.isPointInPolygon(s, rectCorners)) return true;

    for (int i = 0; i < rectCorners.length; i++) {
      Translation2d a = rectCorners[i];
      Translation2d b = rectCorners[(i + 1) % rectCorners.length];
      if (segmentIntersectsAabb(a, b, minX, maxX, minY, maxY)) return true;
    }

    return false;
  }

  private static boolean segmentIntersectsAabb(
      Translation2d a, Translation2d b, double minX, double maxX, double minY, double maxY) {
    double ax = a.getX(), ay = a.getY();
    double bx = b.getX(), by = b.getY();

    double dx = bx - ax;
    double dy = by - ay;

    if (Math.abs(dx) < EPS && (ax < minX || ax > maxX)) return false;
    if (Math.abs(dy) < EPS && (ay < minY || ay > maxY)) return false;

    double[] tt = new double[] {0.0, 1.0};

    if (!clipLiangBarsky(-dx, ax - minX, tt)) return false;
    if (!clipLiangBarsky(dx, maxX - ax, tt)) return false;
    if (!clipLiangBarsky(-dy, ay - minY, tt)) return false;
    if (!clipLiangBarsky(dy, maxY - ay, tt)) return false;

    return tt[0] <= tt[1];
  }

  private static boolean clipLiangBarsky(double p, double q, double[] tt) {
    if (Math.abs(p) < EPS) return q >= 0;
    double r = q / p;
    if (p < 0) {
      if (r > tt[1]) return false;
      if (r > tt[0]) tt[0] = r;
    } else {
      if (r < tt[0]) return false;
      if (r < tt[1]) tt[1] = r;
    }
    return true;
  }
}

