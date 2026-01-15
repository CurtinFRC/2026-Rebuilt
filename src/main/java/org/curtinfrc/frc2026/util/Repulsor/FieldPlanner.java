// File: src/main/java/org/curtinfrc/frc2025/util/Repulsor/FieldPlanner.java
package org.curtinfrc.frc2026.util.Repulsor;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Optional;
import java.util.function.Function;
import org.curtinfrc.frc2026.util.Repulsor.Fallback.PlannerFallback;
import org.curtinfrc.frc2026.util.Repulsor.FieldTracker.GameElement.Alliance;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldMapBuilder.CategorySpec;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.RepulsorSetpoint;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.SetpointContext;
import org.curtinfrc.frc2026.util.Repulsor.Tuning.DefaultDriveTuning;
import org.curtinfrc.frc2026.util.Repulsor.Tuning.DefaultTurnTuning;
import org.curtinfrc.frc2026.util.Repulsor.Tuning.DriveTuning;
import org.curtinfrc.frc2026.util.Repulsor.Tuning.TurnTuning;
import org.littletonrobotics.junction.Logger;

public class FieldPlanner {
  private static final double EPS = 1e-9;
  private static final double COMMIT_ON_DIST = 1.20;
  private static final long SWAP_COOLDOWN_MS = 800;
  private static final double FORCE_THROUGH_GOAL_DIST = 2.0;
  private static final double FORCE_THROUGH_WALL_DIST = 0.7;
  private static final double CORNER_CHAMFER = 0;
  public static final double GOAL_STRENGTH = 1.2;

  public interface ObstacleProvider {
    List<Obstacle> fieldObstacles();

    List<Obstacle> walls();
  }

  public static final class DefaultObstacleProvider implements ObstacleProvider {
    @Override
    public List<Obstacle> fieldObstacles() {
      return List.of();
    }

    @Override
    public List<Obstacle> walls() {
      return List.of();
    }
  }

  private Pose2d committedGoal = null;
  private long lastSwapMs = 0;
  private Optional<RepulsorSetpoint> lastChosenSetpoint = Optional.empty();

  private final TurnTuning turnTuning;
  private final DriveTuning driveTuning;
  public final ReactiveBypass bypass = new ReactiveBypass();
  private final HeadingGate headingGate = new HeadingGate();

  private final ObstacleProvider obstacleProvider;
  private final List<Obstacle> fieldObstacles;
  private final List<Obstacle> walls;

  public ObstacleProvider getObstacleProvider() {
    return obstacleProvider;
  }

  public FieldPlanner() {
    this(new DefaultTurnTuning(), new DefaultDriveTuning(), Constants.FIELD);
  }

  public FieldPlanner(ObstacleProvider obstacleProvider) {
    this(new DefaultTurnTuning(), new DefaultDriveTuning(), obstacleProvider);
  }

  public FieldPlanner(TurnTuning turnTuning, DriveTuning driveTuning) {
    this(turnTuning, driveTuning, new DefaultObstacleProvider());
  }

  public FieldPlanner(
      TurnTuning turnTuning, DriveTuning driveTuning, ObstacleProvider obstacleProvider) {
    this.turnTuning = turnTuning;
    this.driveTuning = driveTuning;
    this.obstacleProvider =
        obstacleProvider == null ? new DefaultObstacleProvider() : obstacleProvider;
    this.fieldObstacles = new ArrayList<>(this.obstacleProvider.fieldObstacles());
    this.walls = new ArrayList<>(this.obstacleProvider.walls());

    for (int i = 0; i < ARROWS_SIZE; i++) arrows.add(new Pose2d());
    String prefix = System.getenv("REACTIVE_BYPASS_ID");
    String logName;
    if (prefix != null && !prefix.isEmpty()) {
      logName = prefix + "ReactiveBypassLog.csv";
    } else {
      logName = "ReactiveBypassLog.csv";
    }
    bypass.enableLogging(logName);
  }

  private static double clamp01(double x) {
    return Math.max(0.0, Math.min(1.0, x));
  }

  private static double smooth01(double x) {
    x = clamp01(x);
    return x * x * (3.0 - 2.0 * x);
  }

  public static Translation2d[] robotRect(
      Translation2d center, Rotation2d yaw, double rx, double ry) {
    return TurnTuning.robotRect(center, yaw, rx, ry);
  }

  private Function<Translation2d[], Boolean> intersectsDynamicFn(
      List<? extends Obstacle> dynamics) {
    return rect -> rectIntersectsDynamic(rect, dynamics);
  }

  private boolean rectIntersectsDynamic(Translation2d[] rect, List<? extends Obstacle> dynamics) {
    for (Obstacle d : dynamics) if (d.intersectsRectangle(rect)) return true;
    return false;
  }

  private boolean rectIntersectsAny(Translation2d[] rect, List<? extends Obstacle> dynamics) {
    for (Obstacle w : walls) if (w.intersectsRectangle(rect)) return true;
    for (Obstacle f : fieldObstacles) if (f.intersectsRectangle(rect)) return true;
    for (Obstacle d : dynamics) if (d.intersectsRectangle(rect)) return true;
    return false;
  }

  private static Rotation2d angleOr(Translation2d v, Rotation2d fallback) {
    return v.getNorm() > EPS ? v.getAngle() : fallback;
  }

  private static Translation2d plus(Translation2d a, Translation2d b) {
    return new Translation2d(a.getX() + b.getX(), a.getY() + b.getY());
  }

  public abstract static class Obstacle {
    double strength = 1.0;
    boolean positive = true;

    public Obstacle(double strength, boolean positive) {
      this.strength = strength;
      this.positive = positive;
    }

    public abstract Force getForceAtPosition(Translation2d position, Translation2d target);

    protected double distToForceMag(double dist) {
      var forceMag = strength / (0.00001 + Math.abs(dist * dist));
      forceMag *= positive ? 1 : -1;
      return forceMag;
    }

    protected double distToForceMag(double dist, double falloff) {
      var original = strength / (0.00001 + Math.abs(dist * dist));
      var falloffMag = strength / (0.00001 + Math.abs(falloff * falloff));
      return Math.max(original - falloffMag, 0) * (positive ? 1 : -1);
    }

    public boolean intersectsRectangle(Translation2d[] rectCorners) {
      return false;
    }
  }

  public static class SquareObstacle extends Obstacle {
    public final Translation2d center;
    public final double halfSize;
    public final double maxRange;

    private static final double X_AXIS_ANGLE_BIAS_RAD = Math.toRadians(18.0);

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

      if (!inside && dist > maxRange && !occludes) return new Force();

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

      Translation2d sum = primary.plus(escape);
      if (sum.getNorm() < EPS) return new Force();
      return new Force(sum.getNorm(), sum.getAngle());
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

  public static class RectangleObstacle extends Obstacle {
    public final Translation2d center;
    public final double halfX;
    public final double halfY;
    public final Rotation2d rot;
    public final double maxRangeX;
    public final double maxRangeY;

    private static final double X_AXIS_ANGLE_BIAS_RAD = Math.toRadians(18.0);

    public RectangleObstacle(
        Translation2d center,
        double widthMeters,
        double heightMeters,
        Rotation2d rot,
        double strength,
        double maxRangeX,
        double maxRangeY) {
      super(strength, true);
      this.center = center;
      this.halfX = Math.max(0.0, widthMeters * 0.5);
      this.halfY = Math.max(0.0, heightMeters * 0.5);
      this.rot = (rot == null) ? Rotation2d.kZero : rot;
      this.maxRangeX = Math.max(0.0, maxRangeX);
      this.maxRangeY = Math.max(0.0, maxRangeY);
    }

    public RectangleObstacle(
        Translation2d center,
        double widthMeters,
        double heightMeters,
        double strength,
        double maxRangeX,
        double maxRangeY) {
      this(center, widthMeters, heightMeters, Rotation2d.kZero, strength, maxRangeX, maxRangeY);
    }

    public RectangleObstacle(
        Translation2d center,
        double widthMeters,
        double heightMeters,
        double strength,
        double maxRange) {
      this(center, widthMeters, heightMeters, Rotation2d.kZero, strength, maxRange, maxRange);
    }

    @Override
    public Force getForceAtPosition(Translation2d position, Translation2d target) {
      Translation2d pLocal = position.minus(center).rotateBy(rot.unaryMinus());
      Translation2d gLocal = target.minus(center).rotateBy(rot.unaryMinus());

      double px = pLocal.getX();
      double py = pLocal.getY();

      double cx = Math.max(-halfX, Math.min(px, halfX));
      double cy = Math.max(-halfY, Math.min(py, halfY));

      double dx = px - cx;
      double dy = py - cy;

      boolean inside = (px >= -halfX && px <= halfX && py >= -halfY && py <= halfY);

      double ax = inside ? 0.0 : Math.max(0.0, Math.abs(dx));
      double ay = inside ? 0.0 : Math.max(0.0, Math.abs(dy));

      Translation2d[] poly = corners();
      boolean occludes = segmentIntersectsPolygon(position, target, poly);

      if (!inside && (ax > maxRangeX || ay > maxRangeY) && !occludes) return new Force();

      Translation2d awayLocal;
      double effectiveDist;

      if (!inside) {
        if (ax < EPS && ay < EPS) return new Force();

        awayLocal = new Translation2d(dx, dy);
        if (awayLocal.getNorm() < EPS) return new Force();

        if (Math.abs(dx) > Math.abs(dy)) {
          double sign = Math.signum(target.getY() - position.getY());
          if (sign == 0.0) sign = 1.0;
          Rotation2d biasedAngle =
              awayLocal.getAngle().rotateBy(Rotation2d.fromRadians(sign * X_AXIS_ANGLE_BIAS_RAD));
          double norm = awayLocal.getNorm();
          awayLocal = new Translation2d(norm, biasedAngle);
        }

        double sx = (maxRangeX > EPS) ? (ax / maxRangeX) : 0.0;
        double sy = (maxRangeY > EPS) ? (ay / maxRangeY) : 0.0;
        double s = Math.hypot(sx, sy);
        effectiveDist = Math.max(EPS, s);
      } else {
        double dL = px + halfX;
        double dR = halfX - px;
        double dB = py + halfY;
        double dT = halfY - py;

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

      double falloff = Math.max(EPS, Math.hypot(1.0, 1.0));
      double mag = distToForceMag(effectiveDist, falloff);

      Translation2d primaryWorld = Translation2d.kZero;
      if (Math.abs(mag) >= EPS && awayLocal.getNorm() >= EPS) {
        Translation2d awayWorld = awayLocal.rotateBy(rot);
        primaryWorld = new Translation2d(Math.abs(mag), awayWorld.getAngle());
      }

      Translation2d escapeWorld = Translation2d.kZero;

      if (occludes) {
        double diag = Math.hypot(halfX, halfY);
        double engageR = Math.max(0.9, Math.max(maxRangeX, maxRangeY) + diag + 1.1);

        if (position.getDistance(center) <= engageR) {
          Translation2d outwardW = position.minus(center);
          if (outwardW.getNorm() < EPS) outwardW = new Translation2d(1.0, 0.0);
          Translation2d outwardWU = outwardW.div(Math.max(EPS, outwardW.getNorm()));

          Translation2d toC = center.minus(position);
          if (toC.getNorm() < EPS) toC = new Translation2d(1.0, 0.0);
          Translation2d tCCW =
              new Translation2d(-toC.getY(), toC.getX()).div(Math.max(EPS, toC.getNorm()));
          Translation2d tCW =
              new Translation2d(toC.getY(), -toC.getX()).div(Math.max(EPS, toC.getNorm()));

          double pad = Math.max(0.55, Math.min(2.0, Math.max(maxRangeX, maxRangeY) * 0.85));
          Translation2d[] polyExp = expandedCorners(pad);

          Translation2d chosenT =
              chooseTangentPoly(position, target, outwardWU, tCW, tCCW, polyExp);

          double d = Math.max(0.15, position.getDistance(center) - diag);
          double followMag = (strength * 8.0) / (0.35 + d * d);
          double pushOutMag = (strength * 2.2) / (0.45 + d * d);

          Translation2d follow = chosenT.times(followMag);
          Translation2d pushOut = outwardWU.times(pushOutMag);

          Translation2d g = target.minus(position);
          if (g.getNorm() > EPS) {
            double along = dot(chosenT, g.div(g.getNorm()));
            if (along < 0.10) {
              follow = follow.times(1.35);
            }
          }

          escapeWorld = escapeWorld.plus(follow).plus(pushOut);
        }
      }

      Translation2d sum = primaryWorld.plus(escapeWorld);
      if (sum.getNorm() < EPS) return new Force();
      return new Force(sum.getNorm(), sum.getAngle());
    }

    private Translation2d[] corners() {
      Translation2d[] local =
          new Translation2d[] {
            new Translation2d(-halfX, -halfY),
            new Translation2d(halfX, -halfY),
            new Translation2d(halfX, halfY),
            new Translation2d(-halfX, halfY)
          };
      Translation2d[] out = new Translation2d[4];
      for (int i = 0; i < 4; i++) out[i] = local[i].rotateBy(rot).plus(center);
      return out;
    }

    private Translation2d[] expandedCorners(double pad) {
      Translation2d[] c = corners();
      Translation2d[] out = new Translation2d[c.length];
      for (int i = 0; i < c.length; i++) {
        Translation2d v = c[i].minus(center);
        double n = Math.max(EPS, v.getNorm());
        Translation2d u = v.div(n);
        out[i] = center.plus(u.times(n + pad));
      }
      return out;
    }

    private static Translation2d chooseTangentPoly(
        Translation2d pos,
        Translation2d goal,
        Translation2d outwardU,
        Translation2d tCW,
        Translation2d tCCW,
        Translation2d[] polyExp) {

      double stepT = 0.55;
      double stepO = 0.22;

      Translation2d pCW = pos.plus(tCW.times(stepT)).plus(outwardU.times(stepO));
      Translation2d pCCW = pos.plus(tCCW.times(stepT)).plus(outwardU.times(stepO));

      double base = pos.getDistance(goal);

      double sCW = scoreCandidatePoly(pos, goal, pCW, base, polyExp);
      double sCCW = scoreCandidatePoly(pos, goal, pCCW, base, polyExp);

      return (sCW <= sCCW) ? tCW : tCCW;
    }

    private static double scoreCandidatePoly(
        Translation2d pos,
        Translation2d goal,
        Translation2d cand,
        double baseDist,
        Translation2d[] polyExp) {

      double d = cand.getDistance(goal);

      boolean stillOccluding = segmentIntersectsPolygon(cand, goal, polyExp);
      double occPenalty = stillOccluding ? 0.65 : 0.0;

      double progressPenalty = (d >= baseDist - 0.01) ? 0.35 : 0.0;

      return d + occPenalty + progressPenalty;
    }

    private static boolean segmentIntersectsPolygon(
        Translation2d a, Translation2d b, Translation2d[] poly) {
      if (FieldPlanner.isPointInPolygon(a, poly) || FieldPlanner.isPointInPolygon(b, poly))
        return true;
      for (int i = 0; i < poly.length; i++) {
        Translation2d c = poly[i];
        Translation2d d = poly[(i + 1) % poly.length];
        if (segmentsIntersect(a, b, c, d)) return true;
      }
      return false;
    }

    private static double orient(Translation2d a, Translation2d b, Translation2d c) {
      return (b.getX() - a.getX()) * (c.getY() - a.getY())
          - (b.getY() - a.getY()) * (c.getX() - a.getX());
    }

    private static boolean onSeg(Translation2d a, Translation2d b, Translation2d p) {
      return p.getX() >= Math.min(a.getX(), b.getX()) - 1e-9
          && p.getX() <= Math.max(a.getX(), b.getX()) + 1e-9
          && p.getY() >= Math.min(a.getY(), b.getY()) - 1e-9
          && p.getY() <= Math.max(a.getY(), b.getY()) + 1e-9;
    }

    private static boolean segmentsIntersect(
        Translation2d a, Translation2d b, Translation2d c, Translation2d d) {
      double o1 = orient(a, b, c);
      double o2 = orient(a, b, d);
      double o3 = orient(c, d, a);
      double o4 = orient(c, d, b);

      if ((o1 > 0) != (o2 > 0) && (o3 > 0) != (o4 > 0)) return true;

      if (Math.abs(o1) < 1e-9 && onSeg(a, b, c)) return true;
      if (Math.abs(o2) < 1e-9 && onSeg(a, b, d)) return true;
      if (Math.abs(o3) < 1e-9 && onSeg(c, d, a)) return true;
      if (Math.abs(o4) < 1e-9 && onSeg(c, d, b)) return true;

      return false;
    }

    @Override
    public boolean intersectsRectangle(Translation2d[] rectCorners) {
      Translation2d[] me = corners();
      for (int i = 0; i < me.length; i++) {
        Translation2d a = me[i];
        Translation2d b = me[(i + 1) % me.length];
        for (int j = 0; j < rectCorners.length; j++) {
          Translation2d c = rectCorners[j];
          Translation2d d = rectCorners[(j + 1) % rectCorners.length];
          if (segmentsIntersect(a, b, c, d)) return true;
        }
      }
      for (Translation2d p : rectCorners) if (FieldPlanner.isPointInPolygon(p, me)) return true;
      for (Translation2d p : me) if (FieldPlanner.isPointInPolygon(p, rectCorners)) return true;
      return false;
    }
  }

  static class PointObstacle extends Obstacle {
    Translation2d loc;
    double radius = 0.5;

    public PointObstacle(Translation2d loc, double strength, boolean positive) {
      super(strength, positive);
      this.loc = loc;
    }

    public Force getForceAtPosition(Translation2d position, Translation2d target) {
      var dist = loc.getDistance(position);
      if (dist > 4) return new Force();
      if (dist < EPS) return new Force();

      var outwardsMag = distToForceMag(dist - radius);
      var away = position.minus(loc);

      var theta = target.minus(position).getAngle().minus(away.getAngle());
      double mag = outwardsMag * Math.signum(Math.sin(theta.getRadians() / 2)) / 2;

      var combined =
          plus(
              new Translation2d(mag, away.getAngle().rotateBy(Rotation2d.kCCW_90deg)),
              new Translation2d(outwardsMag, away.getAngle()));

      if (combined.getNorm() < EPS) return new Force();
      return new Force(combined.getNorm(), combined.getAngle());
    }

    @Override
    public boolean intersectsRectangle(Translation2d[] rectCorners) {
      if (FieldPlanner.isPointInPolygon(loc, rectCorners)) return true;
      for (Translation2d corner : rectCorners) if (corner.getDistance(loc) < radius) return true;
      for (int i = 0; i < rectCorners.length; i++) {
        Translation2d a = rectCorners[i];
        Translation2d b = rectCorners[(i + 1) % rectCorners.length];
        if (FieldPlanner.distanceFromPointToSegment(loc, a, b) < radius) return true;
      }
      return false;
    }
  }

  static class SnowmanObstacle extends Obstacle {
    Translation2d loc;
    double radius = 0.5;

    public SnowmanObstacle(Translation2d loc, double strength, double radius, boolean positive) {
      super(strength, positive);
      this.loc = loc;
      this.radius = radius;
    }

    public Force getForceAtPosition(Translation2d position, Translation2d target) {
      var targetToLoc = loc.minus(target);
      var targetToLocAngle = angleOr(targetToLoc, Rotation2d.kZero);

      var sidewaysCircle = new Translation2d(1, targetToLocAngle).plus(loc);

      var distLoc = Math.max(EPS, loc.getDistance(position));
      var sidewaysMag = distToForceMag(sidewaysCircle.getDistance(position));
      var outwardsMag = distToForceMag(Math.max(0.01, distLoc - radius));

      var away = position.minus(loc);
      var awayAngle = angleOr(away, targetToLocAngle);

      var sidewaysTheta =
          target
              .minus(position)
              .getAngle()
              .minus(angleOr(position.minus(sidewaysCircle), awayAngle));
      double sideways = sidewaysMag * Math.signum(Math.sin(sidewaysTheta.getRadians()));
      var sidewaysAngle = targetToLocAngle.rotateBy(Rotation2d.kCCW_90deg);

      var combined =
          plus(
              new Translation2d(sideways, sidewaysAngle),
              new Translation2d(outwardsMag, awayAngle));
      if (combined.getNorm() < EPS) return new Force();
      return new Force(combined.getNorm(), combined.getAngle());
    }

    @Override
    public boolean intersectsRectangle(Translation2d[] rectCorners) {
      if (FieldPlanner.isPointInPolygon(loc, rectCorners)) return true;
      for (Translation2d corner : rectCorners) if (corner.getDistance(loc) < radius) return true;
      for (int i = 0; i < rectCorners.length; i++) {
        Translation2d a = rectCorners[i];
        Translation2d b = rectCorners[(i + 1) % rectCorners.length];
        if (FieldPlanner.distanceFromPointToSegment(loc, a, b) < radius) return true;
      }
      return false;
    }
  }

  public static class HorizontalObstacle extends Obstacle {
    public double y;

    public HorizontalObstacle(double y, double strength, boolean positive) {
      super(strength, positive);
      this.y = y;
    }

    public Force getForceAtPosition(Translation2d position, Translation2d target) {
      return new Force(0, distToForceMag(y - position.getY(), 1));
    }

    @Override
    public boolean intersectsRectangle(Translation2d[] rectCorners) {
      for (Translation2d a : rectCorners) if (Math.abs(a.getY() - y) < 0.1) return true;
      return false;
    }
  }

  public static class VerticalObstacle extends Obstacle {
    public double x;

    public VerticalObstacle(double x, double strength, boolean positive) {
      super(strength, positive);
      this.x = x;
    }

    public Force getForceAtPosition(Translation2d position, Translation2d target) {
      return new Force(distToForceMag(x - position.getX(), 1), 0);
    }

    @Override
    public boolean intersectsRectangle(Translation2d[] rectCorners) {
      for (Translation2d a : rectCorners) if (Math.abs(a.getX() - x) < 0.1) return true;
      return false;
    }
  }

  public static class TeardropObstacle extends Obstacle {
    public final Translation2d loc;
    public final double primaryMaxRange;
    public final double primaryRadius;
    public final double tailStrength;
    public final double tailLength;
    final double tiny = EPS;

    public TeardropObstacle(
        Translation2d loc,
        double primaryStrength,
        double primaryMaxRange,
        double primaryRadius,
        double tailStrength,
        double tailLength) {
      super(primaryStrength, true);
      this.loc = loc;
      this.primaryMaxRange = primaryMaxRange;
      this.primaryRadius = primaryRadius;
      this.tailStrength = tailStrength;
      this.tailLength = tailLength + primaryMaxRange;
    }

    Rotation2d angleFromVec(Translation2d v, Rotation2d fallback) {
      double x = v.getX(), y = v.getY();
      double n = Math.hypot(x, y);
      return (n > tiny) ? Rotation2d.fromRadians(Math.atan2(y, x)) : fallback;
    }

    Rotation2d angleBetween(Translation2d from, Translation2d to, Rotation2d fallback) {
      double dx = to.getX() - from.getX(), dy = to.getY() - from.getY();
      double n = Math.hypot(dx, dy);
      return (n > tiny) ? Rotation2d.fromRadians(Math.atan2(dy, dx)) : fallback;
    }

    public Force getForceAtPosition(Translation2d position, Translation2d target) {
      var targetToLoc = new Translation2d(loc.getX() - target.getX(), loc.getY() - target.getY());
      var targetToLocAngle = angleFromVec(targetToLoc, Rotation2d.kZero);
      var sidewaysPoint = new Translation2d(tailLength, targetToLocAngle).plus(loc);

      var posToLoc = new Translation2d(position.getX() - loc.getX(), position.getY() - loc.getY());
      double distPosLoc = posToLoc.getNorm();

      Translation2d outwardsForce;
      if (distPosLoc <= primaryMaxRange && distPosLoc >= tiny) {
        double mag =
            distToForceMag(
                Math.max(distPosLoc - primaryRadius, 0), primaryMaxRange - primaryRadius);
        var dir = angleFromVec(posToLoc, targetToLocAngle);
        outwardsForce = new Translation2d(mag, dir);
      } else {
        outwardsForce = Translation2d.kZero;
      }

      var positionRel =
          new Translation2d(position.getX() - loc.getX(), position.getY() - loc.getY())
              .rotateBy(targetToLocAngle.unaryMinus());
      double distanceAlongLine = positionRel.getX();

      Translation2d sidewaysForce;
      double distanceScalar =
          (Math.abs(tailLength) > tiny) ? (distanceAlongLine / tailLength) : 0.0;
      if (distanceScalar >= 0 && distanceScalar <= 1) {
        double secondaryMaxRange =
            MathUtil.interpolate(primaryMaxRange, 0, distanceScalar * distanceScalar);
        double distanceToLine = Math.abs(positionRel.getY());
        if (distanceToLine <= secondaryMaxRange) {
          double strength;
          if (distanceAlongLine < primaryMaxRange) {
            strength = tailStrength * (distanceAlongLine / Math.max(primaryMaxRange, tiny));
          } else {
            double denom = Math.max(tailLength - primaryMaxRange, tiny);
            strength =
                -tailStrength * distanceAlongLine / denom + tailLength * tailStrength / denom;
          }
          strength *= 1 - distanceToLine / Math.max(secondaryMaxRange, tiny);

          double sidewaysMagBase = tailStrength * strength * (secondaryMaxRange - distanceToLine);

          var posMinusSideways =
              new Translation2d(
                  position.getX() - sidewaysPoint.getX(), position.getY() - sidewaysPoint.getY());
          var posMinusSidewaysAngle = angleFromVec(posMinusSideways, targetToLocAngle);
          var toTarget = angleBetween(position, target, posMinusSidewaysAngle);
          var sidewaysTheta =
              new Rotation2d(toTarget.getCos(), toTarget.getSin()).minus(posMinusSidewaysAngle);

          double dir = Math.signum(Math.sin(sidewaysTheta.getRadians()));
          double sidewaysMag = (Math.abs(dir) < tiny) ? 0.0 : sidewaysMagBase * dir;

          sidewaysForce =
              new Translation2d(sidewaysMag, targetToLocAngle.rotateBy(Rotation2d.kCCW_90deg));
        } else {
          sidewaysForce = Translation2d.kZero;
        }
      } else {
        sidewaysForce = Translation2d.kZero;
      }

      var sum =
          new Translation2d(
              outwardsForce.getX() + sidewaysForce.getX(),
              outwardsForce.getY() + sidewaysForce.getY());
      if (sum.getNorm() < tiny) return new Force();
      return new Force(sum.getNorm(), angleFromVec(sum, Rotation2d.kZero));
    }

    @Override
    public boolean intersectsRectangle(Translation2d[] rectCorners) {
      if (FieldPlanner.isPointInPolygon(loc, rectCorners)) return true;
      for (Translation2d corner : rectCorners)
        if (corner.getDistance(loc) < primaryMaxRange) return true;
      for (int i = 0; i < rectCorners.length; i++) {
        Translation2d a = rectCorners[i];
        Translation2d b = rectCorners[(i + 1) % rectCorners.length];
        if (FieldPlanner.distanceFromPointToSegment(loc, a, b) < primaryMaxRange) return true;
      }

      Rotation2d tailDir = new Rotation2d();
      Translation2d tailStart = loc;
      Translation2d tailEnd = tailStart.plus(new Translation2d(tailLength, tailDir));

      for (Translation2d corner : rectCorners) {
        Translation2d delta = tailEnd.minus(tailStart);
        Translation2d unit = delta.div(delta.getNorm());
        double projection = dot(corner.minus(tailStart), unit);
        if (projection >= 0 && projection <= tailLength) {
          double lateral =
              Math.abs((corner.minus(tailStart)).rotateBy(tailDir.unaryMinus()).getY());
          if (lateral < primaryMaxRange) return true;
        }
      }
      return false;
    }
  }

  public static class DiagonalWallObstacle extends Obstacle {
    public final Translation2d a;
    public final Translation2d b;
    public final double maxRange;

    public DiagonalWallObstacle(
        Translation2d a, Translation2d b, double strength, double maxRange) {
      super(strength, true);
      this.a = a;
      this.b = b;
      this.maxRange = maxRange;
    }

    @Override
    public Force getForceAtPosition(Translation2d position, Translation2d target) {
      double dist = distanceFromPointToSegment(position, a, b);
      if (dist < EPS || dist > maxRange) return new Force();

      double mag = distToForceMag(dist, maxRange);

      Translation2d closest = closestPoint(position);
      Translation2d away = position.minus(closest);
      if (away.getNorm() < EPS) return new Force();

      Translation2d vec = new Translation2d(mag, away.getAngle());
      return new Force(vec.getNorm(), vec.getAngle());
    }

    private Translation2d closestPoint(Translation2d p) {
      Translation2d ab = b.minus(a);
      double abLenSq = ab.getNorm() * ab.getNorm();
      if (abLenSq < EPS) return a;
      double t = Math.max(0, Math.min(1, dot(p.minus(a), ab) / abLenSq));
      return a.plus(ab.times(t));
    }

    @Override
    public boolean intersectsRectangle(Translation2d[] rectCorners) {
      for (Translation2d corner : rectCorners) {
        if (distanceFromPointToSegment(corner, a, b) < 0.1) return true;
      }
      if (FieldPlanner.isPointInPolygon(a, rectCorners)) return true;
      if (FieldPlanner.isPointInPolygon(b, rectCorners)) return true;
      return false;
    }
  }

  private boolean isInsideChamferedField(Translation2d p) {
    double x = p.getX();
    double y = p.getY();
    double L = Constants.FIELD_LENGTH;
    double W = Constants.FIELD_WIDTH;

    if (x < 0.0 || x > L || y < 0.0 || y > W) return false;

    if (x <= CORNER_CHAMFER && y <= CORNER_CHAMFER && x + y < CORNER_CHAMFER) return false;

    if (x >= L - CORNER_CHAMFER && y <= CORNER_CHAMFER && (L - x) + y < CORNER_CHAMFER)
      return false;

    if (x <= CORNER_CHAMFER && y >= W - CORNER_CHAMFER && x + (W - y) < CORNER_CHAMFER)
      return false;

    if (x >= L - CORNER_CHAMFER && y >= W - CORNER_CHAMFER && (L - x) + (W - y) < CORNER_CHAMFER)
      return false;

    return true;
  }

  private Pose2d goal = Pose2d.kZero;

  private static final int ARROWS_X = RobotBase.isSimulation() ? 40 : 0;
  private static final int ARROWS_Y = RobotBase.isSimulation() ? 20 : 0;
  private static final int ARROWS_SIZE = (ARROWS_X + 1) * (ARROWS_Y + 1);

  private ArrayList<Pose2d> arrows = new ArrayList<>(ARROWS_SIZE);

  private Optional<Distance> currentErr = Optional.empty();
  private Optional<PlannerFallback> fallback = Optional.empty();

  public boolean suppressIsClearPath = false;
  private int stuckStepCount = 0;
  private static final int MAX_STUCK_STEPS = 40;

  private Pose2d arrowBackstage = new Pose2d(-10, -10, Rotation2d.kZero);

  public List<Obstacle> getObstacles() {
    return fieldObstacles;
  }

  public Translation2d getGoal() {
    return goal.getTranslation();
  }

  public FieldPlanner withFallback(PlannerFallback _fallback) {
    fallback = Optional.of(_fallback);
    return this;
  }

  public void updateArrows(List<? extends Obstacle> dynamicObstacles) {
    if (RobotBase.isReal()) {
      return;
    }

    for (int x = 0; x <= ARROWS_X; x++) {
      for (int y = 0; y <= ARROWS_Y; y++) {
        var translation =
            new Translation2d(
                x * Constants.FIELD_LENGTH / ARROWS_X, y * Constants.FIELD_WIDTH / ARROWS_Y);

        int idx = x * (ARROWS_Y + 1) + y;

        if (!isInsideChamferedField(translation)) {
          arrows.set(idx, arrowBackstage);
          continue;
        }

        var force = Force.kZero;
        force = force.plus(getObstacleForce(translation, goal.getTranslation(), dynamicObstacles));
        force = force.plus(getWallForce(translation, goal.getTranslation()));
        force = force.plus(getGoalForce(translation, goal.getTranslation()));

        if (force.getNorm() < 1e-6) {
          arrows.set(idx, arrowBackstage);
        } else {
          var rotation = force.getAngle();
          arrows.set(idx, new Pose2d(translation, rotation));
        }
      }
    }
    Logger.recordOutput("Repulsor/Arrows", arrows.toArray(new Pose2d[0]));
  }

  public ArrayList<Pose2d> getArrows() {
    return arrows;
  }

  Force getGoalForce(Translation2d curLocation, Translation2d goal) {
    var displacement = goal.minus(curLocation);
    if (displacement.getNorm() == 0) return new Force();
    var direction = displacement.getAngle();
    var mag =
        GOAL_STRENGTH * (1 + 1.0 / (0.0001 + displacement.getNorm() * displacement.getNorm()));
    return new Force(mag, direction);
  }

  Force getWallForce(Translation2d curLocation, Translation2d target) {
    var force = Force.kZero;
    for (Obstacle obs : walls) force = force.plus(obs.getForceAtPosition(curLocation, target));
    return force;
  }

  Force getObstacleForce(
      Translation2d curLocation, Translation2d target, List<? extends Obstacle> extra) {
    var force = Force.kZero;
    for (Obstacle obs : fieldObstacles)
      force = force.plus(obs.getForceAtPosition(curLocation, target));
    for (Obstacle obs : extra) force = force.plus(obs.getForceAtPosition(curLocation, target));
    return force;
  }

  Force getObstacleForce(Translation2d curLocation, Translation2d target) {
    var force = Force.kZero;
    for (Obstacle obs : fieldObstacles)
      force = force.plus(obs.getForceAtPosition(curLocation, target));
    return force;
  }

  Force getForce(Translation2d curLocation, Translation2d target) {
    return getGoalForce(curLocation, target)
        .plus(getObstacleForce(curLocation, target))
        .plus(getWallForce(curLocation, target))
        .times(Math.min(1.0, curLocation.getDistance(target)));
  }

  public static class RepulsorSample {
    private Translation2d m_goal;
    private LinearVelocity m_vx;
    private LinearVelocity m_vy;
    private Angle m_omega;

    public RepulsorSample(Translation2d goal, double vx, double vy, Angle omega) {
      m_goal = goal;
      m_vx = MetersPerSecond.of(vx);
      m_vy = MetersPerSecond.of(vy);
      m_omega = omega;
    }

    public RepulsorSample(Translation2d goal, ChassisSpeeds v) {
      m_goal = goal;
      m_vx = MetersPerSecond.of(v.vxMetersPerSecond);
      m_vy = MetersPerSecond.of(v.vyMetersPerSecond);
    }

    public RepulsorSample(Translation2d goal, ChassisSpeeds v, Angle omega) {
      this(goal, v);
      m_omega = omega;
    }

    public ChassisSpeeds asChassisSpeeds(PIDController omegaPID, Rotation2d currentRot) {
      double desiredYaw = (m_omega == null) ? currentRot.getRadians() : m_omega.in(Radians);
      double omegaCmd = omegaPID.calculate(currentRot.getRadians(), desiredYaw);
      return ChassisSpeeds.fromFieldRelativeSpeeds(
          m_vx, m_vy, RadiansPerSecond.of(omegaCmd), currentRot);
    }

    public Translation2d goal() {
      return m_goal;
    }
  }

  public void setGoal(Pose2d goal) {
    this.goal = goal;
    Logger.recordOutput("Repulsor/Setpoint", goal);
  }

  public Optional<Distance> getErr() {
    return currentErr;
  }

  public void clearCommitted() {
    committedGoal = null;
  }

  public RepulsorSample calculateAndClear(
      Pose2d pose,
      List<? extends Obstacle> dynamicObstacles,
      double robot_x,
      double robot_y,
      double coral_offset,
      double algae_offset,
      CategorySpec cat,
      double shooterReleaseHeightMeters) {
    clearCommitted();
    return calculate(
        pose,
        dynamicObstacles,
        robot_x,
        robot_y,
        coral_offset,
        algae_offset,
        cat,
        false,
        shooterReleaseHeightMeters);
  }

  public RepulsorSample calculate(
      Pose2d pose,
      List<? extends Obstacle> dynamicObstacles,
      double robot_x,
      double robot_y,
      double coral_offset,
      double algae_offset,
      CategorySpec cat,
      boolean suppressFallback,
      double shooterReleaseHeightMeters) {

    long now = System.currentTimeMillis();

    Translation2d curTrans = pose.getTranslation();
    double distToGoal = curTrans.getDistance(goal.getTranslation());

    boolean forceThrough = bypass.isPinnedMode();
    List<? extends Obstacle> effectiveDynamics =
        forceThrough ? Collections.emptyList() : dynamicObstacles;

    if (!forceThrough && !suppressFallback) {
      boolean blockedWithDynamics =
          !ExtraPathing.isClearPath(
              "Repulsor/ForceThrough/WithDyn",
              curTrans,
              goal.getTranslation(),
              dynamicObstacles,
              robot_x,
              robot_y,
              false);

      boolean blockedWithoutDynamics =
          !ExtraPathing.isClearPath(
              "Repulsor/ForceThrough/NoDyn",
              curTrans,
              goal.getTranslation(),
              Collections.emptyList(),
              robot_x,
              robot_y,
              false);

      double dxWall = Math.min(curTrans.getX(), Constants.FIELD_LENGTH - curTrans.getX());
      double dyWall = Math.min(curTrans.getY(), Constants.FIELD_WIDTH - curTrans.getY());
      double dWall = Math.min(dxWall, dyWall);
      boolean nearWall = dWall < FORCE_THROUGH_WALL_DIST;
      boolean nearGoal = distToGoal <= FORCE_THROUGH_GOAL_DIST;

      if (blockedWithDynamics && !blockedWithoutDynamics && nearGoal && nearWall) {
        forceThrough = true;
        effectiveDynamics = Collections.emptyList();
      }
    }

    if (!suppressFallback) {
      if (!forceThrough
          && ExtraPathing.robotIntersects(curTrans, robot_x, robot_y, dynamicObstacles)) {
        Logger.recordOutput("Repulsor/Encapsulated", true);
        currentErr = Optional.of(Meters.of(curTrans.getDistance(goal.getTranslation())));
        return new RepulsorSample(curTrans, 0, 0, Radians.of(pose.getRotation().getRadians()));
      } else {
        Logger.recordOutput("Repulsor/Encapsulated", false);
      }

      if (committedGoal != null) {
        boolean stillClear =
            ExtraPathing.isClearPath(
                "Repulsor/IsClear/Committed",
                curTrans,
                committedGoal.getTranslation(),
                effectiveDynamics,
                robot_x,
                robot_y,
                false);
        if (stillClear || (now - lastSwapMs) < SWAP_COOLDOWN_MS) {
          goal = committedGoal;
          Logger.recordOutput("Repulsor/Setpoint", goal);
        } else {
          Logger.recordOutput("Repulsor/Commit/Abort", committedGoal);
          committedGoal = null;
        }
      }

      if (committedGoal == null) {
        if (distToGoal <= COMMIT_ON_DIST) {
          boolean clear =
              ExtraPathing.isClearPath(
                  "Repulsor/IsClear/ToCommit",
                  curTrans,
                  goal.getTranslation(),
                  effectiveDynamics,
                  robot_x,
                  robot_y,
                  false);
          if (clear) {
            committedGoal = goal;
            lastChosenSetpoint = Optional.empty();
            lastSwapMs = now;
            Logger.recordOutput("Repulsor/Commit/Set", committedGoal);
          }
        }
      }

      boolean pathBlocked = false;
      if (!suppressIsClearPath) {
        pathBlocked =
            !ExtraPathing.isClearPath(
                "Repulsor/IsClear",
                curTrans,
                goal.getTranslation(),
                effectiveDynamics,
                robot_x,
                robot_y,
                true);
      }

      if (pathBlocked && !suppressFallback) {
        Alliance preferred =
            DriverStation.getAlliance().isPresent()
                    && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue
                ? Alliance.kBlue
                : Alliance.kRed;

        var cands =
            FieldTracker.getInstance().getPredictedSetpoints(preferred, curTrans, 3.5, cat, 8);

        SetpointContext spCtx =
            new SetpointContext(
                Optional.of(pose),
                Math.max(0.0, robot_x) * 2.0,
                Math.max(0.0, robot_y) * 2.0,
                coral_offset,
                algae_offset,
                shooterReleaseHeightMeters,
                effectiveDynamics);

        for (RepulsorSetpoint sp : cands) {
          Pose2d altGoal = sp.get(spCtx);

          if (altGoal.getTranslation().getDistance(goal.getTranslation()) < 1e-3) continue;
          if (committedGoal != null
              && altGoal.getTranslation().getDistance(committedGoal.getTranslation()) < 1e-3)
            continue;

          boolean clear =
              ExtraPathing.isClearPath(
                  "Repulsor/IsClear/Reroute",
                  curTrans,
                  altGoal.getTranslation(),
                  effectiveDynamics,
                  robot_x,
                  robot_y,
                  true);

          if (clear) {
            setGoal(altGoal);
            committedGoal = altGoal;
            lastChosenSetpoint = Optional.of(sp);
            lastSwapMs = now;
            Logger.recordOutput("Repulsor/Reroute/Chosen", altGoal);
            pathBlocked = false;
            break;
          }
        }

        if (pathBlocked) {
          Logger.recordOutput("Repulsor/Reroute/Chosen", "None_StayStill");
          return new RepulsorSample(curTrans, 0, 0, Radians.of(pose.getRotation().getRadians()));
        }
      }
    }

    final List<? extends Obstacle> effectiveDynamicsFinal = effectiveDynamics;

    updateArrows(effectiveDynamicsFinal);

    var err = curTrans.minus(goal.getTranslation());
    currentErr = Optional.of(Meters.of(err.getNorm()));

    if (err.getNorm() < 0.04) {
      return new RepulsorSample(curTrans, 0, 0, Radians.of(goal.getRotation().getRadians()));
    }

    if (fallback.isPresent() && fallback.get().within(err)) {
      Logger.recordOutput("Repulsor/Fallback/Engaged", true);
      var speeds = fallback.get().calculate(curTrans, goal.getTranslation());
      return new RepulsorSample(
          goal.getTranslation(), speeds, Radians.of(pose.getRotation().getRadians()));
    }
    Logger.recordOutput("Repulsor/Fallback/Engaged", false);

    var obstacleForceToGoal =
        getObstacleForce(curTrans, goal.getTranslation(), effectiveDynamicsFinal)
            .plus(getWallForce(curTrans, goal.getTranslation()));
    var netForceToGoal = getGoalForce(curTrans, goal.getTranslation()).plus(obstacleForceToGoal);
    Rotation2d headingToGoal = netForceToGoal.getAngle();

    var maybeBypass =
        bypass.update(
            pose,
            goal,
            headingToGoal,
            driveTuning.dtSeconds(),
            robot_x,
            robot_y,
            effectiveDynamicsFinal,
            rect -> rectIntersectsDynamic(rect, effectiveDynamicsFinal),
            tag ->
                ExtraPathing.isClearPath(
                    "Repulsor/Bypass/Rejoin",
                    curTrans,
                    goal.getTranslation(),
                    effectiveDynamicsFinal,
                    robot_x,
                    robot_y,
                    true));

    Pose2d effectiveGoal = maybeBypass.orElse(goal);
    Logger.recordOutput("Repulsor/Goal/Effective", effectiveGoal);

    var obstacleForce =
        getObstacleForce(curTrans, effectiveGoal.getTranslation(), effectiveDynamicsFinal)
            .plus(getWallForce(curTrans, effectiveGoal.getTranslation()));
    var netForce = getGoalForce(curTrans, effectiveGoal.getTranslation()).plus(obstacleForce);
    var dist = curTrans.getDistance(effectiveGoal.getTranslation());

    double stepSize_m =
        driveTuning.stepSizeMeters(dist, obstacleForce.getNorm(), (cat == CategorySpec.kScore));
    var step = new Translation2d(stepSize_m, netForce.getAngle());

    if (step.getNorm() < 1e-3) {
      stuckStepCount++;
    } else {
      stuckStepCount = 0;
    }

    if (stuckStepCount >= MAX_STUCK_STEPS) {
      System.out.println("[Repulsor] Stuck! Aborting after " + stuckStepCount + " tiny steps.");
      return new RepulsorSample(curTrans, 0, 0, Radians.of(pose.getRotation().getRadians()));
    }

    Rotation2d desiredHeadingRaw = netForce.getAngle();
    Rotation2d desiredHeading =
        headingGate.filter(pose.getRotation(), desiredHeadingRaw, driveTuning.dtSeconds());

    var turn =
        turnTuning.plan(
            pose,
            effectiveGoal,
            desiredHeading,
            step,
            (cat == CategorySpec.kScore),
            robot_x,
            robot_y,
            rect -> rectIntersectsAny(rect, effectiveDynamicsFinal));

    step = step.times(turn.speedScale);

    if (!RobotBase.isReal()) {
      Pose2d arrowPose = new Pose2d(curTrans, netForce.getAngle());
      Logger.recordOutput("Repulsor/ArrowFinal", arrowPose);
    }

    return new RepulsorSample(
        effectiveGoal.getTranslation(),
        step.getX() / driveTuning.dtSeconds(),
        step.getY() / driveTuning.dtSeconds(),
        Radians.of(turn.yaw.getRadians()));
  }

  public static boolean isPointInPolygon(Translation2d point, Translation2d[] polygon) {
    int crossings = 0;
    for (int i = 0; i < polygon.length; i++) {
      Translation2d a = polygon[i];
      Translation2d b = polygon[(i + 1) % polygon.length];
      boolean cond1 = (a.getY() > point.getY()) != (b.getY() > point.getY());
      double slope =
          (b.getX() - a.getX()) * (point.getY() - a.getY()) / (b.getY() - a.getY()) + a.getX();
      if (cond1 && point.getX() < slope) crossings++;
    }
    return (crossings % 2 == 1);
  }

  public static double dot(Translation2d a, Translation2d b) {
    return a.getX() * b.getX() + a.getY() * b.getY();
  }

  public static double distanceFromPointToSegment(
      Translation2d p, Translation2d a, Translation2d b) {
    Translation2d ap = p.minus(a);
    Translation2d ab = b.minus(a);
    double abLenSquared = ab.getNorm() * ab.getNorm();
    if (abLenSquared == 0) return ap.getNorm();
    double t = Math.max(0, Math.min(1, dot(ap, ab) / abLenSquared));
    Translation2d projection = a.plus(ab.times(t));
    return p.getDistance(projection);
  }

  public Optional<RepulsorSetpoint> pollChosenSetpoint() {
    var out = lastChosenSetpoint;
    lastChosenSetpoint = Optional.empty();
    return out;
  }
}
