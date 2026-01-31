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
import org.curtinfrc.frc2026.util.Repulsor.DriverStation.NtRepulsorDriverStation;
import org.curtinfrc.frc2026.util.Repulsor.DriverStation.RepulsorDriverStation;
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
  private static final double FORCE_THROUGH_GOAL_DIST = 2.0;
  private static final double FORCE_THROUGH_WALL_DIST = 0.7;
  private static final double CORNER_CHAMFER = 0;
  public static final double GOAL_STRENGTH = 2.2;
  private static final double STAGED_CENTER_BAND_M = 1.2;
  private static final double STAGED_REACH_DIST_M = 0.2;
  private static final double STAGED_RESTAGE_DIST_M = 1.2;
  private static final double STAGED_SAME_GOAL_POS_M = 0.05;
  private static final double STAGED_SAME_GOAL_ROT_DEG = 5.0;
  private static final double STAGED_MAX_SEC = 2.0;
  private GatedAttractorObstacle stagedGate = null;

  private static final class ClearMemo {
    Boolean toGoalDyn;
    Boolean toGoalNoDyn;

    boolean toGoalDyn(
        Translation2d a, Translation2d b, List<? extends Obstacle> dyn, double rx, double ry) {
      if (toGoalDyn != null) return toGoalDyn.booleanValue();
      toGoalDyn = ExtraPathing.isClearPath("Repulsor/IsClear", a, b, dyn, rx, ry, true);
      return toGoalDyn.booleanValue();
    }

    boolean toGoalNoDyn(Translation2d a, Translation2d b, double rx, double ry) {
      if (toGoalNoDyn != null) return toGoalNoDyn.booleanValue();
      toGoalNoDyn =
          ExtraPathing.isClearPath(
              "Repulsor/ForceThrough/NoDyn", a, b, Collections.emptyList(), rx, ry, false);
      return toGoalNoDyn.booleanValue();
    }
  }

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

  private Optional<RepulsorSetpoint> lastChosenSetpoint = Optional.empty();

  private final TurnTuning turnTuning;
  private final DriveTuning driveTuning;
  public final ReactiveBypass bypass = new ReactiveBypass();
  private final HeadingGate headingGate = new HeadingGate();

  private final ObstacleProvider obstacleProvider;
  private final List<Obstacle> fieldObstacles;
  private final List<Obstacle> walls;
  private final List<GatedAttractorObstacle> gatedAttractors = new ArrayList<>();

  public ObstacleProvider getObstacleProvider() {
    return obstacleProvider;
  }

  public FieldPlanner() {
    this(new DefaultTurnTuning(), new DefaultDriveTuning(), Constants.FIELD);
  }

  public FieldPlanner(ObstacleProvider obstacleProvider, DriveTuning driveTuning) {
    this(new DefaultTurnTuning(), driveTuning, obstacleProvider);
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
    for (Obstacle obs : this.fieldObstacles) {
      if (obs instanceof GatedAttractorObstacle gated) {
        if (gated.waypoint) {
          gatedAttractors.add(gated);
        }
      }
    }

    for (int i = 0; i < ARROWS_SIZE; i++) arrows.add(new Pose2d());
    String prefix = System.getenv("REACTIVE_BYPASS_ID");
    String logName;
    if (prefix != null && !prefix.isEmpty()) {
      logName = prefix + "ReactiveBypassLog.csv";
    } else {
      logName = "ReactiveBypassLog.csv";
    }
    // bypass.enableLogging(logName);
  }

  private static double clamp01(double x) {
    return Math.max(0.0, Math.min(1.0, x));
  }

  private static double smooth01(double x) {
    x = clamp01(x);
    return x * x * (3.0 - 2.0 * x);
  }

  private static boolean segmentIntersectsPolygonOuter(
      Translation2d a, Translation2d b, Translation2d[] poly) {
    if (isPointInPolygon(a, poly) || isPointInPolygon(b, poly)) return true;
    for (int i = 0; i < poly.length; i++) {
      Translation2d c = poly[i];
      Translation2d d = poly[(i + 1) % poly.length];
      if (segmentsIntersectOuter(a, b, c, d)) return true;
    }
    return false;
  }

  private static double orientOuter(Translation2d a, Translation2d b, Translation2d c) {
    return (b.getX() - a.getX()) * (c.getY() - a.getY())
        - (b.getY() - a.getY()) * (c.getX() - a.getX());
  }

  private static boolean onSegOuter(Translation2d a, Translation2d b, Translation2d p) {
    return p.getX() >= Math.min(a.getX(), b.getX()) - 1e-9
        && p.getX() <= Math.max(a.getX(), b.getX()) + 1e-9
        && p.getY() >= Math.min(a.getY(), b.getY()) - 1e-9
        && p.getY() <= Math.max(a.getY(), b.getY()) + 1e-9;
  }

  private static boolean segmentsIntersectOuter(
      Translation2d a, Translation2d b, Translation2d c, Translation2d d) {
    double o1 = orientOuter(a, b, c);
    double o2 = orientOuter(a, b, d);
    double o3 = orientOuter(c, d, a);
    double o4 = orientOuter(c, d, b);

    if ((o1 > 0) != (o2 > 0) && (o3 > 0) != (o4 > 0)) return true;

    if (Math.abs(o1) < 1e-9 && onSegOuter(a, b, c)) return true;
    if (Math.abs(o2) < 1e-9 && onSegOuter(a, b, d)) return true;
    if (Math.abs(o3) < 1e-9 && onSegOuter(c, d, a)) return true;
    if (Math.abs(o4) < 1e-9 && onSegOuter(c, d, b)) return true;

    return false;
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

  public static class CorridorCenterlineRail extends Obstacle {
    private final double xCenter;
    private final double xHalfWindow;
    private final double yCenter;
    private final double yHalfWidth;
    private final double maxForce;

    public CorridorCenterlineRail(
        double xCenter,
        double xHalfWindow,
        double yCenter,
        double yHalfWidth,
        double strength,
        double maxForce) {
      super(strength, true);
      this.xCenter = xCenter;
      this.xHalfWindow = Math.max(0.05, xHalfWindow);
      this.yCenter = yCenter;
      this.yHalfWidth = Math.max(0.10, yHalfWidth);
      this.maxForce = Math.max(0.1, maxForce);
    }

    private static double clamp01(double x) {
      return Math.max(0.0, Math.min(1.0, x));
    }

    private static double smooth01(double x) {
      x = clamp01(x);
      return x * x * (3.0 - 2.0 * x);
    }

    @Override
    public Force getForceAtPosition(Translation2d position, Translation2d target) {
      double dx = Math.abs(position.getX() - xCenter);
      if (dx >= xHalfWindow) return new Force();

      double wx = smooth01(1.0 - (dx / xHalfWindow));

      double ey = position.getY() - yCenter;

      double e = ey / yHalfWidth;

      double f = -strength * wx * (e) / (0.35 + e * e);

      if (f > maxForce) f = maxForce;
      if (f < -maxForce) f = -maxForce;

      Translation2d v = new Translation2d(0.0, f);
      double n = v.getNorm();
      if (n < 1e-9) return new Force();
      return new Force(n, v.getAngle());
    }
  }

  public static class RectangleObstacle extends Obstacle {
    public final Translation2d center;
    public final double halfX;
    public final double halfY;
    public final Rotation2d rot;
    public final double maxRangeX;
    public final double maxRangeY;
    private final boolean flowAssist;

    private static final double X_AXIS_ANGLE_BIAS_RAD = Math.toRadians(18.0);

    private static final double CORNER_RANGE_M = 1.15;
    private static final double CORNER_FORCE_SCALE = 14.0;
    private static final double CORNER_FORCE_SOFTEN = 0.07;

    private static final double CORNER_SLIDE_SCALE = 9.0;
    private static final double CORNER_SLIDE_SOFTEN = 0.22;

    private static final double CORNER_ORBIT_SCALE = 7.5;
    private static final double CORNER_ORBIT_SOFTEN = 0.18;

    private static final double TIE_EPS = 0.03;

    private static final double COMMIT_SEC_BASE = 0.65;
    private static final double COMMIT_SEC_CORNER = 0.90;
    private static final double COMMIT_CLEAR_HYST_M = 0.28;

    private static final double CLEARANCE_WEIGHT = 2.8;
    private static final double OCCLUDE_PENALTY = 0.55;
    private static final double PROGRESS_PENALTY_SCALE = 0.42;

    private static final double LOOK_T1 = 0.50;
    private static final double LOOK_T2 = 0.82;
    private static final double LOOK_T3 = 1.12;

    private static final double LOOK_O1 = 0.20;
    private static final double LOOK_O2 = 0.26;
    private static final double LOOK_O3 = 0.32;

    private static final double TUG_OPPOSE_FRAC = 0.22;
    private static final double TUG_MIN_OPPOSE = 0.18;
    private static final double TUG_PERP_SMALL_FRAC = 0.30;
    private static final double TUG_LATERAL_SCALE = 3.9;
    private static final double TUG_LATERAL_BASE = 0.55;

    private static final double EDGE_TEAR_RANGE_M = 1.55;
    private static final double EDGE_TEAR_END_TAPER_M = 0.42;
    private static final double EDGE_TEAR_CORNER_SUPPRESS = 0.22;
    private static final double EDGE_TEAR_BLEND = 0.70;
    private static final double EDGE_TEAR_CORNER_MIN = 0.30;

    private static final double EDGE_CONVEY_RANGE_M = 0.65;
    private static final double EDGE_CONVEY_SCALE = 4.6;
    private static final double EDGE_CONVEY_SOFTEN = 0.24;
    private static final double EDGE_CONVEY_MIN_FRAC = 0.40;
    private static final double EDGE_CONVEY_CORNER_SUPPRESS = 0.55;

    private static final double TEARDROP_EDGE_OFFSET_MIN = 0.18;
    private static final double TEARDROP_EDGE_OFFSET_EXTRA = 0.52;
    private static final double TEARDROP_EDGE_OFFSET_MAX = 0.50;
    private static final double TEARDROP_GOAL_BIAS = 0.65;
    private static final double TEARDROP_OUTWARD_DOT_MIN = 0.12;
    private static final double TEARDROP_SHORT_ALIGN_M = 0.10;

    private static final double CORNER_HANDOFF_WCORNER_ON = 0.28;
    private static final double CORNER_HANDOFF_WCORNER_FULL = 0.70;

    private static final double CORNER_HANDOFF_TEAR_SUPPRESS_MAX = 0.65;

    private static final double CORNER_HANDOFF_PUSH_SCALE = 2.1;
    private static final double CORNER_HANDOFF_TANGENT_SCALE = 3.6;
    private static final double CORNER_HANDOFF_SOFTEN = 0.26;

    private static final double CORNER_HANDOFF_GOAL_PAST_M = 0.10;
    private static final double CORNER_HANDOFF_CENTER_BAND_M = 0.55;

    private static final double HANDOFF_LOCK_SEC = 0.75;

    private static final double CORNER_LOCK_SEC = 0.95;
    private static final double CORNER_SWITCH_HYST_M = 0.10;

    private static final double DESIRED_EDGE_CLEAR_M = 0.24;
    private static final double DESIRED_CORNER_CLEAR_M = 0.30;

    private static final double CLEAR_PUSH_SCALE = 3.6;
    private static final double CLEAR_PUSH_SOFTEN = 0.20;

    private static final double CORNER_CLEAR_PUSH_SCALE = 4.4;
    private static final double CORNER_CLEAR_PUSH_SOFTEN = 0.22;

    private static final double CLEAR_PUSH_MAX = 10.5;

    private static final double CORNER_CONVEY_RANGE_M = 1.25;
    private static final double CORNER_CONVEY_WCORNER_ON = 0.08;
    private static final double CORNER_CONVEY_SCALE = 6.2;
    private static final double CORNER_CONVEY_SOFTEN = 0.28;
    private static final double CORNER_CONVEY_MIN_FRAC = 0.50;
    private static final double CORNER_CONVEY_MAX = 14.0;

    private int commitDir = 0;
    private double commitUntilSec = 0.0;
    private Translation2d commitCornerWorld = null;

    private int handoffShortSign = 0;
    private double handoffUntilSec = 0.0;

    private int cornerLockIdx = -1;
    private Translation2d cornerLockCornerWorld = null;
    private double cornerLockUntilSec = 0.0;

    private final FlowTeardrop tearA_CCW;
    private final FlowTeardrop tearA_CW;
    private final FlowTeardrop tearB_CCW;
    private final FlowTeardrop tearB_CW;
    private final boolean longAxisX;
    private final int shortSignA;
    private final int shortSignB;

    private final class FlowTeardrop {
      final Translation2d loc;
      final double primaryStrength;
      final double primaryMaxRange;
      final double primaryRadius;
      final double tailStrength;
      final double tailLength;
      final Rotation2d tailDirWorld;

      FlowTeardrop(
          Translation2d loc,
          Rotation2d tailDirWorld,
          double primaryStrength,
          double primaryMaxRange,
          double primaryRadius,
          double tailStrength,
          double tailLength) {
        this.loc = loc;
        this.tailDirWorld = (tailDirWorld == null) ? Rotation2d.kZero : tailDirWorld;
        this.primaryStrength = primaryStrength;
        this.primaryMaxRange = primaryMaxRange;
        this.primaryRadius = primaryRadius;
        this.tailStrength = tailStrength;
        this.tailLength = tailLength;
      }

      private double clamp01(double x) {
        return Math.max(0.0, Math.min(1.0, x));
      }

      private double interp(double a, double b, double t) {
        return a + (b - a) * t;
      }

      Translation2d forceVec(Translation2d position, Rotation2d tailDirOverride) {
        final double tiny = EPS;

        Translation2d posToLoc = position.minus(loc);
        double distPosLoc = posToLoc.getNorm();

        Translation2d outwardsForce = Translation2d.kZero;
        if (distPosLoc <= primaryMaxRange && distPosLoc >= tiny) {
          double magBase =
              distToForceMag(
                  Math.max(distPosLoc - primaryRadius, 0.0), primaryMaxRange - primaryRadius);
          double mag = primaryStrength * magBase;
          outwardsForce = posToLoc.div(distPosLoc).times(mag);
        }

        Rotation2d tailDir = (tailDirOverride == null) ? tailDirWorld : tailDirOverride;
        Translation2d positionRel = position.minus(loc).rotateBy(tailDir.unaryMinus());
        double distanceAlongLine = positionRel.getX();
        double distanceScalar =
            (Math.abs(tailLength) > tiny) ? (distanceAlongLine / tailLength) : 0.0;

        Translation2d tailForce = Translation2d.kZero;

        if (distanceScalar >= 0.0 && distanceScalar <= 1.0) {
          double secondaryMaxRange = interp(primaryMaxRange, 0.0, distanceScalar * distanceScalar);
          double distanceToLine = Math.abs(positionRel.getY());

          if (distanceToLine <= secondaryMaxRange && secondaryMaxRange > tiny) {
            double alongWeight;
            if (distanceAlongLine < primaryMaxRange) {
              alongWeight = distanceAlongLine / Math.max(primaryMaxRange, tiny);
            } else {
              double denom = Math.max(tailLength - primaryMaxRange, tiny);
              alongWeight = 1.0 - (distanceAlongLine - primaryMaxRange) / denom;
            }
            alongWeight = clamp01(alongWeight);

            double lateralWeight = 1.0 - (distanceToLine / Math.max(secondaryMaxRange, tiny));
            lateralWeight = clamp01(lateralWeight);

            double tailMag =
                tailStrength
                    * alongWeight
                    * (lateralWeight * lateralWeight)
                    * (secondaryMaxRange - distanceToLine);

            Translation2d tailU = new Translation2d(1.0, tailDir);
            tailForce = tailU.times(tailMag);
          }
        }

        return outwardsForce.plus(tailForce);
      }
    }

    private static double clamp01(double x) {
      return Math.max(0.0, Math.min(1.0, x));
    }

    private static double smooth01(double x) {
      x = clamp01(x);
      return x * x * (3.0 - 2.0 * x);
    }

    private static double remap01(double x, double a, double b) {
      if (b <= a) return (x >= b) ? 1.0 : 0.0;
      return clamp01((x - a) / (b - a));
    }

    private static Translation2d vecFromForce(Force f) {
      if (f == null) return Translation2d.kZero;
      double m = f.getNorm();
      if (m < EPS) return Translation2d.kZero;
      return new Translation2d(m, f.getAngle());
    }

    private static double wallMovePenalty(Translation2d pos, Translation2d cand) {
      double L = Constants.FIELD_LENGTH;
      double W = Constants.FIELD_WIDTH;

      double edge = 0.95;
      double dy = cand.getY() - pos.getY();
      double dx = cand.getX() - pos.getX();

      double pen = 0.0;

      if (pos.getY() > W - edge && dy > 0.02) pen += 1.6 * dy;
      if (pos.getY() < edge && dy < -0.02) pen += 1.6 * (-dy);

      if (pos.getX() > L - edge && dx > 0.02) pen += 1.6 * dx;
      if (pos.getX() < edge && dx < -0.02) pen += 1.6 * (-dx);

      return pen;
    }

    private static Translation2d closestPointOnSegment(
        Translation2d p, Translation2d a, Translation2d b) {
      double ax = a.getX(), ay = a.getY();
      double bx = b.getX(), by = b.getY();
      double px = p.getX(), py = p.getY();

      double vx = bx - ax;
      double vy = by - ay;
      double vv = vx * vx + vy * vy;
      if (vv < 1e-12) return a;

      double t = ((px - ax) * vx + (py - ay) * vy) / vv;
      t = Math.max(0.0, Math.min(1.0, t));
      return new Translation2d(ax + vx * t, ay + vy * t);
    }

    private static Translation2d edgeClearancePush(
        Translation2d pos,
        Translation2d[] poly,
        Translation2d fallbackAwayU,
        double desiredClearM,
        double strength,
        double scale,
        double soften) {

      double bestD = Double.POSITIVE_INFINITY;
      Translation2d bestCp = null;

      for (int i = 0; i < poly.length; i++) {
        Translation2d a = poly[i];
        Translation2d b = poly[(i + 1) % poly.length];
        Translation2d cp = closestPointOnSegment(pos, a, b);
        double d = pos.getDistance(cp);
        if (d < bestD) {
          bestD = d;
          bestCp = cp;
        }
      }

      if (bestCp == null || bestD >= desiredClearM) return Translation2d.kZero;

      Translation2d n = pos.minus(bestCp);
      double nN = n.getNorm();
      Translation2d nU;
      if (nN > EPS) nU = n.div(nN);
      else if (fallbackAwayU != null && fallbackAwayU.getNorm() > EPS) nU = fallbackAwayU;
      else nU = new Translation2d(1.0, 0.0);

      double t = smooth01(clamp01((desiredClearM - bestD) / Math.max(EPS, desiredClearM)));
      double k = (strength * scale) * t / (soften + bestD * bestD);

      Translation2d out = nU.times(k);
      double m = out.getNorm();
      if (m > CLEAR_PUSH_MAX) out = out.times(CLEAR_PUSH_MAX / Math.max(EPS, m));
      return out;
    }

    private static Translation2d cornerClearancePush(
        Translation2d pos,
        Translation2d corner,
        Translation2d fallbackAwayU,
        double desiredClearM,
        double strength,
        double scale,
        double soften) {

      double d = pos.getDistance(corner);
      if (d >= desiredClearM) return Translation2d.kZero;

      Translation2d n = pos.minus(corner);
      double nN = n.getNorm();
      Translation2d nU;
      if (nN > EPS) nU = n.div(nN);
      else if (fallbackAwayU != null && fallbackAwayU.getNorm() > EPS) nU = fallbackAwayU;
      else nU = new Translation2d(1.0, 0.0);

      double t = smooth01(clamp01((desiredClearM - d) / Math.max(EPS, desiredClearM)));
      double k = (strength * scale) * t / (soften + d * d);

      Translation2d out = nU.times(k);
      double m = out.getNorm();
      if (m > CLEAR_PUSH_MAX) out = out.times(CLEAR_PUSH_MAX / Math.max(EPS, m));
      return out;
    }

    private static Translation2d stablePick(
        Translation2d pos,
        Translation2d goal,
        Translation2d centerWorld,
        Translation2d tCW,
        Translation2d tCCW,
        Translation2d pCW,
        Translation2d pCCW,
        double sCW,
        double sCCW) {

      double diff = Math.abs(sCW - sCCW);
      if (diff > TIE_EPS) return (sCW <= sCCW) ? tCW : tCCW;

      double wCW = wallMovePenalty(pos, pCW);
      double wCCW = wallMovePenalty(pos, pCCW);
      if (Math.abs(wCW - wCCW) > 1e-6) return (wCW <= wCCW) ? tCW : tCCW;

      Translation2d toGoal = goal.minus(pos);
      double gN = toGoal.getNorm();
      Translation2d toGoalU = (gN > EPS) ? toGoal.div(gN) : new Translation2d(1.0, 0.0);

      double aCW = dot(tCW, toGoalU);
      double aCCW = dot(tCCW, toGoalU);
      if (Math.abs(aCW - aCCW) > 1e-6) return (aCW >= aCCW) ? tCW : tCCW;

      Translation2d cp = pos.minus(centerWorld);
      Translation2d cg = goal.minus(centerWorld);
      double cross = cp.getX() * cg.getY() - cp.getY() * cg.getX();
      return (cross >= 0.0) ? tCCW : tCW;
    }

    private boolean isCornerOrHandoffLocked() {
      double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
      return (handoffShortSign != 0 && now < handoffUntilSec)
          || (cornerLockIdx >= 0 && now < cornerLockUntilSec);
    }

    private void clearCommitIfNeeded(Translation2d pos, Translation2d goal) {
      if (commitDir == 0) return;

      if (isCornerOrHandoffLocked()) return;

      double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
      boolean nearCorner =
          (commitCornerWorld != null)
              && (pos.getDistance(commitCornerWorld) <= CORNER_RANGE_M + COMMIT_CLEAR_HYST_M);
      if (!nearCorner && now > commitUntilSec) {
        commitDir = 0;
        commitCornerWorld = null;
        return;
      }
      double pad = Math.max(0.55, Math.min(2.0, Math.max(maxRangeX, maxRangeY) * 0.85));
      Translation2d[] polyExp = expandedCorners(pad);
      if (!nearCorner && !segmentIntersectsPolygon(pos, goal, polyExp)) {
        commitDir = 0;
        commitCornerWorld = null;
        return;
      }
      if (!nearCorner && commitCornerWorld != null) {
        commitDir = 0;
        commitCornerWorld = null;
      }
    }

    private void setCommitDir(int dir, boolean corner, Translation2d cornerWorld) {
      if (dir == 0) return;
      double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
      commitDir = (dir > 0) ? 1 : -1;
      commitUntilSec = now + (corner ? COMMIT_SEC_CORNER : COMMIT_SEC_BASE);
      commitCornerWorld = cornerWorld;
    }

    private Translation2d commitLateralFromGoal(Translation2d goalU) {
      Translation2d left = new Translation2d(-goalU.getY(), goalU.getX());
      Translation2d right = new Translation2d(goalU.getY(), -goalU.getX());
      if (commitDir < 0) return right;
      if (commitDir > 0) return left;
      return left;
    }

    private int impliedDirFromGeometry(Translation2d pos, Translation2d goal) {
      Translation2d cp = pos.minus(center);
      Translation2d cg = goal.minus(center);
      double cross = cp.getX() * cg.getY() - cp.getY() * cg.getX();
      return (cross >= 0.0) ? 1 : -1;
    }

    private Translation2d applyAntiTug(
        Translation2d sum,
        Translation2d pos,
        Translation2d goal,
        double wCorner,
        boolean occluding,
        double edgeDist) {

      if (!occluding) return sum;
      double n = sum.getNorm();
      if (n < EPS) return sum;

      Translation2d g = goal.minus(pos);
      double gN = g.getNorm();
      if (gN < EPS) return sum;
      Translation2d gU = g.div(gN);

      double parDot = dot(sum, gU);
      Translation2d par = gU.times(parDot);
      Translation2d perp = sum.minus(par);

      double allowedOpp = Math.max(TUG_MIN_OPPOSE, TUG_OPPOSE_FRAC * n);
      if (parDot < -allowedOpp) parDot = -allowedOpp;

      double perpN = perp.getNorm();
      if (parDot < 0.0 && perpN < TUG_PERP_SMALL_FRAC * n) {
        if (commitDir == 0) setCommitDir(impliedDirFromGeometry(pos, goal), false, null);
        Translation2d latU = commitLateralFromGoal(gU);
        double need = clamp01((-parDot) / Math.max(EPS, allowedOpp));
        double d = Math.max(0.12, edgeDist);
        double lateralMag =
            (strength * TUG_LATERAL_SCALE) * wCorner * (TUG_LATERAL_BASE + need) / (0.35 + d * d);
        perp = perp.plus(latU.times(lateralMag));
      }

      return gU.times(parDot).plus(perp);
    }

    // NEW: guarantees a stable tangential "carry" around the locked/nearest corner (removes
    // back-and-forth inching)
    private Translation2d applyCornerConveyor(
        Translation2d sum,
        Translation2d position,
        Translation2d target,
        Translation2d cornerWorld,
        Translation2d awayWorldU,
        double wCorner,
        boolean occluding) {

      if (wCorner < CORNER_CONVEY_WCORNER_ON) return sum;

      double d = position.getDistance(cornerWorld);
      double wD = smooth01(1.0 - (d / Math.max(EPS, CORNER_CONVEY_RANGE_M)));
      if (wD < 1e-6) return sum;

      double wOcc = occluding ? 1.0 : 0.65;
      double w = wD * wOcc;

      Translation2d v = position.minus(cornerWorld);
      double vN = v.getNorm();
      if (vN < EPS) {
        if (awayWorldU != null && awayWorldU.getNorm() > EPS) v = awayWorldU;
        else v = new Translation2d(1.0, 0.0);
        vN = Math.max(EPS, v.getNorm());
      }
      Translation2d u = v.div(vN);

      int flowDir = (commitDir != 0) ? commitDir : impliedDirFromGeometry(position, target);
      boolean ccw = flowDir > 0;

      Translation2d tanU =
          ccw ? new Translation2d(-u.getY(), u.getX()) : new Translation2d(u.getY(), -u.getX());
      double tanN = tanU.getNorm();
      if (tanN < EPS) return sum;
      tanU = tanU.div(tanN);

      double k = (strength * CORNER_CONVEY_SCALE) * w / (CORNER_CONVEY_SOFTEN + d * d);
      if (k > CORNER_CONVEY_MAX) k = CORNER_CONVEY_MAX;

      double minAlong = k * CORNER_CONVEY_MIN_FRAC;

      double along = dot(sum, tanU);

      if (along < minAlong) {
        sum = sum.plus(tanU.times(minAlong - along));
      }

      return sum;
    }

    private Translation2d applyEdgeConveyor(
        Translation2d sum,
        Translation2d pLocal,
        Translation2d gLocal,
        double ax,
        double ay,
        double edgeDist,
        double wCorner) {

      if (edgeDist >= EDGE_CONVEY_RANGE_M) return sum;

      double wEdge = smooth01(1.0 - (edgeDist / Math.max(EPS, EDGE_CONVEY_RANGE_M)));
      double wNoCorner =
          smooth01(clamp01((EDGE_CONVEY_CORNER_SUPPRESS - wCorner) / EDGE_CONVEY_CORNER_SUPPRESS));
      double w = wEdge * wNoCorner;
      if (w < 1e-6) return sum;

      Translation2d tanLocal;
      if (ax >= ay) {
        double dir = signWithFallback(pLocal.getY(), 1.0);
        tanLocal = new Translation2d(0.0, dir);
      } else {
        double dir = signWithFallback(pLocal.getX(), 1.0);
        tanLocal = new Translation2d(dir, 0.0);
      }

      Translation2d tanWorld = tanLocal.rotateBy(rot);
      double tanN = tanWorld.getNorm();
      if (tanN < EPS) return sum;
      tanWorld = tanWorld.div(tanN);

      double k = (strength * EDGE_CONVEY_SCALE) * w / (EDGE_CONVEY_SOFTEN + edgeDist * edgeDist);
      double minAlong = k * EDGE_CONVEY_MIN_FRAC;

      double along = dot(sum, tanWorld);
      if (along < minAlong) {
        sum = sum.plus(tanWorld.times(minAlong - along));
      }

      return sum;
    }

    private int shortSignFromCornerLocal(Translation2d cornerLocal) {
      double s = longAxisX ? Math.signum(cornerLocal.getY()) : Math.signum(cornerLocal.getX());
      if (s == 0.0) s = 1.0;
      return (s >= 0.0) ? 1 : -1;
    }

    private static int signWithFallback(double primary, double fallback) {
      double s = Math.signum(primary);
      if (s == 0.0) s = Math.signum(fallback);
      if (s == 0.0) s = 1.0;
      return (s >= 0.0) ? 1 : -1;
    }

    private Rotation2d teardropTailDir(
        int goalSideSign,
        int teardropSideSign,
        Translation2d teardropLocWorld,
        Translation2d goalWorld,
        Translation2d gLocal,
        Translation2d pLocal) {
      if (goalSideSign != teardropSideSign) {
        return wallSideDir(teardropLocWorld);
      }

      double gLong = longAxisX ? gLocal.getX() : gLocal.getY();
      double pLong = longAxisX ? pLocal.getX() : pLocal.getY();
      int longSign;
      if (commitDir != 0) {
        if (longAxisX) {
          longSign = (commitDir > 0) ? -teardropSideSign : teardropSideSign;
        } else {
          longSign = (commitDir > 0) ? teardropSideSign : -teardropSideSign;
        }
      } else {
        longSign = signWithFallback(pLong, gLong);
      }
      Translation2d longAxisLocal =
          longAxisX ? new Translation2d(longSign, 0.0) : new Translation2d(0.0, longSign);
      Translation2d longAxisWorld = longAxisLocal.rotateBy(rot);
      double longAxisN = longAxisWorld.getNorm();
      Translation2d longAxisU =
          (longAxisN > EPS) ? longAxisWorld.div(longAxisN) : Translation2d.kZero;
      Rotation2d longAxisDir = (longAxisN > EPS) ? longAxisWorld.getAngle() : Rotation2d.kZero;

      Rotation2d baseDir;
      if (goalSideSign == teardropSideSign) {
        Translation2d toGoal = goalWorld.minus(teardropLocWorld);
        if (toGoal.getNorm() > EPS) baseDir = toGoal.getAngle();
        else baseDir = longAxisDir;
      } else {
        baseDir = longAxisDir;
      }

      Translation2d outwardLocal =
          longAxisX
              ? new Translation2d(0.0, teardropSideSign)
              : new Translation2d(teardropSideSign, 0.0);
      Translation2d outwardWorld = outwardLocal.rotateBy(rot);
      double outN = outwardWorld.getNorm();
      if (outN < EPS) return baseDir;
      Translation2d outwardU = outwardWorld.div(outN);

      Translation2d cornerU = outwardU;
      Translation2d cornerVec = outwardU.plus(longAxisU);
      double cornerN = cornerVec.getNorm();
      if (cornerN > EPS) cornerU = cornerVec.div(cornerN);

      Translation2d flowU = cornerU;
      double flowN = flowU.getNorm();
      if (flowN < EPS) {
        if (longAxisU.getNorm() > EPS) flowU = longAxisU;
        else flowU = outwardU;
      }

      if (flowU.getNorm() < EPS) return baseDir;
      return flowU.getAngle();
    }

    private static Rotation2d wallSideDir(Translation2d locWorld) {
      double y = locWorld.getY();
      double distLow = y;
      double distHigh = Constants.FIELD_WIDTH - y;
      double sign = (distLow <= distHigh) ? -1.0 : 1.0;
      Translation2d dir = new Translation2d(0.0, sign);
      return dir.getAngle();
    }

    private boolean goalPastShortSide(Translation2d pLocal, Translation2d gLocal, int shortSign) {
      if (shortSign == 0) return false;

      if (longAxisX) {
        boolean past =
            (shortSign >= 0)
                ? (gLocal.getY() > halfY + CORNER_HANDOFF_GOAL_PAST_M)
                : (gLocal.getY() < -halfY - CORNER_HANDOFF_GOAL_PAST_M);
        boolean band = Math.abs(gLocal.getX()) < (halfX + CORNER_HANDOFF_CENTER_BAND_M);
        return past && band;
      } else {
        boolean past =
            (shortSign >= 0)
                ? (gLocal.getX() > halfX + CORNER_HANDOFF_GOAL_PAST_M)
                : (gLocal.getX() < -halfX - CORNER_HANDOFF_GOAL_PAST_M);
        boolean band = Math.abs(gLocal.getY()) < (halfY + CORNER_HANDOFF_CENTER_BAND_M);
        return past && band;
      }
    }

    private void updateHandoffLock(double wCorner, boolean want, int shortSign) {
      double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
      boolean active = now < handoffUntilSec && handoffShortSign != 0;

      if (want) {
        if (!active) handoffShortSign = shortSign;
        handoffUntilSec = now + HANDOFF_LOCK_SEC;
        return;
      }

      if (active && wCorner < (CORNER_HANDOFF_WCORNER_ON * 0.65)) {
        handoffShortSign = 0;
        handoffUntilSec = 0.0;
      }
    }

    private void updateCornerLock(
        int bestIdx, double bestDist, double secondDist, double cornerBand) {
      double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
      boolean active = cornerLockIdx >= 0 && now < cornerLockUntilSec;

      if (!active) {
        if (bestDist < cornerBand * 0.98) {
          cornerLockIdx = bestIdx;
          cornerLockUntilSec = now + CORNER_LOCK_SEC;
        } else {
          cornerLockIdx = -1;
          cornerLockUntilSec = 0.0;
        }
        return;
      }

      if (bestIdx == cornerLockIdx) {
        cornerLockUntilSec = now + CORNER_LOCK_SEC;
        return;
      }

      if (bestDist + CORNER_SWITCH_HYST_M < secondDist) {
        cornerLockIdx = bestIdx;
        cornerLockUntilSec = now + CORNER_LOCK_SEC;
      } else {
        cornerLockUntilSec = now + CORNER_LOCK_SEC;
      }
    }

    private Translation2d cornerHandoffNudgeLocked(
        Translation2d position,
        Translation2d pLocal,
        Translation2d gLocal,
        Translation2d cornerWorld,
        double wCorner,
        int shortSign) {

      double t = smooth01(remap01(wCorner, CORNER_HANDOFF_WCORNER_ON, CORNER_HANDOFF_WCORNER_FULL));
      if (t < 1e-6) return Translation2d.kZero;
      if (shortSign == 0) return Translation2d.kZero;
      if (!goalPastShortSide(pLocal, gLocal, shortSign)) return Translation2d.kZero;

      Translation2d shortNormalLocal =
          longAxisX ? new Translation2d(0.0, shortSign) : new Translation2d(shortSign, 0.0);

      Translation2d toGoalLocal = gLocal.minus(pLocal);
      double dirLong;
      if (longAxisX) {
        dirLong = Math.signum(toGoalLocal.getX());
        if (dirLong == 0.0) dirLong = 1.0;
      } else {
        dirLong = Math.signum(toGoalLocal.getY());
        if (dirLong == 0.0) dirLong = 1.0;
      }

      Translation2d tanLocal =
          longAxisX ? new Translation2d(dirLong, 0.0) : new Translation2d(0.0, dirLong);

      Translation2d shortNormalWorld = shortNormalLocal.rotateBy(rot);
      Translation2d tanWorld = tanLocal.rotateBy(rot);

      double dCorner = Math.max(0.06, position.getDistance(cornerWorld));
      double k = (strength * t) / (CORNER_HANDOFF_SOFTEN + dCorner * dCorner);

      Translation2d push = shortNormalWorld.times(CORNER_HANDOFF_PUSH_SCALE * k);
      Translation2d slide = tanWorld.times(CORNER_HANDOFF_TANGENT_SCALE * k);

      Translation2d out = push.plus(slide);

      double cap = strength * 9.0;
      double n = out.getNorm();
      if (n > cap) out = out.times(cap / Math.max(EPS, n));

      return out;
    }

    private Translation2d chooseCornerSlideDir(
        Translation2d pos,
        Translation2d goal,
        Translation2d cornerWorld,
        Translation2d centerWorld,
        Translation2d[] polyExp) {

      if (commitDir != 0
          && commitCornerWorld != null
          && commitCornerWorld.getDistance(cornerWorld) < 0.20) {
        Translation2d outward = pos.minus(centerWorld);
        double outN = outward.getNorm();
        if (outN < EPS) outward = new Translation2d(1.0, 0.0);
        outward = outward.div(Math.max(EPS, outward.getNorm()));

        Translation2d v = pos.minus(cornerWorld);
        double vN = v.getNorm();
        if (vN < EPS) v = outward;
        v = v.div(Math.max(EPS, v.getNorm()));

        Translation2d tCCW = new Translation2d(-v.getY(), v.getX());
        Translation2d tCW = new Translation2d(v.getY(), -v.getX());
        return (commitDir < 0) ? tCW : tCCW;
      }

      Translation2d outward = pos.minus(centerWorld);
      double outN = outward.getNorm();
      if (outN < EPS) outward = new Translation2d(1.0, 0.0);
      outward = outward.div(Math.max(EPS, outward.getNorm()));

      Translation2d v = pos.minus(cornerWorld);
      double vN = v.getNorm();
      if (vN < EPS) v = outward;
      v = v.div(Math.max(EPS, v.getNorm()));

      Translation2d t1 = new Translation2d(-v.getY(), v.getX());
      Translation2d t2 = new Translation2d(v.getY(), -v.getX());

      double baseDist = pos.getDistance(goal);
      double baseClear = minSampleDistanceToPolygon(pos, goal, polyExp);

      double[] steps = new double[] {0.28, 0.42, 0.60};
      double[] outs = new double[] {0.08, 0.10, 0.12};

      double bestS1 = Double.POSITIVE_INFINITY;
      double bestS2 = Double.POSITIVE_INFINITY;
      Translation2d bestP1 = null;
      Translation2d bestP2 = null;

      for (int i = 0; i < steps.length; i++) {
        Translation2d p1 = pos.plus(t1.times(steps[i])).plus(outward.times(outs[i]));
        double s1 = scoreCandidatePolyWithWalls(pos, goal, p1, baseDist, baseClear, polyExp, true);
        if (s1 < bestS1) {
          bestS1 = s1;
          bestP1 = p1;
        }

        Translation2d p2 = pos.plus(t2.times(steps[i])).plus(outward.times(outs[i]));
        double s2 = scoreCandidatePolyWithWalls(pos, goal, p2, baseDist, baseClear, polyExp, true);
        if (s2 < bestS2) {
          bestS2 = s2;
          bestP2 = p2;
        }
      }

      Translation2d tCW = t2;
      Translation2d tCCW = t1;
      Translation2d pCW = bestP2;
      Translation2d pCCW = bestP1;
      double sCW = bestS2;
      double sCCW = bestS1;

      Translation2d pick = stablePick(pos, goal, centerWorld, tCW, tCCW, pCW, pCCW, sCW, sCCW);
      int dir = (pick == tCW) ? -1 : 1;
      setCommitDir(dir, true, cornerWorld);
      return pick;
    }

    private Translation2d chooseCornerOrbitDir(
        Translation2d pos,
        Translation2d goal,
        Translation2d cornerWorld,
        Translation2d centerWorld,
        Translation2d[] polyExp) {

      if (commitDir != 0
          && commitCornerWorld != null
          && commitCornerWorld.getDistance(cornerWorld) < 0.20) {
        Translation2d outward = pos.minus(centerWorld);
        double outN = outward.getNorm();
        if (outN < EPS) outward = new Translation2d(1.0, 0.0);
        outward = outward.div(Math.max(EPS, outward.getNorm()));

        Translation2d v = pos.minus(cornerWorld);
        double vN = v.getNorm();
        if (vN < EPS) v = outward;
        v = v.div(Math.max(EPS, v.getNorm()));

        Translation2d tCCW = new Translation2d(-v.getY(), v.getX());
        Translation2d tCW = new Translation2d(v.getY(), -v.getX());
        return (commitDir < 0) ? tCW : tCCW;
      }

      Translation2d outward = pos.minus(centerWorld);
      double outN = outward.getNorm();
      if (outN < EPS) outward = new Translation2d(1.0, 0.0);
      outward = outward.div(Math.max(EPS, outward.getNorm()));

      Translation2d v = pos.minus(cornerWorld);
      double vN = v.getNorm();
      if (vN < EPS) v = outward;
      v = v.div(Math.max(EPS, v.getNorm()));

      Translation2d tCCW = new Translation2d(-v.getY(), v.getX());
      Translation2d tCW = new Translation2d(v.getY(), -v.getX());

      double baseDist = pos.getDistance(goal);
      double baseClear = minSampleDistanceToPolygon(pos, goal, polyExp);

      double[] stepT = new double[] {0.48, 0.72, 0.96};
      double[] stepO = new double[] {0.16, 0.22, 0.28};

      Translation2d bestPCW = null;
      Translation2d bestPCCW = null;
      double bestSCW = Double.POSITIVE_INFINITY;
      double bestSCCW = Double.POSITIVE_INFINITY;

      for (int i = 0; i < stepT.length; i++) {
        Translation2d pCW = pos.plus(tCW.times(stepT[i])).plus(outward.times(stepO[i]));
        double sCW =
            scoreCandidatePolyWithWalls(pos, goal, pCW, baseDist, baseClear, polyExp, true);
        if (sCW < bestSCW) {
          bestSCW = sCW;
          bestPCW = pCW;
        }

        Translation2d pCCW = pos.plus(tCCW.times(stepT[i])).plus(outward.times(stepO[i]));
        double sCCW =
            scoreCandidatePolyWithWalls(pos, goal, pCCW, baseDist, baseClear, polyExp, true);
        if (sCCW < bestSCCW) {
          bestSCCW = sCCW;
          bestPCCW = pCCW;
        }
      }

      Translation2d pick =
          stablePick(pos, goal, centerWorld, tCW, tCCW, bestPCW, bestPCCW, bestSCW, bestSCCW);
      int dir = (pick == tCW) ? -1 : 1;
      setCommitDir(dir, true, cornerWorld);
      return pick;
    }

    private double edgeTeardropWeight(Translation2d pLocal, double wCorner) {
      double longCoord = longAxisX ? pLocal.getX() : pLocal.getY();
      double edgeDist =
          longAxisX
              ? Math.abs(Math.abs(pLocal.getY()) - halfY)
              : Math.abs(Math.abs(pLocal.getX()) - halfX);

      double wD = smooth01(1.0 - (edgeDist / Math.max(EPS, EDGE_TEAR_RANGE_M)));
      if (wD < 1e-6) return 0.0;

      double longHalf = longAxisX ? halfX : halfY;
      double end = longHalf - Math.abs(longCoord);
      double wEnd = smooth01(clamp01(end / Math.max(EPS, EDGE_TEAR_END_TAPER_M)));

      double wNoCorner =
          smooth01(clamp01((EDGE_TEAR_CORNER_SUPPRESS - wCorner) / EDGE_TEAR_CORNER_SUPPRESS));
      double cornerBlend = EDGE_TEAR_CORNER_MIN + (1.0 - EDGE_TEAR_CORNER_MIN) * wNoCorner;

      return wD * wEnd * cornerBlend;
    }

    public RectangleObstacle(
        Translation2d center,
        double widthMeters,
        double heightMeters,
        Rotation2d rot,
        double strength,
        double maxRangeX,
        double maxRangeY,
        boolean flowAssist) {
      super(strength, true);
      this.center = center;
      this.halfX = Math.max(0.0, widthMeters * 0.5);
      this.halfY = Math.max(0.0, heightMeters * 0.5);
      this.rot = (rot == null) ? Rotation2d.kZero : rot;
      this.maxRangeX = Math.max(0.0, maxRangeX);
      this.maxRangeY = Math.max(0.0, maxRangeY);
      this.flowAssist = flowAssist;

      this.longAxisX = this.halfX >= this.halfY;

      double longHalf = longAxisX ? this.halfX : this.halfY;
      double shortHalf = longAxisX ? this.halfY : this.halfX;

      double maxR = Math.max(this.maxRangeX, this.maxRangeY);
      double primaryMaxRange = Math.max(0.20, Math.min(0.75, maxR + 0.10));
      double primaryRadius = Math.max(0.08, Math.min(0.30, shortHalf * 0.55 + 0.05));
      double tailLenParam = Math.max(0.55, longHalf * 1.15);

      double primaryStrength = strength * 0.20;
      double tailStrength = strength * 0.85;

      double edgeOffset =
          Math.max(
              TEARDROP_EDGE_OFFSET_MIN,
              Math.min(TEARDROP_EDGE_OFFSET_MAX, primaryRadius + TEARDROP_EDGE_OFFSET_EXTRA));

      Translation2d locALocal =
          longAxisX
              ? new Translation2d(0.0, this.halfY + edgeOffset)
              : new Translation2d(this.halfX + edgeOffset, 0.0);
      Translation2d locBLocal =
          longAxisX
              ? new Translation2d(0.0, -this.halfY - edgeOffset)
              : new Translation2d(-this.halfX - edgeOffset, 0.0);

      Translation2d locAWorld = locALocal.rotateBy(this.rot).plus(this.center);
      Translation2d locBWorld = locBLocal.rotateBy(this.rot).plus(this.center);

      Translation2d tA_CCW_L, tA_CW_L, tB_CCW_L, tB_CW_L;

      if (longAxisX) {
        tA_CCW_L = new Translation2d(-1.0, 0.0);
        tA_CW_L = new Translation2d(1.0, 0.0);
        tB_CCW_L = new Translation2d(1.0, 0.0);
        tB_CW_L = new Translation2d(-1.0, 0.0);
      } else {
        tA_CCW_L = new Translation2d(0.0, 1.0);
        tA_CW_L = new Translation2d(0.0, -1.0);
        tB_CCW_L = new Translation2d(0.0, -1.0);
        tB_CW_L = new Translation2d(0.0, 1.0);
      }

      Rotation2d dirA_CCW_W = tA_CCW_L.rotateBy(this.rot).getAngle();
      Rotation2d dirA_CW_W = tA_CW_L.rotateBy(this.rot).getAngle();
      Rotation2d dirB_CCW_W = tB_CCW_L.rotateBy(this.rot).getAngle();
      Rotation2d dirB_CW_W = tB_CW_L.rotateBy(this.rot).getAngle();

      this.tearA_CCW =
          new FlowTeardrop(
              locAWorld,
              dirA_CCW_W,
              primaryStrength,
              primaryMaxRange,
              primaryRadius,
              tailStrength,
              tailLenParam);
      this.tearA_CW =
          new FlowTeardrop(
              locAWorld,
              dirA_CW_W,
              primaryStrength,
              primaryMaxRange,
              primaryRadius,
              tailStrength,
              tailLenParam);
      this.tearB_CCW =
          new FlowTeardrop(
              locBWorld,
              dirB_CCW_W,
              primaryStrength,
              primaryMaxRange,
              primaryRadius,
              tailStrength,
              tailLenParam);
      this.tearB_CW =
          new FlowTeardrop(
              locBWorld,
              dirB_CW_W,
              primaryStrength,
              primaryMaxRange,
              primaryRadius,
              tailStrength,
              tailLenParam);

      double aShort = longAxisX ? locALocal.getY() : locALocal.getX();
      double bShort = longAxisX ? locBLocal.getY() : locBLocal.getX();
      this.shortSignA = (aShort >= 0.0) ? 1 : -1;
      this.shortSignB = (bShort >= 0.0) ? 1 : -1;
    }

    public RectangleObstacle(
        Translation2d center,
        double widthMeters,
        double heightMeters,
        Rotation2d rot,
        double strength,
        double maxRangeX,
        double maxRangeY) {
      this(center, widthMeters, heightMeters, rot, strength, maxRangeX, maxRangeY, true);
    }

    public RectangleObstacle(
        Translation2d center,
        double widthMeters,
        double heightMeters,
        double strength,
        double maxRangeX,
        double maxRangeY) {
      this(
          center,
          widthMeters,
          heightMeters,
          Rotation2d.kZero,
          strength,
          maxRangeX,
          maxRangeY,
          true);
    }

    public static RectangleObstacle simple(
        Translation2d center,
        double widthMeters,
        double heightMeters,
        double strength,
        double maxRangeX,
        double maxRangeY) {
      return new RectangleObstacle(
          center,
          widthMeters,
          heightMeters,
          Rotation2d.kZero,
          strength,
          maxRangeX,
          maxRangeY,
          false);
    }

    @Override
    public Force getForceAtPosition(Translation2d position, Translation2d target) {
      clearCommitIfNeeded(position, target);

      Translation2d pLocal = position.minus(center).rotateBy(rot.unaryMinus());
      Translation2d gLocal = target.minus(center).rotateBy(rot.unaryMinus());

      double px = pLocal.getX();
      double py = pLocal.getY();

      double clx = Math.max(-halfX, Math.min(px, halfX));
      double cly = Math.max(-halfY, Math.min(py, halfY));

      double dx = px - clx;
      double dy = py - cly;

      boolean inside = (px >= -halfX && px <= halfX && py >= -halfY && py <= halfY);

      double ax = inside ? 0.0 : Math.abs(dx);
      double ay = inside ? 0.0 : Math.abs(dy);
      double edgeDist = Math.hypot(ax, ay);

      Translation2d[] poly = corners();
      boolean occludes = segmentIntersectsPolygon(position, target, poly);

      if (!inside && (ax > maxRangeX || ay > maxRangeY) && !occludes) return new Force();

      double shortHalf = Math.min(halfX, halfY);
      double falloffMeters = Math.max(EPS, Math.max(maxRangeX, maxRangeY) + shortHalf + 0.35);

      Translation2d awayLocal;
      double effectiveDistMeters;

      if (!inside) {
        awayLocal = new Translation2d(dx, dy);
        double n = awayLocal.getNorm();
        if (n < EPS) return new Force();
        if (Math.abs(dx) > Math.abs(dy)) {
          double sign = Math.signum(target.getY() - position.getY());
          if (sign == 0.0) sign = 1.0;
          Rotation2d biasedAngle =
              awayLocal.getAngle().rotateBy(Rotation2d.fromRadians(sign * X_AXIS_ANGLE_BIAS_RAD));
          awayLocal = new Translation2d(n, biasedAngle);
        }
        effectiveDistMeters = Math.hypot(ax, ay);
        if (effectiveDistMeters < EPS) effectiveDistMeters = EPS;
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
        effectiveDistMeters = -Math.max(EPS, min);
      }

      double mag = distToForceMag(effectiveDistMeters, falloffMeters);

      Translation2d primaryWorld = Translation2d.kZero;
      Translation2d awayWorldU = Translation2d.kZero;
      if (Math.abs(mag) >= EPS && awayLocal.getNorm() >= EPS) {
        Translation2d awayWorld = awayLocal.rotateBy(rot);
        double n = Math.max(EPS, awayWorld.getNorm());
        awayWorldU = awayWorld.div(n);
        primaryWorld = awayWorldU.times(Math.abs(mag));
      }

      if (!flowAssist) {
        Translation2d sum = primaryWorld;

        boolean veryClose = (edgeDist < 0.35) || inside;
        if (veryClose) {
          sum =
              sum.plus(
                  edgeClearancePush(
                      position, poly, awayWorldU, DESIRED_EDGE_CLEAR_M, strength, 1.4, 0.28));
        }

        double n = sum.getNorm();
        if (n < EPS) return new Force();
        return new Force(n, sum.getAngle());
      }

      Translation2d escapeWorld = Translation2d.kZero;

      double engageR = Math.max(0.9, falloffMeters + 0.65);

      boolean nearObstacle = edgeDist <= Math.max(1.25, falloffMeters + 0.95);

      double padForTug = Math.max(0.55, Math.min(2.0, Math.max(maxRangeX, maxRangeY) * 0.85));
      Translation2d[] polyExpForTug = expandedCorners(padForTug);
      boolean occludesExpForTug = segmentIntersectsPolygon(position, target, polyExpForTug);

      if (edgeDist <= engageR) {
        Translation2d outwardW = position.minus(center);
        double outN = outwardW.getNorm();
        if (outN < EPS) outwardW = new Translation2d(1.0, 0.0);
        outN = Math.max(EPS, outwardW.getNorm());
        Translation2d outwardWU = outwardW.div(outN);

        Translation2d toGoal = target.minus(position);
        double gN = Math.max(EPS, toGoal.getNorm());
        Translation2d toGoalU = toGoal.div(gN);

        Translation2d tCCW = new Translation2d(-outwardWU.getY(), outwardWU.getX());
        Translation2d tCW = new Translation2d(outwardWU.getY(), -outwardWU.getX());

        Translation2d chosenT =
            chooseTangentPolyWithWalls(position, target, outwardWU, tCW, tCCW, polyExpForTug);

        double d = Math.max(0.12, edgeDist);
        double w = smooth01(1.0 - (d / engageR));

        double swirlMag = (strength * 6.8) / (0.30 + d * d);
        double slideMag = (strength * 3.6) / (0.35 + d * d);
        double pushOutMag = (strength * 2.1) / (0.55 + d * d);

        double along = dot(chosenT, toGoalU);
        double boost = (along < 0.12) ? 1.45 : 1.0;

        Translation2d swirl = chosenT.times(swirlMag * w * boost);
        Translation2d slide = chosenT.times(slideMag * w);
        Translation2d pushOut = outwardWU.times(pushOutMag * w);

        Translation2d add = swirl.plus(slide).plus(pushOut);

        if (occludes || occludesExpForTug) {
          escapeWorld = escapeWorld.plus(add);
        } else {
          Translation2d sumTry = primaryWorld.plus(add);
          if (sumTry.getNorm() < 1e-6 || along < 0.03) escapeWorld = escapeWorld.plus(add);
        }

        if (commitDir == 0 && (occludes || occludesExpForTug || nearObstacle)) {
          int dir = (chosenT == tCW) ? -1 : 1;
          setCommitDir(dir, false, null);
        }
      }

      Translation2d[] c = poly;

      double[] dC =
          new double[] {
            position.getDistance(c[0]),
            position.getDistance(c[1]),
            position.getDistance(c[2]),
            position.getDistance(c[3])
          };

      int bestIdx = 0;
      int secondIdx = 1;
      if (dC[1] < dC[0]) {
        bestIdx = 1;
        secondIdx = 0;
      }
      for (int i = 2; i < 4; i++) {
        if (dC[i] < dC[bestIdx]) {
          secondIdx = bestIdx;
          bestIdx = i;
        } else if (dC[i] < dC[secondIdx]) {
          secondIdx = i;
        }
      }

      double dMin = dC[bestIdx];
      double dSecond = dC[secondIdx];

      double cornerBand = CORNER_RANGE_M * 1.35;
      double wCorner = smooth01(1.0 - (dMin / cornerBand));

      updateCornerLock(bestIdx, dMin, dSecond, cornerBand);

      double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
      boolean cornerLocked = (cornerLockIdx >= 0 && now < cornerLockUntilSec);
      int useIdx = cornerLocked ? cornerLockIdx : bestIdx;
      Translation2d cornerWorld = c[useIdx];

      if (cornerLocked) cornerLockCornerWorld = cornerWorld;

      Translation2d cornerBoost = Translation2d.kZero;
      if (wCorner > 1e-6) {
        cornerBoost =
            computeCornerBoostFor(
                position,
                target,
                cornerWorld,
                center,
                polyExpForTug,
                awayWorldU,
                position.getDistance(cornerWorld),
                wCorner);
      }

      Translation2d cornerLocalUse = cornerWorld.minus(center).rotateBy(rot.unaryMinus());
      int shortSign = shortSignFromCornerLocal(cornerLocalUse);

      boolean wantHandoff =
          wCorner > CORNER_HANDOFF_WCORNER_ON && goalPastShortSide(pLocal, gLocal, shortSign);
      updateHandoffLock(wCorner, wantHandoff, shortSign);

      boolean handoffActive = (handoffShortSign != 0 && now < handoffUntilSec);

      Translation2d tearVec = Translation2d.kZero;
      Translation2d handoff = Translation2d.kZero;

      boolean edgeCase =
          !inside
              && nearObstacle
              && (occludes || occludesExpForTug || wCorner > CORNER_CONVEY_WCORNER_ON);
      if (edgeCase && !handoffActive) {
        double wEdge = edgeTeardropWeight(pLocal, wCorner);
        if (wEdge > 1e-6) {
          double gShort = longAxisX ? gLocal.getY() : gLocal.getX();
          double pShort = longAxisX ? pLocal.getY() : pLocal.getX();
          int goalSide = signWithFallback(gShort, pShort);
          int teardropSide = -goalSide;

          FlowTeardrop tear = (teardropSide == shortSignA) ? tearA_CCW : tearB_CCW;
          Rotation2d dir =
              teardropTailDir(goalSide, teardropSide, tear.loc, target, gLocal, pLocal);
          Translation2d v = tear.forceVec(position, dir);
          tearVec = v.times(EDGE_TEAR_BLEND * wEdge);
        }
      }

      if (handoffActive) {
        double handT =
            smooth01(remap01(wCorner, CORNER_HANDOFF_WCORNER_ON, CORNER_HANDOFF_WCORNER_FULL));
        double sup = CORNER_HANDOFF_TEAR_SUPPRESS_MAX * handT;
        tearVec = tearVec.times(1.0 - sup);
        handoff =
            cornerHandoffNudgeLocked(
                position, pLocal, gLocal, cornerWorld, wCorner, handoffShortSign);

        if (commitDir == 0)
          setCommitDir(impliedDirFromGeometry(position, target), true, cornerWorld);
      }

      Translation2d clearPush = Translation2d.kZero;

      boolean enforceClear = nearObstacle && (occludes || occludesExpForTug || wCorner > 0.03);

      if (enforceClear) {
        clearPush =
            clearPush.plus(
                edgeClearancePush(
                    position,
                    poly,
                    awayWorldU,
                    DESIRED_EDGE_CLEAR_M,
                    strength,
                    CLEAR_PUSH_SCALE,
                    CLEAR_PUSH_SOFTEN));
      }

      if (enforceClear && wCorner > 0.06) {
        clearPush =
            clearPush.plus(
                cornerClearancePush(
                    position,
                    cornerWorld,
                    awayWorldU,
                    DESIRED_CORNER_CLEAR_M,
                    strength,
                    CORNER_CLEAR_PUSH_SCALE,
                    CORNER_CLEAR_PUSH_SOFTEN));
      }

      Translation2d sum =
          primaryWorld
              // .plus(escapeWorld)
              .plus(cornerBoost)
              .plus(tearVec)
              .plus(handoff)
              .plus(clearPush);

      sum = applyEdgeConveyor(sum, pLocal, gLocal, ax, ay, edgeDist, wCorner);

      // NEW: apply stable corner conveyor BEFORE anti-tug to prevent oscillatory inching
      boolean cornerConvey = (enforceClear || nearObstacle) && wCorner > CORNER_CONVEY_WCORNER_ON;
      if (cornerConvey) {
        sum =
            applyCornerConveyor(
                sum,
                position,
                target,
                cornerWorld,
                awayWorldU,
                wCorner,
                (occludes || occludesExpForTug));
      }

      boolean tugCase = nearObstacle && wCorner > 0.04 && (occludes || occludesExpForTug);
      if (tugCase) {
        sum = applyAntiTug(sum, position, target, wCorner, true, edgeDist);
      }

      double sumN = sum.getNorm();
      Translation2d g = target.minus(position);
      double gN = g.getNorm();
      if (gN > EPS) {
        Translation2d gU = g.div(gN);
        double engageR2 = Math.max(1.25, falloffMeters + 0.95);
        boolean cornerOrNear = (wCorner > 0.02) || (edgeDist <= engageR2);

        if (cornerOrNear) {
          double pushScale = 1.0;
          if (occludes || occludesExpForTug) {
            pushScale = smooth01(clamp01((wCorner - 0.18) / Math.max(EPS, 0.32)));
          }
          if (pushScale > 1e-6) {
            if (sumN > EPS) {
              double align = dot(sum.div(sumN), gU);
              if (align < 0.25) {
                double d = Math.max(0.18, edgeDist);
                double pushThroughMag = (strength * 2.6) / (0.85 + d * d);
                sum = sum.plus(gU.times(pushThroughMag * pushScale * (0.25 - align)));
                sumN = sum.getNorm();
              }
            } else {
              double d = Math.max(0.18, edgeDist);
              double pushThroughMag = (strength * 2.6) / (0.85 + d * d);
              sum = sum.plus(gU.times(pushThroughMag * pushScale));
              sumN = sum.getNorm();
            }
          }
        }
      }

      if (sumN < EPS) return new Force();
      return new Force(sumN, sum.getAngle());
    }

    private Translation2d chooseTangentPolyWithWalls(
        Translation2d pos,
        Translation2d goal,
        Translation2d outwardU,
        Translation2d tCW,
        Translation2d tCCW,
        Translation2d[] polyExp) {

      if (commitDir != 0) return (commitDir < 0) ? tCW : tCCW;

      double baseDist = pos.getDistance(goal);
      double baseClear = minSampleDistanceToPolygon(pos, goal, polyExp);

      double[] stepT = new double[] {LOOK_T1, LOOK_T2, LOOK_T3};
      double[] stepO = new double[] {LOOK_O1, LOOK_O2, LOOK_O3};

      Translation2d bestPCW = null;
      Translation2d bestPCCW = null;
      double bestSCW = Double.POSITIVE_INFINITY;
      double bestSCCW = Double.POSITIVE_INFINITY;

      for (int i = 0; i < stepT.length; i++) {
        Translation2d pCW = pos.plus(tCW.times(stepT[i])).plus(outwardU.times(stepO[i]));
        double sCW =
            scoreCandidatePolyWithWalls(pos, goal, pCW, baseDist, baseClear, polyExp, true);
        if (sCW < bestSCW) {
          bestSCW = sCW;
          bestPCW = pCW;
        }

        Translation2d pCCW = pos.plus(tCCW.times(stepT[i])).plus(outwardU.times(stepO[i]));
        double sCCW =
            scoreCandidatePolyWithWalls(pos, goal, pCCW, baseDist, baseClear, polyExp, true);
        if (sCCW < bestSCCW) {
          bestSCCW = sCCW;
          bestPCCW = pCCW;
        }
      }

      Translation2d pick =
          stablePick(pos, goal, center, tCW, tCCW, bestPCW, bestPCCW, bestSCW, bestSCCW);
      int dir = (pick == tCW) ? -1 : 1;
      setCommitDir(dir, false, null);
      return pick;
    }

    private Translation2d computeCornerBoostFor(
        Translation2d position,
        Translation2d target,
        Translation2d corner,
        Translation2d centerWorld,
        Translation2d[] polyExpCorner,
        Translation2d awayWorldU,
        double dCorner,
        double wCorner) {

      Translation2d away = position.minus(corner);
      double n = away.getNorm();
      if (n < EPS) {
        Translation2d outward = position.minus(centerWorld);
        double on = outward.getNorm();
        if (on < EPS) outward = new Translation2d(1.0, 0.0);
        away = outward.div(Math.max(EPS, outward.getNorm()));
        n = 1.0;
      }
      Translation2d awayU = away.div(Math.max(EPS, n));

      Translation2d toGoal = target.minus(position);
      double gN = Math.max(EPS, toGoal.getNorm());
      Translation2d toGoalU = toGoal.div(gN);

      double w = smooth01(1.0 - (dCorner / Math.max(EPS, CORNER_RANGE_M)));
      double w2 = w * w;

      double mPush =
          (strength * CORNER_FORCE_SCALE) * w2 / (CORNER_FORCE_SOFTEN + dCorner * dCorner);
      Translation2d push = awayU.times(mPush);

      Translation2d slideDir =
          chooseCornerSlideDir(position, target, corner, centerWorld, polyExpCorner);
      double mSlide =
          (strength * CORNER_SLIDE_SCALE) * w2 / (CORNER_SLIDE_SOFTEN + dCorner * dCorner);
      Translation2d slide = slideDir.times(mSlide);

      double alignAway = 0.0;
      if (awayWorldU.getNorm() > EPS) alignAway = dot(awayWorldU, toGoalU);
      double needOrbit = clamp01((0.42 - alignAway) / 0.42);

      Translation2d orbitDir =
          chooseCornerOrbitDir(position, target, corner, centerWorld, polyExpCorner);
      double mOrbit =
          (strength * CORNER_ORBIT_SCALE)
              * w2
              * needOrbit
              / (CORNER_ORBIT_SOFTEN + dCorner * dCorner);
      Translation2d orbit = orbitDir.times(mOrbit);

      Translation2d outwardFromCenter = position.minus(centerWorld);
      double ocn = outwardFromCenter.getNorm();
      if (ocn < EPS) outwardFromCenter = new Translation2d(1.0, 0.0);
      outwardFromCenter = outwardFromCenter.div(Math.max(EPS, outwardFromCenter.getNorm()));
      Translation2d unstickOut =
          outwardFromCenter.times((strength * 1.6) * w2 / (0.35 + dCorner * dCorner));

      Translation2d boost = push.plus(slide).plus(orbit).plus(unstickOut);
      return boost.times(wCorner);
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

    private static double scoreCandidatePolyWithWalls(
        Translation2d pos,
        Translation2d goal,
        Translation2d cand,
        double baseDist,
        double baseClear,
        Translation2d[] polyExp,
        boolean allowNonProgress) {

      double d = cand.getDistance(goal);

      boolean stillOccluding = segmentIntersectsPolygon(cand, goal, polyExp);

      double candClear = minSampleDistanceToPolygon(cand, goal, polyExp);
      double clearanceCost = -CLEARANCE_WEIGHT * (candClear - baseClear);

      double occPenalty =
          stillOccluding ? (OCCLUDE_PENALTY + 0.65 * clamp01((0.18 - candClear) / 0.18)) : 0.0;

      double progressPenalty = 0.0;
      if (!allowNonProgress) {
        double dp = d - baseDist;
        double tt = clamp01((dp + 0.02) / 0.12);
        progressPenalty = PROGRESS_PENALTY_SCALE * smooth01(tt);
      }

      double x = cand.getX();
      double y = cand.getY();
      double dxW = Math.min(x, Constants.FIELD_LENGTH - x);
      double dyW = Math.min(y, Constants.FIELD_WIDTH - y);
      double wall = Math.min(dxW, dyW);
      double wallPenalty = (wall < 0.70) ? (1.05 * (0.70 - wall) / 0.70) : 0.0;

      double wallMove = wallMovePenalty(pos, cand);

      return d + clearanceCost + occPenalty + progressPenalty + wallPenalty + wallMove;
    }

    private static double minSampleDistanceToPolygon(
        Translation2d a, Translation2d b, Translation2d[] poly) {
      int n = 7;
      double best = Double.POSITIVE_INFINITY;
      for (int i = 0; i <= n; i++) {
        double t = (double) i / (double) n;
        Translation2d p = lerp(a, b, t);
        double d = pointDistanceToPolygonEdges(p, poly);
        if (FieldPlanner.isPointInPolygon(p, poly)) d = -d;
        if (d < best) best = d;
      }
      return best;
    }

    private static Translation2d lerp(Translation2d a, Translation2d b, double t) {
      return new Translation2d(
          a.getX() + (b.getX() - a.getX()) * t, a.getY() + (b.getY() - a.getY()) * t);
    }

    private static double pointDistanceToPolygonEdges(Translation2d p, Translation2d[] poly) {
      double best = Double.POSITIVE_INFINITY;
      for (int i = 0; i < poly.length; i++) {
        Translation2d a = poly[i];
        Translation2d b = poly[(i + 1) % poly.length];
        double d = pointSegmentDist(p, a, b);
        if (d < best) best = d;
      }
      return best;
    }

    private static double pointSegmentDist(Translation2d p, Translation2d a, Translation2d b) {
      double ax = a.getX();
      double ay = a.getY();
      double bx = b.getX();
      double by = b.getY();
      double px = p.getX();
      double py = p.getY();

      double vx = bx - ax;
      double vy = by - ay;
      double wx = px - ax;
      double wy = py - ay;

      double vv = vx * vx + vy * vy;
      if (vv < 1e-12) {
        double dx = px - ax;
        double dy = py - ay;
        return Math.hypot(dx, dy);
      }

      double t = (wx * vx + wy * vy) / vv;
      t = Math.max(0.0, Math.min(1.0, t));
      double cx = ax + vx * t;
      double cy = ay + vy * t;
      return Math.hypot(px - cx, py - cy);
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

  public static class AttractorObstacle extends Obstacle {
    public final Translation2d center;
    public final double maxRange;
    public final double soften;
    public final boolean waypoint;

    public AttractorObstacle(
        Translation2d center, double strength, double maxRange, boolean waypoint) {
      this(center, strength, maxRange, 0.18, waypoint);
    }

    public AttractorObstacle(
        Translation2d center, double strength, double maxRange, double soften, boolean waypoint) {
      super(strength, true);
      this.center = center;
      this.maxRange = Math.max(0.0, maxRange);
      this.soften = Math.max(1e-6, soften);
      this.waypoint = waypoint;
    }

    @Override
    public Force getForceAtPosition(Translation2d position, Translation2d target) {
      double dist = position.getDistance(center);
      if (dist < EPS || dist > maxRange) return new Force();

      double magBase = strength / (soften + dist * dist);
      double magFalloff = strength / (soften + maxRange * maxRange);
      double mag = Math.max(magBase - magFalloff, 0.0);
      if (mag < EPS) return new Force();

      Translation2d toward = center.minus(position);
      if (toward.getNorm() < EPS) return new Force();

      return new Force(mag, toward.getAngle());
    }

    @Override
    public boolean intersectsRectangle(Translation2d[] rectCorners) {
      return false;
    }
  }

  public static class GatedAttractorObstacle extends Obstacle {
    public final Translation2d center;
    public final double maxRange;
    public final double soften;
    public final Translation2d[] gatePoly;
    public final Translation2d bypassPoint;
    public final double bypassStrengthScale;
    public final double bypassRange;
    public final boolean waypoint;

    public GatedAttractorObstacle(
        Translation2d center,
        double strength,
        double maxRange,
        Translation2d[] gatePoly,
        Translation2d bypassPoint,
        double bypassStrengthScale,
        double bypassRange) {
      this(
          center,
          strength,
          maxRange,
          gatePoly,
          bypassPoint,
          bypassStrengthScale,
          bypassRange,
          0.18,
          false);
    }

    public GatedAttractorObstacle(
        Translation2d center,
        double strength,
        double maxRange,
        Translation2d[] gatePoly,
        Translation2d bypassPoint,
        double bypassStrengthScale,
        double bypassRange,
        boolean waypoint) {
      this(
          center,
          strength,
          maxRange,
          gatePoly,
          bypassPoint,
          bypassStrengthScale,
          bypassRange,
          0.18,
          waypoint);
    }

    public GatedAttractorObstacle(
        Translation2d center,
        double strength,
        double maxRange,
        Translation2d[] gatePoly,
        Translation2d bypassPoint,
        double bypassStrengthScale,
        double bypassRange,
        double soften,
        boolean waypoint) {
      super(strength, true);
      this.center = center;
      this.maxRange = Math.max(0.0, maxRange);
      this.gatePoly = gatePoly;
      this.bypassPoint = bypassPoint;
      this.bypassStrengthScale = Math.max(1.0, bypassStrengthScale);
      this.bypassRange = Math.max(0.0, bypassRange);
      this.soften = Math.max(1e-6, soften);
      this.waypoint = waypoint;
    }

    @Override
    public Force getForceAtPosition(Translation2d position, Translation2d target) {
      Translation2d pullTo = center;
      double range = maxRange;
      double strengthScale = 1.0;

      if (gatePoly != null
          && bypassPoint != null
          && segmentIntersectsPolygonOuter(position, center, gatePoly)) {
        pullTo = bypassPoint;
        range = bypassRange;
        strengthScale = bypassStrengthScale;
      }

      double dist = position.getDistance(pullTo);
      if (dist < EPS || dist > range) return new Force();

      double magBase = (strength * strengthScale) / (soften + dist * dist);
      double magFalloff = (strength * strengthScale) / (soften + range * range);
      double mag = Math.max(magBase - magFalloff, 0.0);
      if (mag < EPS) return new Force();

      Translation2d toward = pullTo.minus(position);
      if (toward.getNorm() < EPS) return new Force();

      return new Force(mag, toward.getAngle());
    }

    @Override
    public boolean intersectsRectangle(Translation2d[] rectCorners) {
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
  private Pose2d requestedGoal = Pose2d.kZero;
  private Translation2d stagedAttractor = null;
  private Translation2d lastStagedPoint = null;
  private boolean stagedComplete = false;

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
    var dsBase = RepulsorDriverStation.getInstance();
    for (Obstacle obs : fieldObstacles)
      force = force.plus(obs.getForceAtPosition(curLocation, target));
    for (Obstacle obs : extra)
      force =
          force.plus(
              obs.getForceAtPosition(curLocation, target)
                  .times(
                      dsBase instanceof NtRepulsorDriverStation ds
                          ? ds.getConfigDouble("repulsion_scale")
                          : 1.0));
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

  public void setRequestedGoal(Pose2d requested) {
    boolean same = isPoseNear(this.requestedGoal, requested);
    this.requestedGoal = requested;

    if (!same) {
      this.stagedAttractor = null;
      this.lastStagedPoint = null;
      this.stagedComplete = false;
    }

    lastChosenSetpoint = Optional.empty();
    Logger.recordOutput("Repulsor/RequestedGoal", requested);
  }

  private void setActiveGoal(Pose2d active) {
    this.goal = active;
    Logger.recordOutput("Repulsor/Goal/Active", active);
  }

  // public void setGoal(Pose2d goal) {
  //   boolean sameGoal = isPoseNear(this.requestedGoal, goal);
  //   this.goal = goal;
  //   this.requestedGoal = goal;
  //   if (!sameGoal) {
  //     this.stagedAttractor = null;
  //     this.lastStagedPoint = null;
  //     this.stagedComplete = false;
  //   }
  //   lastChosenSetpoint = Optional.empty();
  //   Logger.recordOutput("Repulsor/Setpoint", goal);
  // }

  public Optional<Distance> getErr() {
    return currentErr;
  }

  public void clearCommitted() {}

  public RepulsorSample calculateAndClear(
      Pose2d pose,
      List<? extends Obstacle> dynamicObstacles,
      double robot_x,
      double robot_y,
      double coral_offset,
      double algae_offset,
      CategorySpec cat,
      double shooterReleaseHeightMeters) {
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

    Translation2d curTrans = pose.getTranslation();
    double distToGoal = curTrans.getDistance(goal.getTranslation());

    var dsBase = RepulsorDriverStation.getInstance();
    if (dsBase instanceof NtRepulsorDriverStation ds) {
      ds.forcedGoalPose("main").ifPresent(this::setRequestedGoal);
    }

    updateStagedGoal(curTrans);
    distToGoal = curTrans.getDistance(goal.getTranslation());

    ClearMemo memo = new ClearMemo();

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
          !memo.toGoalNoDyn(curTrans, goal.getTranslation(), robot_x, robot_y);

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

      boolean pathBlocked = false;
      if (!suppressIsClearPath) {
        pathBlocked =
            !memo.toGoalDyn(curTrans, goal.getTranslation(), effectiveDynamics, robot_x, robot_y);
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
            setActiveGoal(altGoal);
            lastChosenSetpoint = Optional.of(sp);
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

    Rotation2d desiredHeadingRaw =
        (cat == CategorySpec.kCollect) ? effectiveGoal.getRotation() : netForce.getAngle();
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

  private static final double STAGED_REACH_ENTER_M = 0.35; 
private static final double STAGED_REACH_EXIT_M  = 0.55; 
private static final int    STAGED_REACH_TICKS   = 3;    
private int stagedReachTicks = 0;


  private static final double STAGED_GATE_PAD_M = 0.25;   
private static final double STAGED_PASSED_X_HYST_M = 0.35;
private boolean stagedUsingBypass = false;

private static Translation2d polyCentroid(Translation2d[] poly) {
  if (poly == null || poly.length == 0) return null;
  double sx = 0.0, sy = 0.0;
  for (Translation2d p : poly) { sx += p.getX(); sy += p.getY(); }
  return new Translation2d(sx / poly.length, sy / poly.length);
}

private static Translation2d[] expandPoly(Translation2d[] poly, double pad) {
  if (poly == null || poly.length == 0) return poly;
  if (pad <= 1e-9) return poly;

  Translation2d c = polyCentroid(poly);
  if (c == null) return poly;

  Translation2d[] out = new Translation2d[poly.length];
  for (int i = 0; i < poly.length; i++) {
    Translation2d v = poly[i].minus(c);
    double n = v.getNorm();
    if (n < 1e-9) out[i] = poly[i];
    else out[i] = c.plus(v.div(n).times(n + pad));
  }
  return out;
}

private static double firstIntersectionT(Translation2d a, Translation2d b, Translation2d[] poly) {
  if (poly == null || poly.length < 3) return Double.POSITIVE_INFINITY;

  if (FieldPlanner.isPointInPolygon(a, poly)) return 0.0;

  double bestT = Double.POSITIVE_INFINITY;
  for (int i = 0; i < poly.length; i++) {
    Translation2d c = poly[i];
    Translation2d d = poly[(i + 1) % poly.length];
    Double t = segmentIntersectionParam(a, b, c, d);
    if (t != null && t >= 0.0 && t <= 1.0 && t < bestT) bestT = t;
  }
  return bestT;
}

private static Double segmentIntersectionParam(
    Translation2d a, Translation2d b, Translation2d c, Translation2d d) {

  double ax = a.getX(), ay = a.getY();
  double bx = b.getX(), by = b.getY();
  double cx = c.getX(), cy = c.getY();
  double dx = d.getX(), dy = d.getY();

  double rpx = bx - ax;
  double rpy = by - ay;
  double spx = dx - cx;
  double spy = dy - cy;

  double rxs = rpx * spy - rpy * spx;
  double qpx = cx - ax;
  double qpy = cy - ay;

  double qpxr = qpx * rpy - qpy * rpx;

  double eps = 1e-9;
  if (Math.abs(rxs) < eps) {
    return null;
  }

  double t = (qpx * spy - qpy * spx) / rxs;
  double u = qpxr / rxs;

  if (t >= -eps && t <= 1.0 + eps && u >= -eps && u <= 1.0 + eps) {
    if (t < 0.0) t = 0.0;
    if (t > 1.0) t = 1.0;
    return t;
  }

  return null;
}

private static boolean gateIsBehind(Translation2d pos, Translation2d goal, GatedAttractorObstacle gate) {
  if (pos == null || goal == null || gate == null || gate.center == null) return false;
  double dir = Math.signum(goal.getX() - pos.getX());
  if (dir == 0.0) return false;
  double gx = gate.center.getX();
  if (dir > 0.0) return pos.getX() > gx + STAGED_PASSED_X_HYST_M;
  else return pos.getX() < gx - STAGED_PASSED_X_HYST_M;
}

private GatedAttractorObstacle firstOccludingGateAlongSegment(Translation2d pos, Translation2d target) {
  if (gatedAttractors.isEmpty() || pos == null || target == null) return null;

  GatedAttractorObstacle best = null;
  double bestT = Double.POSITIVE_INFINITY;

  for (GatedAttractorObstacle gate : gatedAttractors) {
    if (gate == null || gate.gatePoly == null) continue;
    if (gateIsBehind(pos, target, gate)) continue;

    Translation2d[] poly = expandPoly(gate.gatePoly, STAGED_GATE_PAD_M);

    if (!segmentIntersectsPolygonOuter(pos, target, poly)) continue;

    double t = firstIntersectionT(pos, target, poly);
    if (t < bestT) {
      bestT = t;
      best = gate;
    }
  }

  return best;
}

  private static int sideSignXBand(double x, double band) {
    double mid = Constants.FIELD_LENGTH * 0.5;
    double b = Math.max(0.0, band);
    if (x < mid - b) return -1;
    if (x > mid + b) return 1;
    return 0;
  }

  private static boolean isPoseNear(Pose2d a, Pose2d b) {
    if (a == null || b == null) return false;
    if (a.getTranslation().getDistance(b.getTranslation()) > STAGED_SAME_GOAL_POS_M) return false;
    double rotDeg =
        Math.abs(
            MathUtil.angleModulus(a.getRotation().getRadians() - b.getRotation().getRadians()));
    return rotDeg <= Math.toRadians(STAGED_SAME_GOAL_ROT_DEG);
  }

  private boolean shouldStageThroughAttractor(Translation2d pos, Translation2d target) {
    int goalSide = sideSignXBand(target.getX(), STAGED_CENTER_BAND_M);
    int robotSide = sideSignXBand(pos.getX(), STAGED_CENTER_BAND_M);
    if (goalSide == 0 && robotSide != 0) return true;
    if (goalSide != 0 && robotSide == 0) return true;
    return goalSide != 0 && robotSide != 0 && goalSide != robotSide;
  }

private Translation2d stagingPullPoint(
    GatedAttractorObstacle gate, Translation2d pos, Translation2d target) {

  if (gate == null) return null;
  Translation2d center = gate.center;
  if (center == null) return null;

  if (gate.gatePoly == null || gate.bypassPoint == null) return center;
  if (pos == null || target == null) return center;

  if (gate == stagedGate && stagedUsingBypass) return gate.bypassPoint;

  Translation2d[] poly = expandPoly(gate.gatePoly, STAGED_GATE_PAD_M);
  if (segmentIntersectsPolygonOuter(pos, target, poly)) {
    return gate.bypassPoint;
  }
  return center;
}

private GatedAttractorObstacle chooseBestGateByScore(Translation2d pos, Translation2d target) {
  GatedAttractorObstacle best = null;
  double bestScore = Double.POSITIVE_INFINITY;

  for (GatedAttractorObstacle gate : gatedAttractors) {
    if (gate == null) continue;
    if (gateIsBehind(pos, target, gate)) continue;

    Translation2d pullTo = stagingPullPoint(gate, pos, target);
    if (pullTo == null) continue;

    double score = pos.getDistance(pullTo) + pullTo.getDistance(target);
    if (score < bestScore) {
      bestScore = score;
      best = gate;
    }
  }
  return best;
}

private void updateStagedGoal(Translation2d curPos) {
  if (gatedAttractors.isEmpty()) {
    goal = requestedGoal;
    stagedAttractor = null;
    stagedGate = null;
    stagedUsingBypass = false;
    lastStagedPoint = null;
    stagedComplete = false;
    stagedReachTicks = 0;
    return;
  }

  Translation2d reqT = requestedGoal.getTranslation();

  GatedAttractorObstacle firstBlock = firstOccludingGateAlongSegment(curPos, reqT);

  boolean shouldStage =
      shouldStageThroughAttractor(curPos, reqT) || (firstBlock != null);

  if (shouldStage && stagedAttractor == null) {
    GatedAttractorObstacle gateToUse =
        (firstBlock != null) ? firstBlock : chooseBestGateByScore(curPos, reqT);

    if (gateToUse != null) {
      stagedUsingBypass =
          (gateToUse.gatePoly != null && gateToUse.bypassPoint != null)
              && segmentIntersectsPolygonOuter(
                  curPos, reqT, expandPoly(gateToUse.gatePoly, STAGED_GATE_PAD_M));

      Translation2d pick = stagingPullPoint(gateToUse, curPos, reqT);

      if (pick != null && curPos.getDistance(pick) > STAGED_REACH_EXIT_M) { // <-- change
        stagedGate = gateToUse;
        stagedAttractor = pick;
        stagedReachTicks = 0;

        lastStagedPoint = pick;
        Pose2d staged = new Pose2d(pick, requestedGoal.getRotation());
        setActiveGoal(staged);
        Logger.recordOutput("Repulsor/StagedGoal", staged);
        stagedComplete = false;
        return;
      } else {
        stagedUsingBypass = false;
        stagedReachTicks = 0;
      }
    }
  } // <-- ensure this closes BEFORE the next if

  if (stagedAttractor != null) {
    Translation2d liveTarget =
        (stagedGate != null) ? stagingPullPoint(stagedGate, curPos, reqT) : stagedAttractor;
    if (liveTarget == null) liveTarget = stagedAttractor;

    double d = curPos.getDistance(liveTarget);

    if (d <= STAGED_REACH_ENTER_M) stagedReachTicks++;
    else if (d >= STAGED_REACH_EXIT_M) stagedReachTicks = 0;

    boolean reached = stagedReachTicks >= STAGED_REACH_TICKS;

    boolean gateCleared = true;
    if (stagedGate != null && stagedGate.gatePoly != null) {
      Translation2d[] poly = expandPoly(stagedGate.gatePoly, STAGED_GATE_PAD_M);
      gateCleared =
          gateIsBehind(curPos, reqT, stagedGate) ||
          !segmentIntersectsPolygonOuter(curPos, reqT, poly);
    }

    if (reached && gateCleared) {
      stagedAttractor = null;
      stagedGate = null;
      lastStagedPoint = null;
      stagedUsingBypass = false;
      stagedReachTicks = 0;

      goal = requestedGoal;
      Logger.recordOutput("Repulsor/StagedGoal", goal);
      return;
    }

    if (liveTarget.getDistance(stagedAttractor) > 0.02) {
      stagedAttractor = liveTarget;
      goal = new Pose2d(stagedAttractor, requestedGoal.getRotation());
      Logger.recordOutput("Repulsor/StagedGoal", goal);
    }
    return;
  }

  goal = requestedGoal;
}

}
