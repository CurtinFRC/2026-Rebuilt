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
  private static final double FORCE_THROUGH_GOAL_DIST = 2.0;
  private static final double FORCE_THROUGH_WALL_DIST = 0.7;
  private static final double CORNER_CHAMFER = 0;
  public static final double GOAL_STRENGTH = 1.2;

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

  public static class RectangleObstacle extends Obstacle {
    public final Translation2d center;
    public final double halfX;
    public final double halfY;
    public final Rotation2d rot;
    public final double maxRangeX;
    public final double maxRangeY;

    private static final double X_AXIS_ANGLE_BIAS_RAD = Math.toRadians(18.0);

    private static final double CORNER_RANGE_M = 1.15;
    private static final double CORNER_FORCE_SCALE = 14.0;
    private static final double CORNER_FORCE_SOFTEN = 0.07;

    private static final double CORNER_SLIDE_SCALE = 9.0;
    private static final double CORNER_SLIDE_SOFTEN = 0.22;
    private static final double CORNER_SLIDE_STEP = 0.38;
    private static final double CORNER_SLIDE_OUT_STEP = 0.10;

    private static final double CORNER_ORBIT_SCALE = 7.5;
    private static final double CORNER_ORBIT_SOFTEN = 0.18;
    private static final double CORNER_ORBIT_STEP_T = 0.55;
    private static final double CORNER_ORBIT_STEP_O = 0.20;

    private static final double TIE_EPS = 0.03;

    private static double clamp01(double x) {
      return Math.max(0.0, Math.min(1.0, x));
    }

    private static double smooth01(double x) {
      x = clamp01(x);
      return x * x * (3.0 - 2.0 * x);
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

    private static Translation2d chooseCornerSlideDir(
        Translation2d pos,
        Translation2d goal,
        Translation2d cornerWorld,
        Translation2d centerWorld,
        Translation2d[] polyExp) {

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

      Translation2d p1 =
          pos.plus(t1.times(CORNER_SLIDE_STEP)).plus(outward.times(CORNER_SLIDE_OUT_STEP));
      Translation2d p2 =
          pos.plus(t2.times(CORNER_SLIDE_STEP)).plus(outward.times(CORNER_SLIDE_OUT_STEP));

      double base = pos.getDistance(goal);

      double s1 = scoreCandidatePolyWithWalls(pos, goal, p1, base, polyExp);
      double s2 = scoreCandidatePolyWithWalls(pos, goal, p2, base, polyExp);

      Translation2d tCW = t2;
      Translation2d tCCW = t1;
      Translation2d pCW = p2;
      Translation2d pCCW = p1;
      double sCW = s2;
      double sCCW = s1;

      return stablePick(pos, goal, centerWorld, tCW, tCCW, pCW, pCCW, sCW, sCCW) == tCW ? t2 : t1;
    }

    private static Translation2d chooseCornerOrbitDir(
        Translation2d pos,
        Translation2d goal,
        Translation2d cornerWorld,
        Translation2d centerWorld,
        Translation2d[] polyExp) {

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

      Translation2d pCW =
          pos.plus(tCW.times(CORNER_ORBIT_STEP_T)).plus(outward.times(CORNER_ORBIT_STEP_O));
      Translation2d pCCW =
          pos.plus(tCCW.times(CORNER_ORBIT_STEP_T)).plus(outward.times(CORNER_ORBIT_STEP_O));

      double base = pos.getDistance(goal);

      double sCW = scoreCandidatePolyWithWalls(pos, goal, pCW, base, polyExp);
      double sCCW = scoreCandidatePolyWithWalls(pos, goal, pCCW, base, polyExp);

      return stablePick(pos, goal, centerWorld, tCW, tCCW, pCW, pCCW, sCW, sCCW);
    }

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

      double clx = Math.max(-halfX, Math.min(px, halfX));
      double cly = Math.max(-halfY, Math.min(py, halfY));

      double dx = px - clx;
      double dy = py - cly;

      boolean inside = (px >= -halfX && px <= halfX && py >= -halfY && py <= halfY);

      double ax = inside ? 0.0 : Math.abs(dx);
      double ay = inside ? 0.0 : Math.abs(dy);

      Translation2d[] poly = corners();
      boolean occludes = segmentIntersectsPolygon(position, target, poly);

      if (!inside && (ax > maxRangeX || ay > maxRangeY) && !occludes) return new Force();

      double diag = Math.hypot(halfX, halfY);
      double falloffMeters = Math.max(EPS, Math.max(maxRangeX, maxRangeY) + diag + 0.35);

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

      Translation2d escapeWorld = Translation2d.kZero;

      double engageR = Math.max(0.9, falloffMeters + 0.65);
      double distC = position.getDistance(center);

      if (distC <= engageR) {
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

        double pad = Math.max(0.55, Math.min(2.0, Math.max(maxRangeX, maxRangeY) * 0.85));
        Translation2d[] polyExp = expandedCorners(pad);

        Translation2d chosenT =
            chooseTangentPolyWithWalls(position, target, outwardWU, tCW, tCCW, polyExp);

        double d = Math.max(0.12, distC - diag);
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

        if (occludes) {
          escapeWorld = escapeWorld.plus(add);
        } else {
          Translation2d sumTry = primaryWorld.plus(add);
          if (sumTry.getNorm() < 1e-6 || along < 0.03) escapeWorld = escapeWorld.plus(add);
        }
      }

      Translation2d cornerBoost = Translation2d.kZero;
      Translation2d[] c = poly;
      double dC0 = position.getDistance(c[0]);
      double dC1 = position.getDistance(c[1]);
      double dC2 = position.getDistance(c[2]);
      double dC3 = position.getDistance(c[3]);
      double dCorner = Math.min(Math.min(dC0, dC1), Math.min(dC2, dC3));
      boolean nearCorner = dCorner <= CORNER_RANGE_M;

      if (nearCorner) {
        int idx = 0;
        double best = dC0;
        if (dC1 < best) {
          best = dC1;
          idx = 1;
        }
        if (dC2 < best) {
          best = dC2;
          idx = 2;
        }
        if (dC3 < best) {
          best = dC3;
          idx = 3;
        }

        Translation2d corner = c[idx];

        Translation2d away = position.minus(corner);
        double n = away.getNorm();
        if (n < EPS) {
          Translation2d outward = position.minus(center);
          double on = outward.getNorm();
          if (on < EPS) outward = new Translation2d(1.0, 0.0);
          away = outward.div(Math.max(EPS, outward.getNorm()));
          n = 1.0;
        }
        Translation2d awayU = away.div(Math.max(EPS, n));

        Translation2d toGoal = target.minus(position);
        double gN = Math.max(EPS, toGoal.getNorm());
        Translation2d toGoalU = toGoal.div(gN);

        double w = smooth01(1.0 - (dCorner / CORNER_RANGE_M));
        double w2 = w * w;

        double mPush =
            (strength * CORNER_FORCE_SCALE) * w2 / (CORNER_FORCE_SOFTEN + dCorner * dCorner);
        Translation2d push = awayU.times(mPush);

        double padCorner = Math.max(0.55, Math.min(2.0, Math.max(maxRangeX, maxRangeY) * 0.85));
        Translation2d[] polyExpCorner = expandedCorners(padCorner);

        Translation2d slideDir =
            chooseCornerSlideDir(position, target, corner, center, polyExpCorner);
        double mSlide =
            (strength * CORNER_SLIDE_SCALE) * w2 / (CORNER_SLIDE_SOFTEN + dCorner * dCorner);
        Translation2d slide = slideDir.times(mSlide);

        double alignAway = 0.0;
        if (awayWorldU.getNorm() > EPS) alignAway = dot(awayWorldU, toGoalU);
        double needOrbit = clamp01((0.42 - alignAway) / 0.42);

        Translation2d orbitDir =
            chooseCornerOrbitDir(position, target, corner, center, polyExpCorner);
        double mOrbit =
            (strength * CORNER_ORBIT_SCALE)
                * w2
                * needOrbit
                / (CORNER_ORBIT_SOFTEN + dCorner * dCorner);

        Translation2d orbit = orbitDir.times(mOrbit);

        Translation2d outwardFromCenter = position.minus(center);
        double ocn = outwardFromCenter.getNorm();
        if (ocn < EPS) outwardFromCenter = new Translation2d(1.0, 0.0);
        outwardFromCenter = outwardFromCenter.div(Math.max(EPS, outwardFromCenter.getNorm()));
        Translation2d unstickOut =
            outwardFromCenter.times((strength * 1.6) * w2 / (0.35 + dCorner * dCorner));

        cornerBoost = push.plus(slide).plus(orbit).plus(unstickOut);
      }

      Translation2d sum = primaryWorld.plus(escapeWorld).plus(cornerBoost);

      double sumN = sum.getNorm();
      Translation2d g = target.minus(position);
      double gN = g.getNorm();
      if (gN > EPS) {
        Translation2d gU = g.div(gN);
        double engageR2 = Math.max(1.25, falloffMeters + 0.95);
        if (distC <= engageR2 || nearCorner) {
          if (sumN > EPS) {
            double align = dot(sum.div(sumN), gU);
            if (align < 0.25) {
              double d = Math.max(0.18, distC - diag);
              double pushThroughMag = (strength * 2.6) / (0.85 + d * d);
              sum = sum.plus(gU.times(pushThroughMag * (0.25 - align)));
              sumN = sum.getNorm();
            }
          } else {
            double d = Math.max(0.18, distC - diag);
            double pushThroughMag = (strength * 2.6) / (0.85 + d * d);
            sum = sum.plus(gU.times(pushThroughMag));
            sumN = sum.getNorm();
          }
        }
      }

      if (sumN < EPS) return new Force();
      return new Force(sumN, sum.getAngle());
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

    private static Translation2d chooseTangentPolyWithWalls(
        Translation2d pos,
        Translation2d goal,
        Translation2d outwardU,
        Translation2d tCW,
        Translation2d tCCW,
        Translation2d[] polyExp) {

      double stepT = 0.62;
      double stepO = 0.24;

      Translation2d pCW = pos.plus(tCW.times(stepT)).plus(outwardU.times(stepO));
      Translation2d pCCW = pos.plus(tCCW.times(stepT)).plus(outwardU.times(stepO));

      double base = pos.getDistance(goal);

      double sCW = scoreCandidatePolyWithWalls(pos, goal, pCW, base, polyExp);
      double sCCW = scoreCandidatePolyWithWalls(pos, goal, pCCW, base, polyExp);

      Translation2d centerGuess = pos.minus(outwardU.times(0.001));
      return stablePick(pos, goal, centerGuess, tCW, tCCW, pCW, pCCW, sCW, sCCW);
    }

    private static double scoreCandidatePolyWithWalls(
        Translation2d pos,
        Translation2d goal,
        Translation2d cand,
        double baseDist,
        Translation2d[] polyExp) {

      double d = cand.getDistance(goal);

      boolean stillOccluding = segmentIntersectsPolygon(cand, goal, polyExp);
      double occPenalty = stillOccluding ? 1.10 : 0.0;

      double progressPenalty = (d >= baseDist - 0.015) ? 0.55 : 0.0;

      double x = cand.getX();
      double y = cand.getY();
      double dxW = Math.min(x, Constants.FIELD_LENGTH - x);
      double dyW = Math.min(y, Constants.FIELD_WIDTH - y);
      double wall = Math.min(dxW, dyW);
      double wallPenalty = (wall < 0.70) ? (1.05 * (0.70 - wall) / 0.70) : 0.0;

      double wallMove = wallMovePenalty(pos, cand);

      return d + occPenalty + progressPenalty + wallPenalty + wallMove;
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
    lastChosenSetpoint = Optional.empty();
    Logger.recordOutput("Repulsor/Setpoint", goal);
  }

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
            setGoal(altGoal);
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
}
