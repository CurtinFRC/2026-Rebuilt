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
  private static final double CORNER_CHAMFER = 1.5;
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
