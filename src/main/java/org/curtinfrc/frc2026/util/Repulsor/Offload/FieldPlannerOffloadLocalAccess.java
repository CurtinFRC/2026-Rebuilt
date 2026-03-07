package org.curtinfrc.frc2026.util.Repulsor.Offload;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.FieldPlanner;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacle;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.RepulsorSample;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldMapBuilder.CategorySpec;

public final class FieldPlannerOffloadLocalAccess {
  private static final Object LOCK = new Object();
  private static FieldPlanner planner;

  private FieldPlannerOffloadLocalAccess() {}

  public static FieldPlannerCalculateResultDTO calculateLocal(
      Pose2d pose,
      Pose2d requestedGoalPose,
      List<? extends Obstacle> dynamicObstacles,
      double robot_x,
      double robot_y,
      String categoryName,
      boolean suppressFallback,
      double shooterReleaseHeightMeters) {
    synchronized (LOCK) {
      FieldPlanner localPlanner = planner();
      localPlanner.setRequestedGoal(requestedGoalPose == null ? Pose2d.kZero : requestedGoalPose);

      CategorySpec cat = parseCategory(categoryName);
      RepulsorSample sample =
          localPlanner.calculate(
              pose == null ? Pose2d.kZero : pose,
              dynamicObstacles == null ? List.of() : dynamicObstacles,
              robot_x,
              robot_y,
              cat,
              suppressFallback,
              shooterReleaseHeightMeters);

      FieldPlannerCalculateResultDTO out = new FieldPlannerCalculateResultDTO();
      Translation2d goal = sample.goal();
      out.setGoalX(goal.getX());
      out.setGoalY(goal.getY());
      out.setVxMetersPerSecond(sample.vxMetersPerSecond());
      out.setVyMetersPerSecond(sample.vyMetersPerSecond());
      out.setOmegaRadians(sample.omegaRadians());
      out.setHasErrMeters(localPlanner.getErr().isPresent());
      out.setErrMeters(localPlanner.getErr().map(d -> d.in(Meters)).orElse(0.0));

      Pose2d activeGoal = localPlanner.getGoalPose();
      out.setActiveGoalX(activeGoal.getX());
      out.setActiveGoalY(activeGoal.getY());
      out.setActiveGoalThetaRadians(activeGoal.getRotation().getRadians());
      return out;
    }
  }

  private static FieldPlanner planner() {
    if (planner == null) {
      planner = new FieldPlanner();
    }
    return planner;
  }

  private static CategorySpec parseCategory(String categoryName) {
    if (categoryName == null || categoryName.isBlank()) {
      return CategorySpec.kScore;
    }
    try {
      return CategorySpec.valueOf(categoryName);
    } catch (IllegalArgumentException ignored) {
      return CategorySpec.kScore;
    }
  }
}
