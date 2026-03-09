package org.curtinfrc.frc2026.util.Repulsor.Offload;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.List;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacle;

@SuppressWarnings("unused")
public final class FieldPlannerOffloadEntrypoints {
  private FieldPlannerOffloadEntrypoints() {}

  @Offloadable(
      id = OffloadTaskIds.FIELD_PLANNER_CALCULATE,
      version = 2,
      timeoutMs = 250,
      fallback = false)
  public static FieldPlannerCalculateResultDTO calculate(
      Pose2d pose,
      Pose2d requestedGoalPose,
      Pose2d activeGoalPose,
      List<? extends Obstacle> dynamicObstacles,
      double robot_x,
      double robot_y,
      String categoryName,
      String preferredAllianceName,
      boolean suppressFallback,
      double shooterReleaseHeightMeters) {
    return FieldPlannerOffloadLocalAccess.calculateLocal(
        pose,
        requestedGoalPose,
        activeGoalPose,
        dynamicObstacles,
        robot_x,
        robot_y,
        categoryName,
        preferredAllianceName,
        suppressFallback,
        shooterReleaseHeightMeters);
  }
}
