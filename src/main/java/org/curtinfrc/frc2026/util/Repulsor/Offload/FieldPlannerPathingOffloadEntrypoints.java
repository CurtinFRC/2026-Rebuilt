package org.curtinfrc.frc2026.util.Repulsor.Offload;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import org.curtinfrc.frc2026.util.Repulsor.ExtraPathing;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacle;

@SuppressWarnings("unused")
public final class FieldPlannerPathingOffloadEntrypoints {
  private FieldPlannerPathingOffloadEntrypoints() {}

  @Offloadable(
      id = OffloadTaskIds.FIELD_PLANNER_IS_CLEAR_PATH,
      version = 1,
      timeoutMs = 120,
      fallback = true)
  public static boolean isClearPath(
      String topicRoot,
      Translation2d start,
      Translation2d goal,
      List<? extends Obstacle> obstacles,
      double robotLengthMeters,
      double robotWidthMeters,
      boolean publishSamples) {
    return ExtraPathing.isClearPath(
        topicRoot, start, goal, obstacles, robotLengthMeters, robotWidthMeters, publishSamples);
  }

  @Offloadable(
      id = OffloadTaskIds.FIELD_PLANNER_ROBOT_INTERSECTS,
      version = 1,
      timeoutMs = 120,
      fallback = true)
  public static boolean robotIntersects(
      Translation2d center,
      double robotLengthMeters,
      double robotWidthMeters,
      List<? extends Obstacle> obstacles) {
    return ExtraPathing.robotIntersects(center, robotLengthMeters, robotWidthMeters, obstacles);
  }
}
