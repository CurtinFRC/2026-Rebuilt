package org.curtinfrc.frc2026.util.Repulsor.Shooting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import java.util.Optional;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacle;

public final class DragShotPlannerLocalAccess {
  private DragShotPlannerLocalAccess() {}

  public static Optional<ShotSolution> findBestShotAutoLocal(
      GamePiecePhysics gamePiece,
      Translation2d targetFieldPosition,
      double targetHeightMeters,
      Pose2d robotPose,
      double shooterReleaseHeightMeters,
      double robotHalfLengthMeters,
      double robotHalfWidthMeters,
      List<? extends Obstacle> dynamicObstacles,
      Constraints constraints) {
    return DragShotPlannerCore.findBestShotAuto(
        gamePiece,
        targetFieldPosition,
        targetHeightMeters,
        robotPose,
        shooterReleaseHeightMeters,
        robotHalfLengthMeters,
        robotHalfWidthMeters,
        dynamicObstacles,
        constraints);
  }
}
