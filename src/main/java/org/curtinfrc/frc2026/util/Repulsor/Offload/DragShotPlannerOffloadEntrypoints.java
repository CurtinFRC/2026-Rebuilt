package org.curtinfrc.frc2026.util.Repulsor.Offload;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import java.util.Optional;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacle;
import org.curtinfrc.frc2026.util.Repulsor.Shooting.Constraints;
import org.curtinfrc.frc2026.util.Repulsor.Shooting.DragShotPlannerLocalAccess;
import org.curtinfrc.frc2026.util.Repulsor.Shooting.GamePiecePhysics;
import org.curtinfrc.frc2026.util.Repulsor.Shooting.ShotSolution;

@SuppressWarnings("unused")
public final class DragShotPlannerOffloadEntrypoints {
  private DragShotPlannerOffloadEntrypoints() {}

  @Offloadable(
      id = OffloadTaskIds.DRAG_SHOT_FIND_BEST_SHOT_AUTO,
      version = 1,
      timeoutMs = 20,
      fallback = true)
  public static Optional<ShotSolution> findBestShotAuto(
      GamePiecePhysics gamePiece,
      Translation2d targetFieldPosition,
      double targetHeightMeters,
      Pose2d robotPose,
      double shooterReleaseHeightMeters,
      double robotHalfLengthMeters,
      double robotHalfWidthMeters,
      List<? extends Obstacle> dynamicObstacles,
      Constraints constraints) {
    return DragShotPlannerLocalAccess.findBestShotAutoLocal(
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

  @Offloadable(
      id = OffloadTaskIds.DRAG_SHOT_CALC_STATIC_SHOT_ANGLE_SPEED,
      version = 1,
      timeoutMs = 20,
      fallback = true)
  public static Optional<ShotSolution> calculateStaticShotAngleAndSpeed(
      GamePiecePhysics gamePiece,
      Translation2d shooterFieldPosition,
      Translation2d targetFieldPosition,
      double targetHeightMeters,
      double shooterReleaseHeightMeters,
      Constraints constraints) {
    return DragShotPlannerLocalAccess.calculateStaticShotAngleAndSpeedLocal(
        gamePiece,
        shooterFieldPosition,
        targetFieldPosition,
        targetHeightMeters,
        shooterReleaseHeightMeters,
        constraints);
  }
}
