package org.curtinfrc.frc2026.util.Repulsor.Offload;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import java.util.Optional;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacles.PointObstacle;
import org.curtinfrc.frc2026.util.Repulsor.Shooting.Constraints;
import org.curtinfrc.frc2026.util.Repulsor.Shooting.GamePiecePhysics;
import org.curtinfrc.frc2026.util.Repulsor.Shooting.ShotSolution;
import org.junit.jupiter.api.Test;

class DragShotOffloadMapperTest {
  @Test
  void shouldConvertRequestAndResponseDtos() {
    GamePiecePhysics gamePiece =
        new GamePiecePhysics() {
          @Override
          public double massKg() {
            return 0.2;
          }

          @Override
          public double crossSectionAreaM2() {
            return 0.03;
          }

          @Override
          public double dragCoefficient() {
            return 0.4;
          }
        };

    Constraints constraints = new Constraints(10.0, 20.0, 15.0, 35.0, Constraints.ShotStyle.DIRECT);

    DragShotAutoRequestDTO request =
        DragShotOffloadMapper.toRequest(
            gamePiece,
            new Translation2d(8.0, 4.0),
            2.1,
            new edu.wpi.first.math.geometry.Pose2d(6.0, 3.5, Rotation2d.fromDegrees(10)),
            0.8,
            0.45,
            0.4,
            List.of(new PointObstacle(new Translation2d(7.5, 3.5), 1.0, true)),
            constraints);

    assertNotNull(request);
    assertEquals(8.0, request.getTargetFieldPosition().getX(), 1e-6);
    assertEquals(4.0, request.getTargetFieldPosition().getY(), 1e-6);
    assertEquals(1, request.getDynamicObstacles().size());

    ShotSolution solution =
        new ShotSolution(
            new Translation2d(6.2, 3.3),
            Rotation2d.fromRadians(0.8),
            16.0,
            Rotation2d.fromRadians(0.6),
            0.42,
            new Translation2d(8.0, 4.0),
            0.05);

    DragShotAutoResponseDTO response = DragShotOffloadMapper.toResponse(Optional.of(solution));
    Optional<ShotSolution> roundTripped = DragShotOffloadMapper.fromResponse(response);

    assertTrue(roundTripped.isPresent());
    assertEquals(
        solution.launchSpeedMetersPerSecond(),
        roundTripped.get().launchSpeedMetersPerSecond(),
        1e-6);
    assertEquals(solution.verticalErrorMeters(), roundTripped.get().verticalErrorMeters(), 1e-6);
  }
}
