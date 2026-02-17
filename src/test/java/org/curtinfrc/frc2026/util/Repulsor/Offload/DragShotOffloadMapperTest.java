package org.curtinfrc.frc2026.util.Repulsor.Offload;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import java.util.Optional;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacle;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacles.PointObstacle;
import org.curtinfrc.frc2026.util.Repulsor.Shooting.ShotSolution;
import org.junit.jupiter.api.Test;

class DragShotOffloadMapperTest {
  @Test
  void shouldRoundTripOptionalShotSolutionThroughCodec() {
    Optional<ShotSolution> original =
        Optional.of(
            new ShotSolution(
                new Translation2d(6.2, 3.3),
                Rotation2d.fromRadians(0.8),
                16.0,
                Rotation2d.fromRadians(0.6),
                0.42,
                new Translation2d(8.0, 4.0),
                0.05));

    byte[] encoded =
        OffloadValueCodec.encode(
            "java.util.Optional<org.curtinfrc.frc2026.util.Repulsor.Shooting.ShotSolution>",
            original);

    Optional<ShotSolution> decoded =
        (Optional<ShotSolution>)
            OffloadValueCodec.decode(
                "java.util.Optional<org.curtinfrc.frc2026.util.Repulsor.Shooting.ShotSolution>",
                encoded);

    assertTrue(decoded.isPresent());
    assertEquals(
        original.get().launchSpeedMetersPerSecond(),
        decoded.get().launchSpeedMetersPerSecond(),
        1e-6);
    assertEquals(original.get().verticalErrorMeters(), decoded.get().verticalErrorMeters(), 1e-6);
  }

  @Test
  void shouldRoundTripObstacleListThroughCodecAdapter() {
    List<? extends Obstacle> original =
        List.of(new PointObstacle(new Translation2d(7.5, 3.5), 1.0, true));

    byte[] encoded =
        OffloadValueCodec.encode(
            "java.util.List<? extends org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacle>",
            original);

    List<? extends Obstacle> decoded =
        (List<? extends Obstacle>)
            OffloadValueCodec.decode(
                "java.util.List<? extends org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacle>",
                encoded);

    assertEquals(1, decoded.size());
  }
}
