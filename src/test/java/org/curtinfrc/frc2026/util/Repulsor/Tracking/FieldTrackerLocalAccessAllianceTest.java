package org.curtinfrc.frc2026.util.Repulsor.Tracking;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.curtinfrc.frc2026.util.Repulsor.Constants;
import org.junit.jupiter.api.Test;

class FieldTrackerLocalAccessAllianceTest {
  private static final double EPS = 1e-9;

  @Test
  void fallbackGoalClampsInsideAllianceZoneBounds() {
    Pose2d robotPoseBlue = new Pose2d(5.0, 4.0, Rotation2d.fromDegrees(20.0));

    Pose2d goal = FieldTrackerLocalAccess.fallbackGoal(robotPoseBlue);

    assertEquals(Constants.FIELD_LENGTH * 0.28, goal.getX(), EPS);
    assertEquals(robotPoseBlue.getY(), goal.getY(), EPS);
    assertEquals(robotPoseBlue.getRotation().getRadians(), goal.getRotation().getRadians(), EPS);
  }

  @Test
  void fallbackGoalClampsLowAndHighEdges() {
    Pose2d low = FieldTrackerLocalAccess.fallbackGoal(new Pose2d(0.1, 0.2, Rotation2d.kZero));
    assertEquals(0.7, low.getX(), EPS);
    assertEquals(0.7, low.getY(), EPS);

    Pose2d high =
        FieldTrackerLocalAccess.fallbackGoal(
            new Pose2d(Constants.FIELD_LENGTH, Constants.FIELD_WIDTH, Rotation2d.kZero));
    assertEquals(Constants.FIELD_LENGTH * 0.28, high.getX(), EPS);
    assertEquals(Constants.FIELD_WIDTH - 0.7, high.getY(), EPS);
  }
}
