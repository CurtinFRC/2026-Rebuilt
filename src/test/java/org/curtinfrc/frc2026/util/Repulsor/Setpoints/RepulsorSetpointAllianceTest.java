package org.curtinfrc.frc2026.util.Repulsor.Setpoints;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.concurrent.atomic.AtomicReference;
import org.junit.jupiter.api.Test;

class RepulsorSetpointAllianceTest {
  private static final double EPS = 1e-9;

  private static final RepulsorSetpoint SAMPLE_SETPOINT =
      new RepulsorSetpoint(
          new GameSetpoint("SAMPLE", SetpointType.kScore) {
            @Override
            public Pose2d bluePose(SetpointContext ctx) {
              return new Pose2d(2.10, 1.35, Rotation2d.fromDegrees(17.0));
            }
          },
          HeightSetpoint.NONE);

  @Test
  void getForAllianceReturnsBluePoseForBlueAlliance() {
    Pose2d expected = SAMPLE_SETPOINT.getBlue(SetpointContext.EMPTY);
    Pose2d actual = SAMPLE_SETPOINT.getForAlliance(Alliance.Blue, SetpointContext.EMPTY);

    assertPoseNear(expected, actual);
  }

  @Test
  void getForAllianceReturnsRedPoseForRedAlliance() {
    Pose2d expected = SAMPLE_SETPOINT.getRed(SetpointContext.EMPTY);
    Pose2d actual = SAMPLE_SETPOINT.getForAlliance(Alliance.Red, SetpointContext.EMPTY);

    assertPoseNear(expected, actual);
  }

  @Test
  void mutablePoseSetpointDoesNotFlipForRedAlliance() {
    Pose2d blueCollect = new Pose2d(3.1, 1.8, Rotation2d.fromDegrees(33.0));
    RepulsorSetpoint collectRoute =
        new RepulsorSetpoint(
            new MutablePoseSetpoint(
                "COLLECT_ROUTE", SetpointType.kOther, new AtomicReference<>(blueCollect)),
            HeightSetpoint.NONE);

    Pose2d blue = collectRoute.getForAlliance(Alliance.Blue, SetpointContext.EMPTY);
    Pose2d red = collectRoute.getForAlliance(Alliance.Red, SetpointContext.EMPTY);

    assertPoseNear(blueCollect, blue);
    assertPoseNear(blueCollect, red);
  }

  private static void assertPoseNear(Pose2d expected, Pose2d actual) {
    assertEquals(expected.getX(), actual.getX(), EPS);
    assertEquals(expected.getY(), actual.getY(), EPS);
    assertEquals(
        0.0,
        MathUtil.angleModulus(
            expected.getRotation().getRadians() - actual.getRotation().getRadians()),
        EPS);
  }
}
