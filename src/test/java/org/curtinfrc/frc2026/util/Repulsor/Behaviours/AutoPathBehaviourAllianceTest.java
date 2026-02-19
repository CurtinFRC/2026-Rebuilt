package org.curtinfrc.frc2026.util.Repulsor.Behaviours;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.concurrent.atomic.AtomicReference;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.GameSetpoint;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.HeightSetpoint;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.MutablePoseSetpoint;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.RepulsorSetpoint;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.SetpointContext;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.SetpointType;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Model.Alliance;
import org.junit.jupiter.api.Test;

class AutoPathBehaviourAllianceTest {
  private static final double EPS = 1e-9;

  @Test
  void toRepulsorAllianceMapsDriverStationAlliance() {
    assertEquals(Alliance.kBlue, AutoPathBehaviour.toRepulsorAlliance(DriverStation.Alliance.Blue));
    assertEquals(Alliance.kRed, AutoPathBehaviour.toRepulsorAlliance(DriverStation.Alliance.Red));
    assertEquals(Alliance.kBlue, AutoPathBehaviour.toRepulsorAlliance(null));
  }

  @Test
  void resolvePlannerGoalPoseUsesRequestedAlliance() {
    RepulsorSetpoint setpoint =
        fixedScoreSetpoint("SAMPLE", new Pose2d(2.4, 1.1, Rotation2d.fromDegrees(25.0)));

    Pose2d expectedBlue = setpoint.getBlue(SetpointContext.EMPTY);
    Pose2d blueActual =
        AutoPathBehaviour.resolvePlannerGoalPose(
            setpoint, SetpointContext.EMPTY, DriverStation.Alliance.Blue);
    assertPoseNear(expectedBlue, blueActual);

    Pose2d expectedRed = setpoint.getRed(SetpointContext.EMPTY);
    Pose2d redActual =
        AutoPathBehaviour.resolvePlannerGoalPose(
            setpoint, SetpointContext.EMPTY, DriverStation.Alliance.Red);
    assertPoseNear(expectedRed, redActual);
  }

  @Test
  void resolvePlannerGoalPoseReturnsZeroForNullSetpoint() {
    Pose2d actual =
        AutoPathBehaviour.resolvePlannerGoalPose(
            null, SetpointContext.EMPTY, DriverStation.Alliance.Blue);
    assertPoseNear(Pose2d.kZero, actual);
  }

  @Test
  void resolvePlannerGoalPoseDoesNotFlipMutableCollectRoute() {
    Pose2d collectBluePose = new Pose2d(5.2, 2.6, Rotation2d.fromDegrees(11.0));
    RepulsorSetpoint collectRoute =
        new RepulsorSetpoint(
            new MutablePoseSetpoint(
                "COLLECT_ROUTE", SetpointType.kOther, new AtomicReference<>(collectBluePose)),
            HeightSetpoint.NONE);

    Pose2d blueGoal =
        AutoPathBehaviour.resolvePlannerGoalPose(
            collectRoute, SetpointContext.EMPTY, DriverStation.Alliance.Blue);
    Pose2d redGoal =
        AutoPathBehaviour.resolvePlannerGoalPose(
            collectRoute, SetpointContext.EMPTY, DriverStation.Alliance.Red);

    assertPoseNear(collectBluePose, blueGoal);
    assertPoseNear(collectBluePose, redGoal);
  }

  private static RepulsorSetpoint fixedScoreSetpoint(String name, Pose2d bluePose) {
    return new RepulsorSetpoint(
        new GameSetpoint(name, SetpointType.kScore) {
          @Override
          public Pose2d bluePose(SetpointContext ctx) {
            return bluePose;
          }
        },
        HeightSetpoint.NONE);
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
