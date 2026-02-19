package org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Helpers;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import org.curtinfrc.frc2026.util.Repulsor.Constants;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacle;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacles.GatedAttractorObstacle;
import org.junit.jupiter.api.Test;

class FieldPlannerGoalManagerTest {
  private static final double EPS = 1e-9;

  @Test
  void releasesStageWhenCloseEnoughOnGoalSide() {
    GatedAttractorObstacle gate = gate(new Translation2d(8.0, 4.0), new Translation2d(8.2, 4.0));
    FieldPlannerGoalManager manager = new FieldPlannerGoalManager(List.of(gate));
    List<? extends Obstacle> obstacles = List.of(gate);

    Pose2d requested = new Pose2d(new Translation2d(12.0, 4.0), Rotation2d.kZero);
    manager.setRequestedGoal(requested);

    assertFalse(manager.updateStagedGoal(new Translation2d(6.0, 4.0), obstacles));
    assertEquals(8.2, manager.getGoalTranslation().getX(), EPS);
    assertEquals(4.0, manager.getGoalTranslation().getY(), EPS);

    assertTrue(manager.updateStagedGoal(new Translation2d(8.2, 4.0), obstacles));
    assertEquals(requested.getX(), manager.getGoalTranslation().getX(), EPS);
    assertEquals(requested.getY(), manager.getGoalTranslation().getY(), EPS);
  }

  @Test
  void doesNotRestageImmediatelyWithinRestageDistance() {
    GatedAttractorObstacle gate = gate(new Translation2d(8.0, 4.0), new Translation2d(8.2, 4.0));
    FieldPlannerGoalManager manager = new FieldPlannerGoalManager(List.of(gate));
    List<? extends Obstacle> obstacles = List.of(gate);

    Pose2d requested = new Pose2d(new Translation2d(4.0, 4.0), Rotation2d.kZero);
    manager.setRequestedGoal(requested);

    assertFalse(manager.updateStagedGoal(new Translation2d(10.0, 4.0), obstacles));
    assertTrue(manager.updateStagedGoal(new Translation2d(7.8, 4.0), obstacles));

    assertTrue(manager.updateStagedGoal(new Translation2d(9.0, 4.0), obstacles));
    assertEquals(requested.getX(), manager.getGoalTranslation().getX(), EPS);
    assertEquals(requested.getY(), manager.getGoalTranslation().getY(), EPS);
  }

  @Test
  void releasesStageAfterGateClearWithoutExactWaypointHit() {
    GatedAttractorObstacle gate = gate(new Translation2d(8.0, 4.0), new Translation2d(8.2, 5.0));
    FieldPlannerGoalManager manager = new FieldPlannerGoalManager(List.of(gate));
    List<? extends Obstacle> obstacles = List.of(gate);

    Pose2d requested = new Pose2d(new Translation2d(12.0, 4.0), Rotation2d.kZero);
    manager.setRequestedGoal(requested);

    assertFalse(manager.updateStagedGoal(new Translation2d(6.0, 4.0), obstacles));
    assertFalse(manager.updateStagedGoal(new Translation2d(8.3, 4.0), obstacles));

    assertTrue(manager.updateStagedGoal(new Translation2d(8.3, 4.0), obstacles));
    assertEquals(requested.getX(), manager.getGoalTranslation().getX(), EPS);
    assertEquals(requested.getY(), manager.getGoalTranslation().getY(), EPS);
  }

  @Test
  void keepsSameLaneAcrossMultipleGatesWhenSecondGateTieWouldFlip() {
    Translation2d[] leftBarrier =
        new Translation2d[] {
          new Translation2d(7.7, 0.7),
          new Translation2d(8.3, 0.7),
          new Translation2d(8.3, 7.3),
          new Translation2d(7.7, 7.3)
        };
    Translation2d[] rightBarrier =
        new Translation2d[] {
          new Translation2d(11.7, 0.7),
          new Translation2d(12.3, 0.7),
          new Translation2d(12.3, 7.3),
          new Translation2d(11.7, 7.3)
        };

    GatedAttractorObstacle leftTop =
        gate(new Translation2d(8.0, 7.0), new Translation2d(8.2, 7.0), leftBarrier);
    GatedAttractorObstacle leftBottom =
        gate(new Translation2d(8.0, 1.0), new Translation2d(8.2, 1.0), leftBarrier);
    // Right gates are intentionally ordered bottom before top. Without lane lock this tie can flip.
    GatedAttractorObstacle rightBottom =
        gate(new Translation2d(12.0, 1.0), new Translation2d(12.2, 1.0), rightBarrier);
    GatedAttractorObstacle rightTop =
        gate(new Translation2d(12.0, 7.0), new Translation2d(12.2, 7.0), rightBarrier);

    FieldPlannerGoalManager manager =
        new FieldPlannerGoalManager(List.of(leftTop, leftBottom, rightBottom, rightTop));
    List<? extends Obstacle> obstacles = List.of(leftTop, leftBottom, rightBottom, rightTop);

    Pose2d requested = new Pose2d(new Translation2d(16.0, 4.0), Rotation2d.kZero);
    manager.setRequestedGoal(requested);

    assertFalse(manager.updateStagedGoal(new Translation2d(6.0, 5.0), obstacles));
    assertEquals(7.0, manager.getGoalTranslation().getY(), EPS);

    assertFalse(manager.updateStagedGoal(new Translation2d(8.3, 5.0), obstacles));
    assertTrue(manager.updateStagedGoal(new Translation2d(8.3, 5.0), obstacles));

    assertFalse(manager.updateStagedGoal(new Translation2d(10.0, 4.0), obstacles));
    assertEquals(7.0, manager.getGoalTranslation().getY(), EPS);
  }

  @Test
  void doesNotForceStageWhenLeavingCenterIfNoGateOccludesPath() {
    GatedAttractorObstacle farGate = gate(new Translation2d(8.0, 7.0), new Translation2d(8.2, 7.0));
    FieldPlannerGoalManager manager = new FieldPlannerGoalManager(List.of(farGate));
    List<? extends Obstacle> obstacles = List.of(farGate);

    Pose2d requested = new Pose2d(new Translation2d(3.0, 4.0), Rotation2d.kZero);
    manager.setRequestedGoal(requested);

    // Robot starts in center band and drives to alliance side. Path at y=4 does not occlude this
    // gate.
    assertTrue(manager.updateStagedGoal(new Translation2d(8.2, 4.0), obstacles));
    assertEquals(requested.getX(), manager.getGoalTranslation().getX(), EPS);
    assertEquals(requested.getY(), manager.getGoalTranslation().getY(), EPS);
  }

  @Test
  void doesNotForceStageEarlyWhenLeavingDeepCenterAndOccludingGateIsFar() {
    double mid = Constants.FIELD_LENGTH * 0.5;
    Translation2d robot = new Translation2d(mid, 4.0);
    Translation2d goal = new Translation2d(mid - 4.8, 4.0);
    GatedAttractorObstacle gate =
        gate(new Translation2d(mid - 2.8, 4.0), new Translation2d(mid - 2.6, 4.0));
    FieldPlannerGoalManager manager = new FieldPlannerGoalManager(List.of(gate));
    List<? extends Obstacle> obstacles = List.of(gate);

    Pose2d requested = new Pose2d(goal, Rotation2d.kZero);
    manager.setRequestedGoal(requested);

    assertTrue(manager.updateStagedGoal(robot, obstacles));
    assertEquals(requested.getX(), manager.getGoalTranslation().getX(), EPS);
    assertEquals(requested.getY(), manager.getGoalTranslation().getY(), EPS);
  }

  @Test
  void centerReturnUsesExitStageBeforeReleasingRequestedGoal() {
    double mid = Constants.FIELD_LENGTH * 0.5;
    Translation2d gateCenter = new Translation2d(mid - 3.6, 4.0);
    GatedAttractorObstacle gate =
        gate(gateCenter, new Translation2d(gateCenter.getX() + 0.2, gateCenter.getY()));
    FieldPlannerGoalManager manager = new FieldPlannerGoalManager(List.of(gate));
    List<? extends Obstacle> obstacles = List.of(gate);

    Pose2d requested = new Pose2d(new Translation2d(mid - 6.0, 4.0), Rotation2d.kZero);
    manager.setRequestedGoal(requested);

    double firstStageX = gateCenter.getX() - 0.2;

    assertTrue(manager.updateStagedGoal(new Translation2d(mid, 4.0), obstacles));
    assertFalse(manager.updateStagedGoal(new Translation2d(mid - 2.0, 4.0), obstacles));
    assertFalse(manager.updateStagedGoal(new Translation2d(firstStageX, 4.0), obstacles));
    assertFalse(manager.updateStagedGoal(new Translation2d(firstStageX, 4.0), obstacles));

    double exitX = manager.getGoalTranslation().getX();
    assertTrue(exitX < firstStageX - 0.5);

    assertFalse(manager.updateStagedGoal(new Translation2d(exitX, 4.0), obstacles));
    assertFalse(manager.updateStagedGoal(new Translation2d(exitX, 4.0), obstacles));
    assertTrue(manager.updateStagedGoal(new Translation2d(exitX, 4.0), obstacles));

    assertEquals(requested.getX(), manager.getGoalTranslation().getX(), EPS);
    assertEquals(requested.getY(), manager.getGoalTranslation().getY(), EPS);
  }

  @Test
  void nearGateStillStagesWhenGateOccludesPath() {
    double mid = Constants.FIELD_LENGTH * 0.5;
    Translation2d gateCenter = new Translation2d(mid - 3.6, 4.0);
    GatedAttractorObstacle gate =
        gate(gateCenter, new Translation2d(gateCenter.getX() + 0.2, gateCenter.getY()));
    FieldPlannerGoalManager manager = new FieldPlannerGoalManager(List.of(gate));
    List<? extends Obstacle> obstacles = List.of(gate);

    Pose2d requested = new Pose2d(new Translation2d(mid - 6.0, 4.0), Rotation2d.kZero);
    manager.setRequestedGoal(requested);

    // Robot is already close to the staging point, but the gate still occludes the path.
    // Staging should still be active rather than being skipped.
    assertFalse(
        manager.updateStagedGoal(new Translation2d(gateCenter.getX() + 0.33, 4.0), obstacles));
    assertTrue(Math.abs(manager.getGoalTranslation().getX() - requested.getX()) > EPS);
  }

  private static GatedAttractorObstacle gate(Translation2d center, Translation2d bypassPoint) {
    Translation2d[] gatePoly =
        new Translation2d[] {
          new Translation2d(center.getX() - 0.30, center.getY() - 0.30),
          new Translation2d(center.getX() + 0.30, center.getY() - 0.30),
          new Translation2d(center.getX() + 0.30, center.getY() + 0.30),
          new Translation2d(center.getX() - 0.30, center.getY() + 0.30)
        };

    return gate(center, bypassPoint, gatePoly);
  }

  private static GatedAttractorObstacle gate(
      Translation2d center, Translation2d bypassPoint, Translation2d[] gatePoly) {
    return new GatedAttractorObstacle(center, 1.0, 5.0, gatePoly, bypassPoint, 1.0, 5.0, true);
  }
}
