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
    Translation2d entry = manager.getGoalTranslation();
    assertTrue(entry.getX() < gate.center.getX());
    assertEquals(4.0, manager.getGoalTranslation().getY(), EPS);

    boolean sawExit = false;
    for (int i = 0; i < 8; i++) {
      assertFalse(manager.updateStagedGoal(entry, obstacles));
      if (manager.getGoalTranslation().getX() > gate.center.getX() + 0.05) {
        sawExit = true;
        break;
      }
    }
    assertTrue(sawExit);

    boolean done = false;
    for (int i = 0; i < 8; i++) {
      done = manager.updateStagedGoal(manager.getGoalTranslation(), obstacles);
      if (done) break;
    }

    assertTrue(done);
    assertEquals(requested.getX(), manager.getGoalTranslation().getX(), EPS);
    assertEquals(requested.getY(), manager.getGoalTranslation().getY(), EPS);
  }

  @Test
  void restagesWithinRestageDistanceWhenGateStillOccludesPath() {
    GatedAttractorObstacle gate = gate(new Translation2d(8.0, 4.0), new Translation2d(8.2, 4.0));
    FieldPlannerGoalManager manager = new FieldPlannerGoalManager(List.of(gate));
    List<? extends Obstacle> obstacles = List.of(gate);

    Pose2d requested = new Pose2d(new Translation2d(4.0, 4.0), Rotation2d.kZero);
    manager.setRequestedGoal(requested);

    assertFalse(manager.updateStagedGoal(new Translation2d(10.0, 4.0), obstacles));
    boolean firstDone = false;
    for (int i = 0; i < 6; i++) {
      firstDone = manager.updateStagedGoal(manager.getGoalTranslation(), obstacles);
      if (firstDone) break;
    }
    assertTrue(firstDone);

    assertFalse(manager.updateStagedGoal(new Translation2d(9.0, 4.0), obstacles));
    assertTrue(Math.abs(manager.getGoalTranslation().getX() - requested.getX()) > EPS);
  }

  @Test
  void releasesStageAfterGateClearWithoutExactWaypointHit() {
    GatedAttractorObstacle gate = gate(new Translation2d(8.0, 4.0), new Translation2d(8.2, 5.0));
    FieldPlannerGoalManager manager = new FieldPlannerGoalManager(List.of(gate));
    List<? extends Obstacle> obstacles = List.of(gate);

    Pose2d requested = new Pose2d(new Translation2d(12.0, 4.0), Rotation2d.kZero);
    manager.setRequestedGoal(requested);

    assertFalse(manager.updateStagedGoal(new Translation2d(6.0, 4.0), obstacles));
    Translation2d entry = manager.getGoalTranslation();
    assertTrue(entry.getX() < gate.center.getX());

    boolean sawExit = false;
    for (int i = 0; i < 8; i++) {
      assertFalse(manager.updateStagedGoal(entry, obstacles));
      if (manager.getGoalTranslation().getX() > gate.center.getX() + 0.05) {
        sawExit = true;
        break;
      }
    }
    assertTrue(sawExit);

    Translation2d exit = manager.getGoalTranslation();
    Translation2d through = new Translation2d(exit.getX() + 0.12, exit.getY());
    boolean done = false;
    for (int i = 0; i < 6; i++) {
      done = manager.updateStagedGoal(through, obstacles);
      if (done) break;
    }
    assertTrue(done);
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

    boolean firstDone = false;
    for (int i = 0; i < 8; i++) {
      firstDone = manager.updateStagedGoal(manager.getGoalTranslation(), obstacles);
      if (firstDone) break;
    }
    assertTrue(firstDone);

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

    assertFalse(manager.updateStagedGoal(robot, obstacles));
    assertTrue(Math.abs(manager.getGoalTranslation().getX() - requested.getX()) > EPS);
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

    manager.updateStagedGoal(new Translation2d(mid, 4.0), obstacles);
    assertFalse(manager.updateStagedGoal(new Translation2d(mid - 2.0, 4.0), obstacles));
    assertFalse(manager.updateStagedGoal(new Translation2d(firstStageX, 4.0), obstacles));
    assertFalse(manager.updateStagedGoal(new Translation2d(firstStageX, 4.0), obstacles));

    double exitX = manager.getGoalTranslation().getX();
    assertTrue(exitX < firstStageX - 0.5);

    assertFalse(manager.updateStagedGoal(new Translation2d(exitX, 4.0), obstacles));
    boolean done = false;
    for (int i = 0; i < 6; i++) {
      done = manager.updateStagedGoal(new Translation2d(exitX, 4.0), obstacles);
      if (done) break;
    }
    assertTrue(done);

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

  @Test
  void doesNotSuppressImmediateExitRestageAfterFirstCorridorStage() {
    double mid = Constants.FIELD_LENGTH * 0.5;

    GatedAttractorObstacle entryGate =
        gate(new Translation2d(mid - 3.6, 4.0), new Translation2d(mid - 3.4, 4.0));
    GatedAttractorObstacle exitWaypoint =
        new GatedAttractorObstacle(
            new Translation2d(mid + 3.6, 4.0), 1.0, 5.0, null, null, 1.0, 5.0, true);

    FieldPlannerGoalManager manager = new FieldPlannerGoalManager(List.of(entryGate, exitWaypoint));
    List<? extends Obstacle> obstacles = List.of(entryGate, exitWaypoint);

    Pose2d requested = new Pose2d(new Translation2d(mid + 6.0, 4.0), Rotation2d.kZero);
    manager.setRequestedGoal(requested);

    Translation2d start = new Translation2d(mid - 6.0, 4.0);
    assertFalse(manager.updateStagedGoal(start, obstacles));
    Translation2d firstStage = manager.getGoalTranslation();
    assertTrue(firstStage.getX() < entryGate.center.getX());

    boolean sawExit = false;
    for (int i = 0; i < 8; i++) {
      assertFalse(manager.updateStagedGoal(firstStage, obstacles));
      if (manager.getGoalTranslation().getX() > entryGate.center.getX() + 0.5) {
        sawExit = true;
        break;
      }
    }
    assertTrue(sawExit);
  }

  @Test
  void entryStageAdvancesToExitQuicklyWhenRobotGetsNearEntryPoint() {
    GatedAttractorObstacle gate = gate(new Translation2d(8.0, 4.0), new Translation2d(8.2, 4.0));
    FieldPlannerGoalManager manager = new FieldPlannerGoalManager(List.of(gate));
    List<? extends Obstacle> obstacles = List.of(gate);

    Pose2d requested = new Pose2d(new Translation2d(12.0, 4.0), Rotation2d.kZero);
    manager.setRequestedGoal(requested);

    assertFalse(manager.updateStagedGoal(new Translation2d(6.0, 4.0), obstacles));
    Translation2d entry = manager.getGoalTranslation();
    assertTrue(entry.getX() < gate.center.getX());

    Translation2d nearEntry = new Translation2d(entry.getX() + 0.10, entry.getY());
    assertFalse(manager.updateStagedGoal(nearEntry, obstacles));

    assertTrue(
        manager.getGoalTranslation().getX() > gate.center.getX(),
        "Expected immediate advance to exit waypoint after reaching near entry");
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
