package org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Helpers;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
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

  private static GatedAttractorObstacle gate(Translation2d center, Translation2d bypassPoint) {
    Translation2d[] gatePoly =
        new Translation2d[] {
          new Translation2d(center.getX() - 0.30, center.getY() - 0.30),
          new Translation2d(center.getX() + 0.30, center.getY() - 0.30),
          new Translation2d(center.getX() + 0.30, center.getY() + 0.30),
          new Translation2d(center.getX() - 0.30, center.getY() + 0.30)
        };

    return new GatedAttractorObstacle(center, 1.0, 5.0, gatePoly, bypassPoint, 1.0, 5.0, true);
  }
}
