package org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacles;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Translation2d;
import org.junit.jupiter.api.Test;

class CorridorCenterlineRailTest {
  private static final double EPS = 1e-9;

  @Test
  void hasDeadbandAroundCenterline() {
    CorridorCenterlineRail rail = new CorridorCenterlineRail(5.0, 1.0, 4.0, 0.40, 2.0, 3.0);

    var f = rail.getForceAtPosition(new Translation2d(5.0, 4.05), new Translation2d(8.0, 4.0));
    assertEquals(0.0, f.getNorm(), EPS);
  }

  @Test
  void appliesRestoringForceOutsideDeadband() {
    CorridorCenterlineRail rail = new CorridorCenterlineRail(5.0, 1.0, 4.0, 0.40, 2.0, 3.0);

    var high = rail.getForceAtPosition(new Translation2d(5.0, 4.20), new Translation2d(8.0, 4.0));
    var low = rail.getForceAtPosition(new Translation2d(5.0, 3.80), new Translation2d(8.0, 4.0));

    assertTrue(high.getY() < 0.0);
    assertTrue(low.getY() > 0.0);
  }

  @Test
  void noForceOutsideXWindow() {
    CorridorCenterlineRail rail = new CorridorCenterlineRail(5.0, 1.0, 4.0, 0.40, 2.0, 3.0);

    var f = rail.getForceAtPosition(new Translation2d(6.1, 4.25), new Translation2d(8.0, 4.0));
    assertEquals(0.0, f.getNorm(), EPS);
  }
}
