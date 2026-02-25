package org.curtinfrc.frc2026.util.Repulsor.Fields;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Translation2d;
import org.curtinfrc.frc2026.util.Repulsor.Constants;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacle;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacles.GatedAttractorObstacle;
import org.junit.jupiter.api.Test;

class Rebuilt2026ObstacleLayoutTest {
  private static final double EPS = 1e-9;

  @Test
  void gatedBypassPointsKeepRobotClearanceFromGateFace() {
    Rebuilt2026 field = new Rebuilt2026();
    double minClearance =
        0.5
                * Math.max(
                    org.curtinfrc.frc2026.Constants.ROBOT_X,
                    org.curtinfrc.frc2026.Constants.ROBOT_Y)
            + 0.08;

    int checked = 0;
    for (Obstacle obs : field.fieldObstacles()) {
      if (!(obs instanceof GatedAttractorObstacle gate)) continue;
      if (gate.gatePoly == null || gate.bypassPoint == null) continue;

      checked++;
      double d = pointDistanceToPolygonEdges(gate.bypassPoint, gate.gatePoly);
      assertTrue(
          d + EPS >= minClearance,
          "Bypass point too close to gate face: d=" + d + " min=" + minClearance);
    }

    assertTrue(checked > 0, "Expected gated bypass points in Rebuilt2026 field obstacles");
  }

  @Test
  void waypointGatesAreBalancedOnBothCorridorSides() {
    Rebuilt2026 field = new Rebuilt2026();
    double mid = Constants.FIELD_LENGTH * 0.5;

    int left = 0;
    int right = 0;
    for (Obstacle obs : field.fieldObstacles()) {
      if (!(obs instanceof GatedAttractorObstacle gate)) continue;
      if (!gate.waypoint || gate.center == null) continue;
      if (gate.center.getX() < mid) left++;
      else if (gate.center.getX() > mid) right++;
    }

    assertTrue(left > 0, "Expected left-side corridor waypoint gates");
    assertTrue(right > 0, "Expected right-side corridor waypoint gates");
    assertEquals(left, right, "Waypoint gates should be balanced left/right for corridor staging");
  }

  private static double pointDistanceToPolygonEdges(Translation2d p, Translation2d[] poly) {
    double best = Double.POSITIVE_INFINITY;
    for (int i = 0; i < poly.length; i++) {
      Translation2d a = poly[i];
      Translation2d b = poly[(i + 1) % poly.length];
      double d = pointSegmentDist(p, a, b);
      if (d < best) best = d;
    }
    return best;
  }

  private static double pointSegmentDist(Translation2d p, Translation2d a, Translation2d b) {
    double ax = a.getX();
    double ay = a.getY();
    double bx = b.getX();
    double by = b.getY();
    double px = p.getX();
    double py = p.getY();

    double vx = bx - ax;
    double vy = by - ay;
    double wx = px - ax;
    double wy = py - ay;

    double vv = vx * vx + vy * vy;
    if (vv < 1e-12) return Math.hypot(px - ax, py - ay);

    double t = (wx * vx + wy * vy) / vv;
    t = Math.max(0.0, Math.min(1.0, t));
    double cx = ax + vx * t;
    double cy = ay + vy * t;
    return Math.hypot(px - cx, py - cy);
  }
}
