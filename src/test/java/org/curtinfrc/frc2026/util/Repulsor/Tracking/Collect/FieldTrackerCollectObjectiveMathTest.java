package org.curtinfrc.frc2026.util.Repulsor.Tracking.Collect;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Translation2d;
import org.curtinfrc.frc2026.util.Repulsor.Constants;
import org.junit.jupiter.api.Test;

class FieldTrackerCollectObjectiveMathTest {
  private static final double EPS = 1e-9;

  @Test
  void sideSignXBandReturnsLeftCenterRight() {
    double mid = Constants.FIELD_LENGTH * 0.5;
    double band = 0.10;

    assertEquals(-1, FieldTrackerCollectObjectiveMath.sideSignXBand(mid - 0.30, band));
    assertEquals(0, FieldTrackerCollectObjectiveMath.sideSignXBand(mid, band));
    assertEquals(1, FieldTrackerCollectObjectiveMath.sideSignXBand(mid + 0.30, band));
  }

  @Test
  void canonicalizeCollectPointSnapsNearKnownSetpoint() {
    Translation2d canonical = new Translation2d(4.0, 2.0);
    Translation2d nearby = new Translation2d(4.2, 2.1);
    Translation2d far = new Translation2d(5.0, 2.0);

    Translation2d snapped =
        FieldTrackerCollectObjectiveMath.canonicalizeCollectPoint(
            nearby, new Translation2d[] {canonical});
    Translation2d unchanged =
        FieldTrackerCollectObjectiveMath.canonicalizeCollectPoint(
            far, new Translation2d[] {canonical});

    assertEquals(canonical.getX(), snapped.getX(), EPS);
    assertEquals(canonical.getY(), snapped.getY(), EPS);
    assertEquals(far.getX(), unchanged.getX(), EPS);
    assertEquals(far.getY(), unchanged.getY(), EPS);
  }

  @Test
  void stickySwitchedDetectsGoalChanges() {
    Translation2d a = new Translation2d(2.0, 2.0);
    Translation2d nearA = new Translation2d(2.2, 2.0);
    Translation2d b = new Translation2d(3.2, 2.0);

    assertFalse(FieldTrackerCollectObjectiveMath.stickySwitched(a, nearA));
    assertTrue(FieldTrackerCollectObjectiveMath.stickySwitched(a, b));
  }

  @Test
  void isHubFrontTrapPointDetectsCenterlineTrapNearInnerBandEdge() {
    double leftInner = 4.9;
    double rightInner = 12.7;
    Translation2d trap = new Translation2d(leftInner + 0.30, Constants.FIELD_WIDTH * 0.5);
    Translation2d safeFar =
        new Translation2d((leftInner + rightInner) * 0.5, Constants.FIELD_WIDTH * 0.5);
    Translation2d safeOffCenterline =
        new Translation2d(leftInner + 0.30, Constants.FIELD_WIDTH * 0.5 + 1.6);

    assertTrue(FieldTrackerCollectObjectiveMath.isHubFrontTrapPoint(trap, leftInner, rightInner));
    assertFalse(
        FieldTrackerCollectObjectiveMath.isHubFrontTrapPoint(safeFar, leftInner, rightInner));
    assertFalse(
        FieldTrackerCollectObjectiveMath.isHubFrontTrapPoint(
            safeOffCenterline, leftInner, rightInner));
  }
}
