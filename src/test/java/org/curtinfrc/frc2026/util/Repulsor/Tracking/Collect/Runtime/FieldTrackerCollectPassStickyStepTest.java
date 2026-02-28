package org.curtinfrc.frc2026.util.Repulsor.Tracking.Collect.Runtime;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import java.util.function.Function;
import java.util.function.Predicate;
import org.curtinfrc.frc2026.util.Repulsor.Constants;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.PointCandidate;
import org.junit.jupiter.api.Test;

class FieldTrackerCollectPassStickyStepTest {
  private static final double EPS = 1e-9;
  private static final FieldTrackerCollectPassContext CTX =
      new FieldTrackerCollectPassContext(
          Pose2d.kZero,
          Translation2d.kZero,
          3.0,
          new Translation2d[0],
          false,
          List.of(),
          List.of(),
          0,
          0,
          0L,
          0.02,
          8.0,
          3.0,
          4.0,
          12.0,
          13.0,
          p -> p,
          p -> false,
          p -> false,
          p -> p,
          p -> p,
          p -> Pose2d.kZero,
          0,
          null);

  @Test
  void preferRankedCandidateForStickyPromotesBetterRankedPoint() {
    Translation2d current = new Translation2d(2.0, 1.0);
    Translation2d ranked = new Translation2d(2.4, 1.0);

    Predicate<Translation2d> valid = p -> true;
    Function<Translation2d, Double> score =
        p -> Math.abs(p.getX() - ranked.getX()) < 1e-9 ? 2.0 : 1.8;

    FieldTrackerCollectPassCandidateResult cand =
        new FieldTrackerCollectPassCandidateResult(
            new PointCandidate(ranked, Rotation2d.kZero, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            current,
            valid,
            valid,
            score,
            null);

    Translation2d out =
        FieldTrackerCollectPassStickyStep.preferRankedCandidateForSticky(current, cand, CTX);

    assertEquals(ranked.getX(), out.getX(), EPS);
    assertEquals(ranked.getY(), out.getY(), EPS);
  }

  @Test
  void preferRankedCandidateForStickyKeepsCurrentWhenRankedNotBetter() {
    Translation2d current = new Translation2d(2.0, 1.0);
    Translation2d ranked = new Translation2d(2.4, 1.0);

    Predicate<Translation2d> valid = p -> true;
    Function<Translation2d, Double> score =
        p -> Math.abs(p.getX() - ranked.getX()) < 1e-9 ? 1.82 : 1.8;

    FieldTrackerCollectPassCandidateResult cand =
        new FieldTrackerCollectPassCandidateResult(
            new PointCandidate(ranked, Rotation2d.kZero, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            current,
            valid,
            valid,
            score,
            null);

    Translation2d out =
        FieldTrackerCollectPassStickyStep.preferRankedCandidateForSticky(current, cand, CTX);

    assertSame(current, out);
  }

  @Test
  void preferRankedCandidateForStickyKeepsNonTrapCandidateOverTrapRankedPoint() {
    double cy = Constants.FIELD_WIDTH * 0.5;
    Translation2d nonTrap = new Translation2d(7.0, cy);
    Translation2d trapRanked = new Translation2d(4.2, cy);

    Predicate<Translation2d> valid = p -> true;
    Function<Translation2d, Double> score =
        p -> Math.abs(p.getX() - trapRanked.getX()) < 1e-9 ? 3.0 : 1.0;

    FieldTrackerCollectPassCandidateResult cand =
        new FieldTrackerCollectPassCandidateResult(
            new PointCandidate(trapRanked, Rotation2d.kZero, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            nonTrap,
            valid,
            valid,
            score,
            null);

    Translation2d out =
        FieldTrackerCollectPassStickyStep.preferRankedCandidateForSticky(nonTrap, cand, CTX);

    assertSame(nonTrap, out);
  }

  @Test
  void shouldBlockFarSwitchBlocksWhenFarAndUpgradeNotLarge() {
    boolean blocked =
        FieldTrackerCollectPassStickyStep.shouldBlockFarSwitch(
            3.4, 1.00, 1.18, 0.12, false, false, true, true, 0.0, 0.5, 0.2);
    assertTrue(blocked);
  }

  @Test
  void shouldBlockFarSwitchAllowsTrapEscapeAndBigUpgrade() {
    boolean trapEscape =
        FieldTrackerCollectPassStickyStep.shouldBlockFarSwitch(
            3.4, 1.00, 1.10, 0.12, true, false, true, true, 0.0, 0.5, 0.2);
    boolean largeUpgrade =
        FieldTrackerCollectPassStickyStep.shouldBlockFarSwitch(
            3.4, 1.00, 1.30, 0.12, false, false, true, true, 0.0, 0.5, 0.2);

    assertFalse(trapEscape);
    assertFalse(largeUpgrade);
  }

  @Test
  void shouldBlockFarSwitchReleasesWhenStillOrNoProgress() {
    boolean stillRelease =
        FieldTrackerCollectPassStickyStep.shouldBlockFarSwitch(
            3.4, 1.00, 1.18, 0.12, false, false, true, true, 0.35, 0.5, 0.2);
    boolean noProgressRelease =
        FieldTrackerCollectPassStickyStep.shouldBlockFarSwitch(
            3.4, 1.00, 1.18, 0.12, false, false, true, true, 0.0, 0.05, 0.7);

    assertFalse(stillRelease);
    assertFalse(noProgressRelease);
  }

  @Test
  void adaptSwitchMarginForDistanceReducesMarginWhenClose() {
    double out = FieldTrackerCollectPassStickyStep.adaptSwitchMarginForDistance(0.20, 1.0);
    assertEquals(0.11, out, EPS);
  }

  @Test
  void shouldHoldPreviousForTooSoonDoesNotHoldWhenClose() {
    boolean hold =
        FieldTrackerCollectPassStickyStep.shouldHoldPreviousForTooSoon(true, 0.01, false, 1.0);
    assertFalse(hold);
  }
}
