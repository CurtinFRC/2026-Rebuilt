package org.curtinfrc.frc2026.util.Repulsor.Tracking.Collect.Runtime;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.function.Function;
import java.util.function.Predicate;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.PointCandidate;
import org.junit.jupiter.api.Test;

class FieldTrackerCollectPassStickyStepTest {
  private static final double EPS = 1e-9;

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
        FieldTrackerCollectPassStickyStep.preferRankedCandidateForSticky(current, cand);

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
        FieldTrackerCollectPassStickyStep.preferRankedCandidateForSticky(current, cand);

    assertSame(current, out);
  }
}
