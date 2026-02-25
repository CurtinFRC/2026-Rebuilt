package org.curtinfrc.frc2026.util.Repulsor.Tracking.Collect.Runtime;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.function.Function;
import java.util.function.Predicate;
import org.junit.jupiter.api.Test;

class FieldTrackerCollectPassCandidateStepTest {
  private static final double EPS = 1e-9;

  @Test
  void maybeCanonicalizeCandidateRejectsInvalidCanonicalPoint() {
    Translation2d best = new Translation2d(2.00, 1.00);
    Translation2d canonical = new Translation2d(2.30, 1.00);

    Predicate<Translation2d> collectValid = p -> Math.abs(p.getX() - best.getX()) < 1e-9;
    Function<Translation2d, Double> score = p -> 10.0;

    Translation2d out =
        FieldTrackerCollectPassCandidateStep.maybeCanonicalizeCandidate(
            best, new Translation2d[] {canonical}, collectValid, score);

    assertSame(best, out);
  }

  @Test
  void maybeCanonicalizeCandidateAcceptsValidCanonicalPointWithSimilarScore() {
    Translation2d best = new Translation2d(2.00, 1.00);
    Translation2d canonical = new Translation2d(2.28, 1.00);

    Predicate<Translation2d> collectValid = p -> true;
    Function<Translation2d, Double> score =
        p -> Math.abs(p.getX() - canonical.getX()) < 1e-9 ? 9.95 : 10.0;

    Translation2d out =
        FieldTrackerCollectPassCandidateStep.maybeCanonicalizeCandidate(
            best, new Translation2d[] {canonical}, collectValid, score);

    assertEquals(canonical.getX(), out.getX(), EPS);
    assertEquals(canonical.getY(), out.getY(), EPS);
  }

  @Test
  void maybeCanonicalizeCandidateRejectsLargeScoreRegression() {
    Translation2d best = new Translation2d(2.00, 1.00);
    Translation2d canonical = new Translation2d(2.25, 1.00);

    Predicate<Translation2d> collectValid = p -> true;
    Function<Translation2d, Double> score =
        p -> Math.abs(p.getX() - canonical.getX()) < 1e-9 ? 9.7 : 10.0;

    Translation2d out =
        FieldTrackerCollectPassCandidateStep.maybeCanonicalizeCandidate(
            best, new Translation2d[] {canonical}, collectValid, score);

    assertSame(best, out);
  }

  @Test
  void preferRicherCandidatePromotesHigherFuelWhenTravelIsComparable() {
    Translation2d robot = new Translation2d(1.0, 1.0);
    Translation2d base = new Translation2d(2.0, 1.0);
    Translation2d rich = new Translation2d(2.3, 1.0);

    Predicate<Translation2d> collectValid = p -> true;
    Function<Translation2d, Double> units =
        p -> Math.abs(p.getX() - rich.getX()) < 1e-9 ? 0.28 : 0.10;
    Function<Translation2d, Double> score =
        p -> Math.abs(p.getX() - rich.getX()) < 1e-9 ? 1.85 : 1.90;

    Translation2d out =
        FieldTrackerCollectPassCandidateStep.preferRicherCandidate(
            base, new Translation2d[] {rich}, robot, 3.0, collectValid, units, score);

    assertEquals(rich.getX(), out.getX(), EPS);
    assertEquals(rich.getY(), out.getY(), EPS);
  }

  @Test
  void preferRicherCandidateKeepsBaseWhenRicherWouldDetourTooFar() {
    Translation2d robot = new Translation2d(1.0, 1.0);
    Translation2d base = new Translation2d(2.0, 1.0);
    Translation2d farRich = new Translation2d(5.6, 1.0);

    Predicate<Translation2d> collectValid = p -> true;
    Function<Translation2d, Double> units =
        p -> Math.abs(p.getX() - farRich.getX()) < 1e-9 ? 0.35 : 0.10;
    Function<Translation2d, Double> score =
        p -> Math.abs(p.getX() - farRich.getX()) < 1e-9 ? 1.95 : 2.00;

    Translation2d out =
        FieldTrackerCollectPassCandidateStep.preferRicherCandidate(
            base, new Translation2d[] {farRich}, robot, 3.0, collectValid, units, score);

    assertSame(base, out);
  }
}
