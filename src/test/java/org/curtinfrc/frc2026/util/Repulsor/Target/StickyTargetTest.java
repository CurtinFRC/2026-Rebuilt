package org.curtinfrc.frc2026.util.Repulsor.Target;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;
import java.util.function.Predicate;
import java.util.function.ToDoubleBiFunction;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;

class StickyTargetTest {
  private static final double EPS = 1e-9;
  private static final double START = 0.0;
  private static final double CLOSE = 0.12;
  private static final double MID = 1.116;
  private static final double USER_CASE = 0.707;
  private static final double FAR = 0.80;
  private static final ToDoubleBiFunction<Double, Double> DIST = (a, b) -> Math.abs(a - b);

  @Test
  void closeCandidateSwitchesWithModestAdvantage() {
    Fixture fx = new Fixture();
    fx.setScore(START, 1.00);

    assertEquals(START, fx.update(START), EPS);
    fx.advance(0.08);
    assertEquals(START, fx.update(START), EPS);

    fx.setScore(CLOSE, 1.08);
    fx.advance(0.02);
    assertEquals(START, fx.update(CLOSE), EPS);
    fx.advance(0.08);
    assertEquals(START, fx.update(CLOSE), EPS);

    fx.advance(0.16);
    assertEquals(CLOSE, fx.update(CLOSE), EPS);
  }

  @Test
  void farCandidateWithSameAdvantageDoesNotSwitchEarly() {
    Fixture fx = new Fixture();
    fx.setScore(START, 1.00);

    assertEquals(START, fx.update(START), EPS);
    fx.advance(0.08);
    assertEquals(START, fx.update(START), EPS);

    fx.setScore(FAR, 1.08);
    fx.advance(0.02);
    assertEquals(START, fx.update(FAR), EPS);
    fx.advance(0.08);
    assertEquals(START, fx.update(FAR), EPS);

    fx.advance(0.16);
    assertEquals(START, fx.update(FAR), EPS);

    fx.advance(0.30);
    assertEquals(START, fx.update(FAR), EPS);
  }

  @Test
  void switchBackRespectsCooldown() {
    Fixture fx = new Fixture();
    fx.setScore(START, 1.00);

    assertEquals(START, fx.update(START), EPS);
    fx.advance(0.08);
    assertEquals(START, fx.update(START), EPS);

    fx.setScore(CLOSE, 1.10);
    fx.advance(0.02);
    assertEquals(START, fx.update(CLOSE), EPS);
    fx.advance(0.08);
    assertEquals(START, fx.update(CLOSE), EPS);
    fx.advance(0.16);
    assertEquals(CLOSE, fx.update(CLOSE), EPS);

    fx.setScore(START, 1.20);
    fx.setScore(CLOSE, 1.00);
    fx.advance(0.06);
    assertEquals(CLOSE, fx.update(START), EPS);
    fx.advance(0.20);
    assertEquals(CLOSE, fx.update(START), EPS);
  }

  @Test
  void invalidStickyCanSwitchEvenIfSwitchBackWouldBeBlocked() {
    Fixture fx = new Fixture();
    fx.setScore(START, 1.00);
    fx.setScore(CLOSE, 1.30);
    fx.setValid(START, true);
    fx.setValid(CLOSE, true);

    assertEquals(START, fx.update(START), EPS);
    fx.advance(0.08);
    assertEquals(START, fx.update(START), EPS);

    fx.advance(0.02);
    assertEquals(START, fx.update(CLOSE), EPS);
    fx.advance(0.08);
    assertEquals(START, fx.update(CLOSE), EPS);
    fx.advance(0.16);
    assertEquals(CLOSE, fx.update(CLOSE), EPS);

    fx.setScore(START, 1.20);
    fx.advance(0.06);
    assertEquals(CLOSE, fx.update(START), EPS);
    fx.advance(0.20);
    assertEquals(CLOSE, fx.update(START), EPS);

    fx.setValid(CLOSE, false);
    assertEquals(CLOSE, fx.update(START), EPS);
    fx.advance(0.30);
    assertEquals(START, fx.update(START), EPS);
  }

  @Test
  void collectLikeMidDistanceCandidateCanSwitch() {
    Fixture fx = new Fixture();
    fx.setScore(START, 1.00);

    assertEquals(START, fx.updateCollectLike(START), EPS);
    fx.advance(0.08);
    assertEquals(START, fx.updateCollectLike(START), EPS);

    fx.setScore(MID, 1.50);
    fx.advance(0.02);
    assertEquals(START, fx.updateCollectLike(MID), EPS);
    fx.advance(0.08);
    assertEquals(START, fx.updateCollectLike(MID), EPS);
    fx.advance(0.22);
    assertEquals(MID, fx.updateCollectLike(MID), EPS);
  }

  @Test
  void collectLikeUserCaseDistanceCanSwitch() {
    Fixture fx = new Fixture();
    fx.setScore(START, 1.00);

    assertEquals(START, fx.updateCollectLike(START), EPS);
    fx.advance(0.08);
    assertEquals(START, fx.updateCollectLike(START), EPS);

    fx.setScore(USER_CASE, 1.55);
    fx.advance(0.02);
    assertEquals(START, fx.updateCollectLike(USER_CASE), EPS);
    fx.advance(0.08);
    assertEquals(START, fx.updateCollectLike(USER_CASE), EPS);
    fx.advance(0.22);
    assertEquals(USER_CASE, fx.updateCollectLike(USER_CASE), EPS);
  }

  @Test
  void doesNotSwitchBeforeDwellTimeEvenIfCandidateMuchBetter() {
    Fixture fx = new Fixture();
    fx.setScore(START, 1.00);
    fx.setScore(CLOSE, 10.00);

    assertEquals(START, fx.update(START), EPS);
    fx.advance(0.08);
    assertEquals(START, fx.update(START), EPS);

    assertEquals(START, fx.update(CLOSE), EPS);
    tick(fx, CLOSE, 0.01, 29, false);
    assertEquals(START, fx.update(CLOSE), EPS);

    fx.advance(0.02);
    assertEquals(CLOSE, fx.update(CLOSE), EPS);
  }

  @Test
  void candidateMustRemainBestLongEnough_jitterResetsProgress() {
    Fixture fx = new Fixture();
    fx.setScore(START, 1.00);
    fx.setScore(CLOSE, 1.50);
    fx.setScore(FAR, 1.60);

    assertEquals(START, fx.update(START), EPS);

    for (int i = 0; i < 40; i++) {
      fx.advance(0.01);
      double best = (i % 2 == 0) ? CLOSE : FAR;
      assertEquals(START, fx.update(best), EPS);
    }
  }

  @Test
  void equalScoreDoesNotCauseSwitch() {
    Fixture fx = new Fixture();
    fx.setScore(START, 1.00);
    fx.setScore(CLOSE, 1.00);

    assertEquals(START, fx.update(START), EPS);
    tick(fx, CLOSE, 0.02, 50, false);
    assertEquals(START, fx.update(CLOSE), EPS);
  }

  @Test
  void candidateValidityFlickerPreventsSwitch() {
    Fixture fx = new Fixture();
    fx.setScore(START, 1.00);
    fx.setScore(CLOSE, 2.00);
    fx.setValid(START, true);
    fx.setValid(CLOSE, true);

    assertEquals(START, fx.update(START), EPS);
    fx.advance(0.08);
    assertEquals(START, fx.update(START), EPS);

    assertEquals(START, fx.update(CLOSE), EPS);

    for (int i = 0; i < 60; i++) {
      fx.advance(0.01);
      fx.setValid(CLOSE, (i % 3) != 0);
      assertEquals(START, fx.update(CLOSE), EPS);
    }
  }

  @Test
  void candidateValidityBecomesStable_thenCanSwitch() {
    Fixture fx = new Fixture();
    fx.setScore(START, 1.00);
    fx.setScore(CLOSE, 2.00);
    fx.setValid(START, true);
    fx.setValid(CLOSE, true);

    assertEquals(START, fx.update(START), EPS);
    fx.advance(0.08);
    assertEquals(START, fx.update(START), EPS);

    assertEquals(START, fx.update(CLOSE), EPS);

    for (int i = 0; i < 30; i++) {
      fx.advance(0.01);
      fx.setValid(CLOSE, (i % 2) == 0);
      assertEquals(START, fx.update(CLOSE), EPS);
    }

    fx.setValid(CLOSE, true);
    tick(fx, CLOSE, 0.02, 20, false);
    assertEquals(CLOSE, fx.update(CLOSE), EPS);
  }

  @Test
  void nullBestDoesNotCrashAndKeepsSticky() {
    Fixture fx = new Fixture();
    fx.setScore(START, 1.00);

    assertEquals(START, fx.update(START), EPS);
    fx.advance(0.10);
    assertEquals(START, fx.update(null), EPS);
  }

  @ParameterizedTest
  @CsvSource({"0.12, 1.08, true", "0.80, 1.08, false", "0.707, 1.55, true"})
  void candidateSwitchDecisionGrid(double dist, double candidateScore, boolean shouldSwitch) {
    Fixture fx = new Fixture();
    fx.setScore(START, 1.00);
    fx.setScore(dist, candidateScore);

    assertEquals(START, fx.updateCollectLike(START), EPS);
    tick(fx, dist, 0.02, 40, true);
    double got = fx.updateCollectLike(dist);

    if (shouldSwitch) {
      assertEquals(dist, got, EPS);
    } else {
      assertEquals(START, got, EPS);
    }
  }

  @Test
  void higherCandidateScoreDoesNotDelaySwitch() {
    Fixture fx1 = new Fixture();
    Fixture fx2 = new Fixture();

    fx1.setScore(START, 1.0);
    fx2.setScore(START, 1.0);

    fx1.setScore(CLOSE, 1.2);
    fx2.setScore(CLOSE, 2.0);

    assertEquals(START, fx1.update(START), EPS);
    assertEquals(START, fx2.update(START), EPS);
    fx1.advance(0.08);
    fx2.advance(0.08);
    assertEquals(START, fx1.update(START), EPS);
    assertEquals(START, fx2.update(START), EPS);

    assertEquals(START, fx1.update(CLOSE), EPS);
    assertEquals(START, fx2.update(CLOSE), EPS);

    double t1 = timeToSwitch(fx1, CLOSE, false);
    double t2 = timeToSwitch(fx2, CLOSE, false);

    assertTrue(t2 <= t1 + 1e-9);
  }

  private double tick(Fixture fx, double best, double dt, int n, boolean collectLike) {
    double sticky = Double.NaN;
    for (int i = 0; i < n; i++) {
      fx.advance(dt);
      sticky = collectLike ? fx.updateCollectLike(best) : fx.update(best);
    }
    return sticky;
  }

  private double timeToSwitch(Fixture fx, double best, boolean collectLike) {
    double start = fx.clock.nowSec;
    for (int i = 0; i < 200; i++) {
      fx.advance(0.01);
      double out = collectLike ? fx.updateCollectLike(best) : fx.update(best);
      if (Double.compare(out, best) == 0) return fx.clock.nowSec - start;
    }
    return Double.POSITIVE_INFINITY;
  }

  private static final class Fixture {
    private final ManualClock clock = new ManualClock();
    private final Map<Double, Double> scores = new HashMap<>();
    private final Map<Double, Boolean> valids = new HashMap<>();
    private final StickyTarget<Double> sticky = new StickyTarget<>(0.30, 1.0, 0.50, clock);

    private void setScore(double point, double score) {
      scores.put(point, score);
    }

    private void setValid(double point, boolean valid) {
      valids.put(point, valid);
    }

    private double update(Double best) {
      return sticky.update(
          best, score(best), this::score, valid(), 0.30, 0.35, 0.60, null, DIST, 0.05, 0.0, 0.0);
    }

    private double updateCollectLike(Double best) {
      return sticky.update(
          best, score(best), this::score, valid(), 3.80, 1.45, 2.68, null, DIST, 0.45, 0.0, 0.0);
    }

    private Predicate<Double> valid() {
      return x -> x != null && valids.getOrDefault(x, true);
    }

    private double score(Double point) {
      if (point == null) return 0.0;
      return scores.getOrDefault(point, 0.0);
    }

    private void advance(double dtSec) {
      clock.nowSec += dtSec;
    }
  }

  private static final class ManualClock implements DoubleSupplier {
    private double nowSec = 0.0;

    @Override
    public double getAsDouble() {
      return nowSec;
    }
  }
}
