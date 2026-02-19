package org.curtinfrc.frc2026.util.Repulsor.Target;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;
import java.util.function.Predicate;
import java.util.function.ToDoubleBiFunction;
import org.junit.jupiter.api.Test;

class StickyTargetTest {
  private static final double EPS = 1e-9;
  private static final double START = 0.0;
  private static final double CLOSE = 0.12;
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

    private double update(double best) {
      return sticky.update(
          best, score(best), this::score, valid(), 0.30, 0.35, 0.60, null, DIST, 0.05, 0.0, 0.0);
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
