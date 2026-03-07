package org.curtinfrc.frc2026.util.Repulsor.Predictive;

public final class PredictiveClock {
  private PredictiveClock() {}

  public static double nowSeconds() {
    return System.nanoTime() * 1.0e-9;
  }
}
