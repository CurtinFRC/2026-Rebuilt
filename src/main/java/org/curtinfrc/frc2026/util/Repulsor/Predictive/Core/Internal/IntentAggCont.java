package org.curtinfrc.frc2026.util.Repulsor.Predictive.Core.Internal;

import edu.wpi.first.math.geometry.Translation2d;

public final class IntentAggCont {
  public final Translation2d[] regions;
  public final double[] intentMass;
  public final int count;
  private final double sigma;

  public IntentAggCont(Translation2d[] regions, double[] intentMass, int count, double sigma) {
    this.regions = regions != null ? regions : new Translation2d[0];
    this.intentMass = intentMass != null ? intentMass : new double[this.regions.length];
    this.count = count;
    this.sigma = Math.max(1e-6, sigma);
  }

  public double intentAt(Translation2d p) {
    if (p == null || regions.length == 0) return 0.0;
    double sum = 0.0;
    double s2 = sigma * sigma;
    for (int i = 0; i < regions.length; i++) {
      Translation2d c = regions[i];
      if (c == null) continue;
      double d = c.getDistance(p);
      double k = Math.exp(-0.5 * (d * d) / Math.max(1e-6, s2));
      sum += intentMass[i] * k;
    }
    return sum;
  }
}
