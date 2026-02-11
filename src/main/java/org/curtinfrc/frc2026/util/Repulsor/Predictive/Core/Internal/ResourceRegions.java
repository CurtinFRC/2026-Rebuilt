package org.curtinfrc.frc2026.util.Repulsor.Predictive.Core.Internal;

import edu.wpi.first.math.geometry.Translation2d;

public final class ResourceRegions {
  public final Translation2d[] centers;
  public final double[] mass;

  public ResourceRegions(Translation2d[] centers, double[] mass) {
    this.centers = centers != null ? centers : new Translation2d[0];
    this.mass = mass != null ? mass : new double[this.centers.length];
  }
}
