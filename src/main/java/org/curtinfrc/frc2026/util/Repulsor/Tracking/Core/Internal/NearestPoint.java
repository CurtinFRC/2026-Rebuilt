package org.curtinfrc.frc2026.util.Repulsor.Tracking.Core.Internal;

import edu.wpi.first.math.geometry.Translation2d;

public final class NearestPoint {
  public final Translation2d p;
  public final double d;

  public NearestPoint(Translation2d p, double d) {
    this.p = p;
    this.d = d;
  }
}
