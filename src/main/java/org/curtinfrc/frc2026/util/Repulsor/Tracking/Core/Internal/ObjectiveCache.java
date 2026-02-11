package org.curtinfrc.frc2026.util.Repulsor.Tracking.Core.Internal;

import edu.wpi.first.math.geometry.Translation2d;

public final class ObjectiveCache {
  public volatile Translation2d[] points = new Translation2d[0];
  public volatile int lastHash = 0;
}
