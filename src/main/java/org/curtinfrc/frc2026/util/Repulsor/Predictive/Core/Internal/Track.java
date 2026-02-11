package org.curtinfrc.frc2026.util.Repulsor.Predictive.Core.Internal;

import edu.wpi.first.math.geometry.Translation2d;

public final class Track {
  public Translation2d pos;
  public Translation2d vel;
  public double speedCap;
  public double lastTs;

  public Track(Translation2d p, Translation2d v, double cap, double t) {
    pos = p != null ? p : new Translation2d();
    vel = v != null ? v : new Translation2d();
    speedCap = cap;
    lastTs = t;
  }
}
