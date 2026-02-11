package org.curtinfrc.frc2026.util.Repulsor.Tracking.Core.Internal;

import edu.wpi.first.math.geometry.Pose3d;

public final class TrackedObj {
  public final String id;
  public volatile String type;
  public volatile Pose3d pos;
  public volatile Pose3d prev;
  public volatile long tNs;
  public volatile double vx;
  public volatile double vy;
  public volatile double vz;

  public TrackedObj(String id) {
    this.id = id;
    this.type = "unknown";
    this.pos = null;
    this.prev = null;
    this.tNs = 0L;
    this.vx = 0.0;
    this.vy = 0.0;
    this.vz = 0.0;
  }
}
