/*
 * Copyright (C) 2026 Paul Hodges
 *
 * This file is part of Repulsor.
 *
 * Repulsor is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Repulsor is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Repulsor. If not, see https://www.gnu.org/licenses/.
 */
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
