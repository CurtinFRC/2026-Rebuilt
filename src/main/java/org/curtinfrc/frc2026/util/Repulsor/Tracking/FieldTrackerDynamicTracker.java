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
package org.curtinfrc.frc2026.util.Repulsor.Tracking;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ConcurrentHashMap;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.DynamicObject;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Internal.TrackedObj;

final class FieldTrackerDynamicTracker {
  private final ConcurrentHashMap<String, TrackedObj> tracked = new ConcurrentHashMap<>();

  void ingestTracked(String id, String type, Pose3d p, long nowNs) {
    if (id == null || id.isEmpty() || p == null) return;
    TrackedObj st = tracked.computeIfAbsent(id, TrackedObj::new);
    Pose3d prev = st.pos;
    long t0 = st.tNs;
    st.prev = prev;
    st.pos = p;
    st.type = type != null ? type : "unknown";
    st.tNs = nowNs;

    if (prev != null && t0 != 0L && nowNs > t0) {
      double dt = (nowNs - t0) / 1e9;
      if (dt > 1e-6) {
        st.vx = (p.getX() - prev.getX()) / dt;
        st.vy = (p.getY() - prev.getY()) / dt;
        st.vz = (p.getZ() - prev.getZ()) / dt;
      }
    }
  }

  List<DynamicObject> snapshotDynamics() {
    long nowNs = System.nanoTime();
    boolean isSim = RobotBase.isSimulation();

    ArrayList<DynamicObject> out = new ArrayList<>(tracked.size());
    for (TrackedObj o : tracked.values()) {
      if (o == null) continue;
      Pose3d p = o.pos;
      if (p == null) continue;
      long t = o.tNs;
      if (t == 0L) continue;

      double ageS = (nowNs - t) / 1e9;
      if (ageS < 0.0) continue;

      String ty = o.type != null ? o.type : "unknown";
      if (isSim) ageS = 0.0;

      out.add(
          new DynamicObject(
              o.id,
              ty,
              new Translation2d(p.getX(), p.getY()),
              new Translation2d(o.vx, o.vy),
              ageS));
    }
    return out;
  }
}
