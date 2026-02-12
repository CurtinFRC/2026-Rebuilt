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
import java.util.ArrayList;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldMapBuilder.CategorySpec;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Internal.ObjectiveCache;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Model.GameElement;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Model.GameElementModel;

final class FieldTrackerObjectiveCache {
  private FieldTrackerObjectiveCache() {}

  static int mixHash(int h, double x, double y) {
    long a = Double.doubleToLongBits(x);
    long b = Double.doubleToLongBits(y);
    h ^= (int) (a ^ (a >>> 32));
    h = (h * 16777619) ^ (int) (b ^ (b >>> 32));
    return h;
  }

  static void rebuildObjectiveCacheForCategory(
      ObjectiveCache cache, CategorySpec cat, GameElement[] fieldMap) {
    GameElement[] fm = fieldMap;
    if (fm == null || fm.length == 0) {
      cache.points = new Translation2d[0];
      cache.lastHash = 0;
      return;
    }
    ArrayList<Translation2d> pts = new ArrayList<>(256);
    int h = 146959810;
    for (GameElement e : fm) {
      if (e == null) continue;
      if (e.getCategory() != cat) continue;
      GameElementModel m = e.getModel();
      if (m == null) continue;
      Pose3d p = m.getPosition();
      if (p == null) continue;
      double x = p.getX();
      double y = p.getY();
      pts.add(new Translation2d(x, y));
      h = mixHash(h, x, y);
    }
    Translation2d[] arr = pts.toArray(new Translation2d[0]);
    cache.points = arr;
    cache.lastHash = h;
  }
}
