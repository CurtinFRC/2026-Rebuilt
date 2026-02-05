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


package org.curtinfrc.frc2026.util.Repulsor.Vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;

public class VisionCustom implements RepulsorVision {
  private final NetworkTable table;

  public VisionCustom() {
    this.table = NetworkTableInstance.getDefault().getTable("RepulsorVision");
  }

  @Override
  public Obstacle[] getObstacles() {
    Set<String> keys = table.getKeys();
    List<Obstacle> out = new ArrayList<>();
    for (String key : keys) {
      if (!key.startsWith("obs_")) continue;
      String base = key;
      String kindStr = table.getEntry(base + "/kind").getString("kUnknown");
      Kind kind;
      try {
        kind = Kind.valueOf(kindStr);
      } catch (Exception e) {
        kind = Kind.kUnknown;
      }
      double x = table.getEntry(base + "/x").getDouble(0.0);
      double y = table.getEntry(base + "/y").getDouble(0.0);
      double sx = table.getEntry(base + "/size_x").getDouble(0.4);
      double sy = table.getEntry(base + "/size_y").getDouble(0.4);
      ObstacleType ot = new ObstacleType(sx, sy, kind);
      out.add(new Obstacle(x, y, ot));
    }
    return out.toArray(new Obstacle[0]);
  }
}

