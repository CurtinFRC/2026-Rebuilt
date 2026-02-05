/*
 * =============================================================
 *  Repulsor Library
 *  Copyright (c) 2026 Paul Hodges
 *
 *  Licensed under the Repulsor Non-Commercial License (RNC-1.0).
 * =============================================================
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

