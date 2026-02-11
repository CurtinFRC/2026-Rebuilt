package org.curtinfrc.frc2026.util.Repulsor.Tracking.Core;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldMapBuilder.CategorySpec;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Core.Internal.ObjectiveCache;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Core.Model.GameElement;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Core.Model.GameElementModel;

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
