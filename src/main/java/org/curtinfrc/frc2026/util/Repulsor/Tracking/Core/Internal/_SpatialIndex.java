package org.curtinfrc.frc2026.util.Repulsor.Tracking.Core.Internal;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.HashMap;

public final class _SpatialIndex {
  private final double cell;
  private final HashMap<_CellKey, ArrayList<Translation2d>> map = new HashMap<>();

  public _SpatialIndex(double cell) {
    this.cell = Math.max(1e-6, cell);
  }

  public void add(Translation2d p) {
    int ix = (int) Math.floor(p.getX() / cell);
    int iy = (int) Math.floor(p.getY() / cell);
    _CellKey k = new _CellKey(ix, iy);
    map.computeIfAbsent(k, kk -> new ArrayList<>()).add(p);
  }

  public ArrayList<Translation2d> getCell(int ix, int iy) {
    return map.get(new _CellKey(ix, iy));
  }

  public int ix(double x) {
    return (int) Math.floor(x / cell);
  }

  public int iy(double y) {
    return (int) Math.floor(y / cell);
  }
}
