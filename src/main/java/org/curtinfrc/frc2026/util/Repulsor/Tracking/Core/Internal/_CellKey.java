package org.curtinfrc.frc2026.util.Repulsor.Tracking.Core.Internal;

public final class _CellKey {
  private final int ix;
  private final int iy;

  public _CellKey(int ix, int iy) {
    this.ix = ix;
    this.iy = iy;
  }

  @Override
  public int hashCode() {
    return (ix * 73856093) ^ (iy * 19349663);
  }

  @Override
  public boolean equals(Object o) {
    if (this == o) return true;
    if (!(o instanceof _CellKey)) return false;
    _CellKey k = (_CellKey) o;
    return ix == k.ix && iy == k.iy;
  }
}
