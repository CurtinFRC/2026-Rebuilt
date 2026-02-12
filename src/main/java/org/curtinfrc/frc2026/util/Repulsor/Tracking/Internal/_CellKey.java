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
package org.curtinfrc.frc2026.util.Repulsor.Tracking.Internal;

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
