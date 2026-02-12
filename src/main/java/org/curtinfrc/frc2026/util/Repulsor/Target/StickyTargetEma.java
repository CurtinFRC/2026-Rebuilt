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

package org.curtinfrc.frc2026.util.Repulsor.Target;

import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;

final class StickyTargetEma<T> {
  private static final double DEFAULT_TAU_SEC = 0.26;

  private final Map<T, EmaEntry> emaScore = new HashMap<>(8);
  private double tauSec = DEFAULT_TAU_SEC;
  private double evictSec = 3.5;

  void setEvictSec(double sec) {
    if (!Double.isFinite(sec)) return;
    evictSec = Math.max(0.25, sec);
  }

  void clear() {
    emaScore.clear();
  }

  void evict(double now) {
    if (emaScore.isEmpty()) return;
    double ttl = Math.max(0.25, evictSec);

    Iterator<Map.Entry<T, EmaEntry>> it = emaScore.entrySet().iterator();
    while (it.hasNext()) {
      Map.Entry<T, EmaEntry> e = it.next();
      EmaEntry v = e.getValue();
      if (v == null || (now - v.t) > ttl) it.remove();
    }
  }

  double update(double now, T key, double raw, double dt) {
    if (key == null) return raw;

    EmaEntry prevE = emaScore.get(key);
    double prev = prevE != null ? prevE.v : raw;

    double tau = Math.max(1e-3, tauSec);
    double a = 1.0 - Math.exp(-dt / tau);
    double v = prev + (raw - prev) * a;

    emaScore.put(key, new EmaEntry(v, now));
    return v;
  }

  private static final class EmaEntry {
    final double v;
    final double t;

    EmaEntry(double v, double t) {
      this.v = v;
      this.t = t;
    }
  }
}
