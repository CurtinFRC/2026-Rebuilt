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

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.function.ToDoubleBiFunction;

final class StickyTargetCanon<T> {
  private final ArrayList<T> canon = new ArrayList<>(TargetConfig.CANON_MAX);
  private final Map<T, Double> canonLastUseSec = new HashMap<>(TargetConfig.CANON_MAX);

  void clear() {
    canon.clear();
    canonLastUseSec.clear();
  }

  void add(double now, T v) {
    canonAdd(now, v);
  }

  T canonicalize(double now, T v, ToDoubleBiFunction<T, T> distanceFn, double eps) {
    if (v == null) return null;

    if (distanceFn == null) {
      if (!canonLastUseSec.containsKey(v)) canonAdd(now, v);
      canonLastUseSec.put(v, now);
      return v;
    }

    canonEvict(now);

    for (int i = 0; i < canon.size(); i++) {
      T r = canon.get(i);
      if (r == null) continue;

      if (StickyTargetMath.eq(v, r, distanceFn, eps)) {
        canonLastUseSec.put(r, now);
        return r;
      }
    }

    canonAdd(now, v);
    return v;
  }

  private void canonEvict(double now) {
    if (canon.isEmpty()) return;
    double ttl = Math.max(0.75, TargetConfig.CANON_EVICT_SEC);

    boolean any = false;
    for (int i = canon.size() - 1; i >= 0; i--) {
      T r = canon.get(i);
      Double t = canonLastUseSec.get(r);
      if (r == null || t == null || (now - t) > ttl) {
        canon.remove(i);
        canonLastUseSec.remove(r);
        any = true;
      }
    }

    if (!any && canon.size() > TargetConfig.CANON_MAX) {
      while (canon.size() > TargetConfig.CANON_MAX) {
        T r = canon.remove(0);
        canonLastUseSec.remove(r);
      }
    }
  }

  private void canonAdd(double now, T v) {
    if (v == null) return;

    if (canon.size() >= TargetConfig.CANON_MAX) {
      canonEvict(now);
      if (canon.size() >= TargetConfig.CANON_MAX) {
        T r = canon.remove(0);
        canonLastUseSec.remove(r);
      }
    }

    canon.add(v);
    canonLastUseSec.put(v, now);
  }
}
