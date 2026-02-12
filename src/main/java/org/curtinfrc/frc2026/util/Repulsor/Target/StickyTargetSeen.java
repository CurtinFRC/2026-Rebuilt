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
import java.util.Map;

final class StickyTargetSeen<T> {
  private final Map<T, Double> lastSeenSec = new HashMap<>(16);
  private double timeoutSec = TargetConfig.DEFAULT_SEEN_TIMEOUT_SEC;

  void setTimeoutSec(double sec) {
    if (!Double.isFinite(sec)) return;
    timeoutSec = Math.max(0.12, sec);
  }

  void clear() {
    lastSeenSec.clear();
  }

  void noteSeen(double now, T value) {
    if (value == null) return;
    lastSeenSec.put(value, now);
  }

  void noteSeen(double now, Iterable<T> values) {
    if (values == null) return;
    for (T v : values) {
      if (v != null) lastSeenSec.put(v, now);
    }
  }

  double age(double now, T t) {
    if (t == null) return 1e9;
    Double s = lastSeenSec.get(t);
    if (s == null) return 1e9;
    return now - s;
  }

  boolean seenRecently(double now, T t) {
    return age(now, t) <= Math.max(0.12, timeoutSec);
  }
}
