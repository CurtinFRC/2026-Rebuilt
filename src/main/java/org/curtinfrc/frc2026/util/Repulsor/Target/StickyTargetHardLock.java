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

final class StickyTargetHardLock {
  private double untilSec = -1e9;

  void clear() {
    untilSec = -1e9;
  }

  boolean isLocked(double now) {
    return now < untilSec;
  }

  void extendToAtLeast(double newUntilSec) {
    if (!Double.isFinite(newUntilSec)) return;
    untilSec = Math.max(untilSec, newUntilSec);
  }

  void arm(double now, double switchDistM) {
    double d = Double.isFinite(switchDistM) ? Math.max(0.0, switchDistM) : 0.0;

    double lock = TargetConfig.BASE_HARD_LOCK_SEC + TargetConfig.HARD_LOCK_PER_M_SEC * d;
    if (lock > TargetConfig.HARD_LOCK_MAX_SEC) lock = TargetConfig.HARD_LOCK_MAX_SEC;
    if (lock < TargetConfig.BASE_HARD_LOCK_SEC) lock = TargetConfig.BASE_HARD_LOCK_SEC;

    untilSec = now + lock;
  }
}
