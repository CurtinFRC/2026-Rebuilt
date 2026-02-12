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

import java.util.function.ToDoubleBiFunction;

final class StickyTargetPingPong<T> {
  private T pingA;
  private T pingB;
  private double pingSinceSec = -1e9;
  private double pingBlockUntilSec = -1e9;

  private double pingPeriodEmaSec = 0.0;

  void clear() {
    pingA = null;
    pingB = null;
    pingSinceSec = -1e9;
    pingBlockUntilSec = -1e9;
    pingPeriodEmaSec = 0.0;
  }

  void canonicalizePing(
      double now, StickyTargetCanon<T> canon, ToDoubleBiFunction<T, T> distanceFn, double eps) {
    if (canon == null) return;
    if (pingA != null) pingA = canon.canonicalize(now, pingA, distanceFn, eps);
    if (pingB != null) pingB = canon.canonicalize(now, pingB, distanceFn, eps);
  }

  boolean isBlocked(double now, T next, ToDoubleBiFunction<T, T> distanceFn, double eps) {
    if (now >= pingBlockUntilSec) return false;
    if (next == null) return false;
    if (pingA != null && StickyTargetMath.eq(next, pingA, distanceFn, eps)) return true;
    if (pingB != null && StickyTargetMath.eq(next, pingB, distanceFn, eps)) return true;
    return false;
  }

  void noteSwitch(
      double now,
      T from,
      T to,
      ToDoubleBiFunction<T, T> distanceFn,
      double eps,
      double switchDistM) {

    if (from == null || to == null) return;

    double d = Double.isFinite(switchDistM) ? Math.max(0.0, switchDistM) : 0.0;

    double windowDist =
        TargetConfig.PINGPONG_WINDOW_BASE_SEC + TargetConfig.PINGPONG_WINDOW_PER_M_SEC * d;
    if (windowDist > TargetConfig.PINGPONG_WINDOW_MAX_SEC)
      windowDist = TargetConfig.PINGPONG_WINDOW_MAX_SEC;
    if (windowDist < TargetConfig.PINGPONG_WINDOW_BASE_SEC)
      windowDist = TargetConfig.PINGPONG_WINDOW_BASE_SEC;

    double blockDist =
        TargetConfig.PINGPONG_BLOCK_BASE_SEC + TargetConfig.PINGPONG_BLOCK_PER_M_SEC * d;
    if (blockDist > TargetConfig.PINGPONG_BLOCK_MAX_SEC)
      blockDist = TargetConfig.PINGPONG_BLOCK_MAX_SEC;
    if (blockDist < TargetConfig.PINGPONG_BLOCK_BASE_SEC)
      blockDist = TargetConfig.PINGPONG_BLOCK_BASE_SEC;

    if (pingA == null || pingB == null) {
      pingA = from;
      pingB = to;
      pingSinceSec = now;
      return;
    }

    boolean sameAB =
        StickyTargetMath.eq(from, pingA, distanceFn, eps)
            && StickyTargetMath.eq(to, pingB, distanceFn, eps);
    boolean sameBA =
        StickyTargetMath.eq(from, pingB, distanceFn, eps)
            && StickyTargetMath.eq(to, pingA, distanceFn, eps);

    if (sameAB || sameBA) {
      double age = now - pingSinceSec;

      if (age > 0.0 && Double.isFinite(age)) {
        if (pingPeriodEmaSec <= 0.0) pingPeriodEmaSec = age;
        else pingPeriodEmaSec += (age - pingPeriodEmaSec) * TargetConfig.PINGPONG_EMA_ALPHA;
      }

      double windowLearned = pingPeriodEmaSec > 0.0 ? (pingPeriodEmaSec * 1.12 + 0.18) : 0.0;
      double blockLearned = pingPeriodEmaSec > 0.0 ? (pingPeriodEmaSec * 0.85 + 0.65) : 0.0;

      if (windowLearned < TargetConfig.PINGPONG_WINDOW_MIN_SEC)
        windowLearned = TargetConfig.PINGPONG_WINDOW_MIN_SEC;
      if (windowLearned > TargetConfig.PINGPONG_WINDOW_MAX_SEC)
        windowLearned = TargetConfig.PINGPONG_WINDOW_MAX_SEC;

      if (blockLearned < TargetConfig.PINGPONG_BLOCK_MIN_SEC)
        blockLearned = TargetConfig.PINGPONG_BLOCK_MIN_SEC;
      if (blockLearned > TargetConfig.PINGPONG_BLOCK_MAX_SEC)
        blockLearned = TargetConfig.PINGPONG_BLOCK_MAX_SEC;

      double window = Math.max(windowDist, windowLearned);
      double block = Math.max(blockDist, blockLearned);

      if (age <= window) {
        pingBlockUntilSec = Math.max(pingBlockUntilSec, now + block);
      }

      pingSinceSec = now;
      return;
    }

    pingA = from;
    pingB = to;
    pingSinceSec = now;
  }
}
