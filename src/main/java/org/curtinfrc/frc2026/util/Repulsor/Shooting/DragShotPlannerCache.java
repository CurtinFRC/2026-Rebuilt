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

package org.curtinfrc.frc2026.util.Repulsor.Shooting;

import edu.wpi.first.math.geometry.Translation2d;

final class DragShotPlannerCache {
  private static final int SOLVE_Q_MM = 20;
  private static final int SOLVE_Q_DEG_TENTH = 5;
  private static final int FAST_Q_MM = 250;
  private static final int FAST_Q_DEG = 3;
  private static final int FAST_Q_HEIGHT_MM = 80;
  private static final double FAST_Q_SPEED = 0.8;

  private static final int SOLVE_CACHE_BITS = 14;
  private static final int SOLVE_CACHE_SIZE = 1 << SOLVE_CACHE_BITS;
  private static final long[] SOLVE_CACHE_KEYS = new long[SOLVE_CACHE_SIZE];
  private static final ShotSolution[] SOLVE_CACHE_VALS = new ShotSolution[SOLVE_CACHE_SIZE];
  private static final int SOLVE_CACHE_LOCK_BITS = 6;
  private static final int SOLVE_CACHE_LOCKS = 1 << SOLVE_CACHE_LOCK_BITS;
  private static final Object[] SOLVE_CACHE_GUARDS = new Object[SOLVE_CACHE_LOCKS];

  private static final int FAST_CACHE_BITS = 12;
  private static final int FAST_CACHE_SIZE = 1 << FAST_CACHE_BITS;
  private static final long[] FAST_CACHE_KEYS = new long[FAST_CACHE_SIZE];
  private static final ShotSolution[] FAST_CACHE_VALS = new ShotSolution[FAST_CACHE_SIZE];
  private static final int FAST_CACHE_LOCK_BITS = 5;
  private static final int FAST_CACHE_LOCKS = 1 << FAST_CACHE_LOCK_BITS;
  private static final Object[] FAST_CACHE_GUARDS = new Object[FAST_CACHE_LOCKS];

  static {
    for (int i = 0; i < SOLVE_CACHE_GUARDS.length; i++) SOLVE_CACHE_GUARDS[i] = new Object();
    for (int i = 0; i < FAST_CACHE_GUARDS.length; i++) FAST_CACHE_GUARDS[i] = new Object();
  }

  private DragShotPlannerCache() {}

  static ShotSolution get(long key) {
    int idx = (int) mix64(key) & (SOLVE_CACHE_SIZE - 1);
    if (SOLVE_CACHE_KEYS[idx] == key) {
      return SOLVE_CACHE_VALS[idx];
    }
    return null;
  }

  static void put(long key, ShotSolution val) {
    int idx = (int) mix64(key) & (SOLVE_CACHE_SIZE - 1);
    Object g = SOLVE_CACHE_GUARDS[idx & (SOLVE_CACHE_LOCKS - 1)];
    synchronized (g) {
      SOLVE_CACHE_KEYS[idx] = key;
      SOLVE_CACHE_VALS[idx] = val;
    }
  }

  static ShotSolution fastGet(long key) {
    int idx = (int) mix64(key) & (FAST_CACHE_SIZE - 1);
    if (FAST_CACHE_KEYS[idx] == key) {
      return FAST_CACHE_VALS[idx];
    }
    return null;
  }

  static void fastPut(long key, ShotSolution val) {
    int idx = (int) mix64(key) & (FAST_CACHE_SIZE - 1);
    Object g = FAST_CACHE_GUARDS[idx & (FAST_CACHE_LOCKS - 1)];
    synchronized (g) {
      FAST_CACHE_KEYS[idx] = key;
      FAST_CACHE_VALS[idx] = val;
    }
  }

  static long fastKey(
      Translation2d shooterPos,
      Translation2d target,
      double targetH,
      double releaseH,
      double minSpeed,
      double maxSpeed,
      double minAngDeg,
      double maxAngDeg,
      Constraints.ShotStyle style) {

    int sx = (int) Math.round(shooterPos.getX() * 1000.0 / FAST_Q_MM);
    int sy = (int) Math.round(shooterPos.getY() * 1000.0 / FAST_Q_MM);
    int tx = (int) Math.round(target.getX() * 1000.0 / FAST_Q_MM);
    int ty = (int) Math.round(target.getY() * 1000.0 / FAST_Q_MM);

    int th = (int) Math.round(targetH * 1000.0 / FAST_Q_HEIGHT_MM);
    int rh = (int) Math.round(releaseH * 1000.0 / FAST_Q_HEIGHT_MM);

    int ms = (int) Math.round(minSpeed / FAST_Q_SPEED);
    int xs = (int) Math.round(maxSpeed / FAST_Q_SPEED);

    int a0 = (int) Math.round(minAngDeg / FAST_Q_DEG);
    int a1 = (int) Math.round(maxAngDeg / FAST_Q_DEG);

    int st = style == null ? 0 : style.ordinal();

    long k = 1469598103934665603L;
    k = (k ^ sx) * 1099511628211L;
    k = (k ^ sy) * 1099511628211L;
    k = (k ^ tx) * 1099511628211L;
    k = (k ^ ty) * 1099511628211L;
    k = (k ^ th) * 1099511628211L;
    k = (k ^ rh) * 1099511628211L;
    k = (k ^ ms) * 1099511628211L;
    k = (k ^ xs) * 1099511628211L;
    k = (k ^ a0) * 1099511628211L;
    k = (k ^ a1) * 1099511628211L;
    k = (k ^ st) * 1099511628211L;
    return k;
  }

  static long solveKey(
      Translation2d shooterPos,
      Translation2d target,
      double targetH,
      double releaseH,
      double minSpeed,
      double maxSpeed,
      double minAngDeg,
      double maxAngDeg,
      Constraints.ShotStyle style) {

    int sx = (int) Math.round(shooterPos.getX() * 1000.0 / SOLVE_Q_MM);
    int sy = (int) Math.round(shooterPos.getY() * 1000.0 / SOLVE_Q_MM);
    int tx = (int) Math.round(target.getX() * 1000.0 / SOLVE_Q_MM);
    int ty = (int) Math.round(target.getY() * 1000.0 / SOLVE_Q_MM);

    int th = (int) Math.round(targetH * 1000.0 / 20.0);
    int rh = (int) Math.round(releaseH * 1000.0 / 20.0);

    int ms = (int) Math.round(minSpeed * 10.0);
    int xs = (int) Math.round(maxSpeed * 10.0);

    int a0 = (int) Math.round(minAngDeg * 10.0 / SOLVE_Q_DEG_TENTH);
    int a1 = (int) Math.round(maxAngDeg * 10.0 / SOLVE_Q_DEG_TENTH);

    int st = style == null ? 0 : style.ordinal();

    long k = 1469598103934665603L;
    k = (k ^ sx) * 1099511628211L;
    k = (k ^ sy) * 1099511628211L;
    k = (k ^ tx) * 1099511628211L;
    k = (k ^ ty) * 1099511628211L;
    k = (k ^ th) * 1099511628211L;
    k = (k ^ rh) * 1099511628211L;
    k = (k ^ ms) * 1099511628211L;
    k = (k ^ xs) * 1099511628211L;
    k = (k ^ a0) * 1099511628211L;
    k = (k ^ a1) * 1099511628211L;
    k = (k ^ st) * 1099511628211L;
    return k;
  }

  private static long mix64(long z) {
    z ^= (z >>> 33);
    z *= 0xff51afd7ed558ccdL;
    z ^= (z >>> 33);
    z *= 0xc4ceb9fe1a85ec53L;
    z ^= (z >>> 33);
    return z;
  }
}
