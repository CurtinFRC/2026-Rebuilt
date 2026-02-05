/*
 * =============================================================
 *  Repulsor Library
 *  Copyright (c) 2026 Paul Hodges
 *
 *  Licensed under the Repulsor Non-Commercial License (RNC-1.0).
 * =============================================================
 */
package org.curtinfrc.frc2026.util.Repulsor.Profiler;

import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.LongAdder;

final class ProfilerStats {
  final String name;

  final LongAdder wCount = new LongAdder();
  final LongAdder wTotalNs = new LongAdder();
  final AtomicLong wMinNs = new AtomicLong(Long.MAX_VALUE);
  final AtomicLong wMaxNs = new AtomicLong(Long.MIN_VALUE);

  ProfilerStats(String name) {
    this.name = name;
  }

  void record(long durNs) {
    if (durNs <= 0) return;
    wCount.increment();
    wTotalNs.add(durNs);
    updateMin(wMinNs, durNs);
    updateMax(wMaxNs, durNs);
  }

  Snapshot snapshotAndResetWindow() {
    long c = wCount.sum();
    long t = wTotalNs.sum();
    long mn = wMinNs.get();
    long mx = wMaxNs.get();
    if (mn == Long.MAX_VALUE) mn = 0L;
    if (mx == Long.MIN_VALUE) mx = 0L;

    wCount.reset();
    wTotalNs.reset();
    wMinNs.set(Long.MAX_VALUE);
    wMaxNs.set(Long.MIN_VALUE);

    return new Snapshot(name, c, t, mn, mx);
  }

  private static void updateMin(AtomicLong a, long v) {
    long cur = a.get();
    while (v < cur) {
      if (a.compareAndSet(cur, v)) return;
      cur = a.get();
    }
  }

  private static void updateMax(AtomicLong a, long v) {
    long cur = a.get();
    while (v > cur) {
      if (a.compareAndSet(cur, v)) return;
      cur = a.get();
    }
  }

  static final class Snapshot {
    final String name;
    final long count;
    final long totalNs;
    final long minNs;
    final long maxNs;

    Snapshot(String name, long count, long totalNs, long minNs, long maxNs) {
      this.name = name;
      this.count = count;
      this.totalNs = totalNs;
      this.minNs = minNs;
      this.maxNs = maxNs;
    }
  }
}

