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

package org.curtinfrc.frc2026.util.Repulsor.Profiler;

import edu.wpi.first.wpilibj.RobotBase;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ThreadFactory;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicLong;

public final class Profiler {
  private static final class Scope implements AutoCloseable {
    private final Profiler p;
    private final String name;
    private final long startNs;

    Scope(Profiler p, String name, long startNs) {
      this.p = p;
      this.name = name;
      this.startNs = startNs;
    }

    @Override
    public void close() {
      long end = System.nanoTime();
      p.record(name, end - startNs);
    }
  }

  private static final AutoCloseable NOOP =
      new AutoCloseable() {
        @Override
        public void close() {}
      };

  private static volatile Profiler INSTANCE;

  private final ProfilerConfig cfg;
  private final ConcurrentHashMap<String, ProfilerStats> stats = new ConcurrentHashMap<>();
  private final ConcurrentHashMap<String, AtomicLong> counters = new ConcurrentHashMap<>();
  private final ConcurrentHashMap<String, AtomicLong> gauges = new ConcurrentHashMap<>();
  private final ProfilerEventWriter writer;
  private final ScheduledExecutorService ses;

  private Profiler(ProfilerConfig cfg) {
    this.cfg = cfg;
    this.writer =
        (cfg.enabled && cfg.writeFile)
            ? new ProfilerEventWriter(cfg.queueCapacity, cfg.gzipFile)
            : null;

    ThreadFactory tf =
        r -> {
          Thread t = new Thread(r, "RepulsorProfilerSummary");
          t.setDaemon(true);
          return t;
        };
    this.ses = Executors.newSingleThreadScheduledExecutor(tf);

    this.ses.scheduleAtFixedRate(
        () -> {
          try {
            dumpSummary("periodic");
          } catch (Throwable ignored) {
          }
        },
        cfg.summaryPeriodMs,
        cfg.summaryPeriodMs,
        TimeUnit.MILLISECONDS);

    Runtime.getRuntime()
        .addShutdownHook(
            new Thread(
                () -> {
                  try {
                    dumpSummary("shutdown");
                  } catch (Throwable ignored) {
                  }
                  try {
                    close();
                  } catch (Throwable ignored) {
                  }
                },
                "RepulsorProfilerShutdown"));
  }

  public static boolean enabled() {
    Profiler p = INSTANCE;
    return p != null && p.cfg.enabled;
  }

  public static void ensureInit() {
    if (!RobotBase.isSimulation()) return;
    if (INSTANCE != null) return;
    synchronized (Profiler.class) {
      if (INSTANCE != null) return;
      ProfilerConfig cfg = ProfilerConfig.fromSystemProperties(true);
      if (!cfg.enabled) {
        INSTANCE = new Profiler(new ProfilerConfig(false, false, false, 1500, 1024, 300, 120, 120));
        return;
      }
      INSTANCE = new Profiler(cfg);
      INSTANCE.dumpSummary("init");
    }
  }

  public static AutoCloseable section(String name) {
    if (!RobotBase.isSimulation()) return NOOP;
    ensureInit();
    Profiler p = INSTANCE;
    if (p == null || !p.cfg.enabled) return NOOP;
    return new Scope(p, name, System.nanoTime());
  }

  public static void counterAdd(String name, long delta) {
    if (!RobotBase.isSimulation()) return;
    ensureInit();
    Profiler p = INSTANCE;
    if (p == null || !p.cfg.enabled) return;
    p.counters.computeIfAbsent(name, k -> new AtomicLong(0L)).addAndGet(delta);
  }

  public static void gaugeSet(String name, long value) {
    if (!RobotBase.isSimulation()) return;
    ensureInit();
    Profiler p = INSTANCE;
    if (p == null || !p.cfg.enabled) return;
    p.gauges.computeIfAbsent(name, k -> new AtomicLong(0L)).set(value);
  }

  public static String outputPathOrEmpty() {
    Profiler p = INSTANCE;
    if (p == null || p.writer == null) return "";
    return p.writer.path().toString();
  }

  public static void dumpNow(String reason) {
    if (!RobotBase.isSimulation()) return;
    ensureInit();
    Profiler p = INSTANCE;
    if (p == null || !p.cfg.enabled) return;
    p.dumpSummary(reason == null ? "manual" : reason);
  }

  public static void flushNow(String reason) {
    dumpNow(reason == null ? "flush" : reason);
  }

  private void record(String name, long durNs) {
    stats.computeIfAbsent(name, ProfilerStats::new).record(durNs);
  }

  private void dumpSummary(String reason) {
    if (writer == null) {
      for (Map.Entry<String, ProfilerStats> e : stats.entrySet()) {
        e.getValue().snapshotAndResetWindow();
      }
      return;
    }

    ArrayList<ProfilerStats.Snapshot> snaps = new ArrayList<>(stats.size());
    for (Map.Entry<String, ProfilerStats> e : stats.entrySet()) {
      ProfilerStats.Snapshot s = e.getValue().snapshotAndResetWindow();
      if (s.count > 0) snaps.add(s);
    }
    snaps.sort(Comparator.comparingLong((ProfilerStats.Snapshot s) -> s.totalNs).reversed());

    ArrayList<Map.Entry<String, AtomicLong>> ctr = new ArrayList<>(counters.entrySet());
    ctr.sort(
        Comparator.comparingLong((Map.Entry<String, AtomicLong> e) -> e.getValue().get())
            .reversed());

    ArrayList<Map.Entry<String, AtomicLong>> ggs = new ArrayList<>(gauges.entrySet());
    ggs.sort(Comparator.comparing((Map.Entry<String, AtomicLong> e) -> e.getKey()));

    StringBuilder b = new StringBuilder(32_768);
    b.append("{\"t\":\"s\",\"ts\":").append(nowMs());
    b.append(",\"r\":\"").append(ProfilerEventWriter.escape(reason)).append("\"");
    b.append(",\"f\":\"").append(ProfilerEventWriter.escape(writer.path().toString())).append("\"");
    b.append(",\"d\":").append(writer.droppedCount());
    b.append(",\"sec\":[");

    int lim = Math.min(cfg.topSections, snaps.size());
    for (int i = 0; i < lim; i++) {
      ProfilerStats.Snapshot s = snaps.get(i);
      if (i != 0) b.append(',');
      b.append("[\"").append(ProfilerEventWriter.escape(s.name)).append("\",");
      b.append(s.count).append(',');
      b.append(s.totalNs).append(',');
      b.append(s.minNs).append(',');
      b.append(s.maxNs).append(']');
    }

    b.append("],\"ctr\":[");
    int clim = Math.min(cfg.topCounters, ctr.size());
    for (int i = 0; i < clim; i++) {
      var e = ctr.get(i);
      if (i != 0) b.append(',');
      b.append("[\"").append(ProfilerEventWriter.escape(e.getKey())).append("\",");
      b.append(e.getValue().get()).append(']');
    }

    b.append("],\"g\":[");
    int glim = Math.min(cfg.topGauges, ggs.size());
    for (int i = 0; i < glim; i++) {
      var e = ggs.get(i);
      if (i != 0) b.append(',');
      b.append("[\"").append(ProfilerEventWriter.escape(e.getKey())).append("\",");
      b.append(e.getValue().get()).append(']');
    }

    b.append("]}");

    writer.writeRaw(b.toString());
    writer.flushNow();
  }

  private void close() {
    try {
      ses.shutdownNow();
    } catch (Throwable ignored) {
    }
    if (writer != null) {
      try {
        writer.close();
      } catch (Throwable ignored) {
      }
    }
  }

  private static long nowMs() {
    return System.currentTimeMillis();
  }

  public static void shutdown() {
    Profiler p = INSTANCE;
    if (p == null) return;
    try {
      p.dumpSummary("shutdown");
    } catch (Throwable ignored) {
    }
    try {
      p.close();
    } catch (Throwable ignored) {
    }
  }
}
