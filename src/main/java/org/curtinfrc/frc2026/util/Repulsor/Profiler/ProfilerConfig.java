/*
 * =============================================================
 *  Repulsor Library
 *  Copyright (c) 2026 Paul Hodges
 *
 *  Licensed under the Repulsor Non-Commercial License (RNC-1.0).
 * =============================================================
 */
package org.curtinfrc.frc2026.util.Repulsor.Profiler;

public final class ProfilerConfig {
  public final boolean enabled;
  public final boolean writeFile;
  public final boolean gzipFile;
  public final int summaryPeriodMs;
  public final int queueCapacity;
  public final int topSections;
  public final int topCounters;
  public final int topGauges;

  public ProfilerConfig(
      boolean enabled,
      boolean writeFile,
      boolean gzipFile,
      int summaryPeriodMs,
      int queueCapacity,
      int topSections,
      int topCounters,
      int topGauges) {
    this.enabled = enabled;
    this.writeFile = writeFile;
    this.gzipFile = gzipFile;
    this.summaryPeriodMs = Math.max(250, summaryPeriodMs);
    this.queueCapacity = Math.max(1024, queueCapacity);
    this.topSections = clamp(topSections, 10, 2000);
    this.topCounters = clamp(topCounters, 0, 2000);
    this.topGauges = clamp(topGauges, 0, 2000);
  }

  public static ProfilerConfig fromSystemProperties(boolean defaultEnabled) {
    boolean enabled = boolProp("repulsor.profiler.enabled", defaultEnabled);
    boolean writeFile = boolProp("repulsor.profiler.file", true);
    boolean gzipFile = boolProp("repulsor.profiler.gzip", false);
    int summaryMs = intProp("repulsor.profiler.summaryMs", 1500);
    int queueCap = intProp("repulsor.profiler.queueCapacity", 32768);
    int topSections = intProp("repulsor.profiler.topSections", 300);
    int topCounters = intProp("repulsor.profiler.topCounters", 120);
    int topGauges = intProp("repulsor.profiler.topGauges", 120);
    return new ProfilerConfig(
        enabled, writeFile, gzipFile, summaryMs, queueCap, topSections, topCounters, topGauges);
  }

  private static int clamp(int v, int lo, int hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
  }

  private static boolean boolProp(String k, boolean def) {
    String v = System.getProperty(k);
    if (v == null) return def;
    v = v.trim().toLowerCase();
    if (v.equals("1") || v.equals("true") || v.equals("yes") || v.equals("y") || v.equals("on"))
      return true;
    if (v.equals("0") || v.equals("false") || v.equals("no") || v.equals("n") || v.equals("off"))
      return false;
    return def;
  }

  private static int intProp(String k, int def) {
    String v = System.getProperty(k);
    if (v == null) return def;
    try {
      return Integer.parseInt(v.trim());
    } catch (Exception e) {
      return def;
    }
  }
}

