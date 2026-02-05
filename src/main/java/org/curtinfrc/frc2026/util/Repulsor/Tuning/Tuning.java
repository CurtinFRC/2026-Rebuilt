/*
 * =============================================================
 *  Repulsor Library
 *  Copyright (c) 2026 Paul Hodges
 *
 *  Licensed under the Repulsor Non-Commercial License (RNC-1.0).
 * =============================================================
 */
package org.curtinfrc.frc2026.util.Repulsor.Tuning;

public abstract class Tuning {
  private final String key;
  private boolean enabled = true;
  private double dtSeconds = 0.02;

  protected Tuning(String key) {
    this.key = key;
    applyDefaults();
  }

  protected static double clamp01(double x) {
    return Math.max(0.0, Math.min(1.0, x));
  }

  protected static double smooth01(double x) {
    x = clamp01(x);
    return x * x * (3.0 - 2.0 * x);
  }

  protected static double sigmoid(double x) {
    return 1.0 / (1.0 + Math.exp(-x));
  }

  public String key() {
    return key;
  }

  public boolean enabled() {
    return enabled;
  }

  public void setEnabled(boolean enabled) {
    this.enabled = enabled;
  }

  public double dtSeconds() {
    return dtSeconds;
  }

  public void setDtSeconds(double dtSeconds) {
    this.dtSeconds = dtSeconds;
  }

  public abstract void applyDefaults();

  public abstract void reset();
}

