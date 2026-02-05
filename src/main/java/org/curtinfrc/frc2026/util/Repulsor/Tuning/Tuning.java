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

