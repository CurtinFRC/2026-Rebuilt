/*
 * =============================================================
 *  Repulsor Library
 *  Copyright (c) 2026 Paul Hodges
 *
 *  Licensed under the Repulsor Non-Commercial License (RNC-1.0).
 * =============================================================
 */
package org.curtinfrc.frc2026.util.Repulsor;

public class RepulsorUtil {
  public static double clamp(double value, double min, double max) {
    return Math.max(min, Math.min(max, value));
  }
}

