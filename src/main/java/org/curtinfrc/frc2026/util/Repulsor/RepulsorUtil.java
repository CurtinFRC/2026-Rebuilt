/*
 * =============================================================
 *  Repulsor Library
 *  Copyright (c) 2026 Paul Hodges
 *
 *  Licensed under the MIT License.
 *  SPDX-License-Identifier: MIT
 * =============================================================
 */


package org.curtinfrc.frc2026.util.Repulsor;

public class RepulsorUtil {
  public static double clamp(double value, double min, double max) {
    return Math.max(min, Math.min(max, value));
  }
}
