package org.curtinfrc.frc2026.util.Repulsor;

public class RepulsorUtil {
  public static double clamp(double value, double min, double max) {
    return Math.max(min, Math.min(max, value));
  }
}
