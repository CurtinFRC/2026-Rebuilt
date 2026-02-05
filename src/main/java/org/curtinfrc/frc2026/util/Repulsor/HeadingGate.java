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

package org.curtinfrc.frc2026.util.Repulsor;

import edu.wpi.first.math.geometry.Rotation2d;

public final class HeadingGate {
  public static final class Config {
    public double deadbandDeg = 6.0;
    public double releaseDeg = 9.0;
    public double snapDeg = 22.0;
    public double maxDegPerSec = 180.0;
  }

  private final Config cfg = new Config();
  private Rotation2d held;
  private boolean latched = false;

  public void configure(java.util.function.Consumer<Config> c) {
    c.accept(cfg);
  }

  public void reset(Rotation2d current) {
    held = current;
    latched = false;
  }

  public Rotation2d filter(Rotation2d currentYaw, Rotation2d desiredYaw, double dtSeconds) {
    if (held == null) held = currentYaw;

    double errDeg = Math.toDegrees(wrap(desiredYaw.getRadians() - held.getRadians()));
    if (!latched) {
      if (Math.abs(errDeg) > cfg.releaseDeg) latched = true;
      else return held;
    } else {
      if (Math.abs(errDeg) < cfg.deadbandDeg) latched = false;
    }

    double maxStepDeg = Math.max(0.0, cfg.maxDegPerSec * dtSeconds);
    double stepDeg = clamp(errDeg, -maxStepDeg, maxStepDeg);

    if (Math.abs(errDeg) >= cfg.snapDeg) held = desiredYaw;
    else held = Rotation2d.fromRadians(held.getRadians() + Math.toRadians(stepDeg));

    return held;
  }

  private static double wrap(double a) {
    while (a > Math.PI) a -= 2 * Math.PI;
    while (a < -Math.PI) a += 2 * Math.PI;
    return a;
  }

  private static double clamp(double x, double lo, double hi) {
    return Math.max(lo, Math.min(hi, x));
  }
}
