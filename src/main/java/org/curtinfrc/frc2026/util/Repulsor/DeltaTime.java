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

import edu.wpi.first.wpilibj.Timer;

public final class DeltaTime {
  private static final DeltaTime INSTANCE = new DeltaTime();

  public static DeltaTime getInstance() {
    return INSTANCE;
  }

  public static double get() {
    return INSTANCE._get();
  }

  public static void update() {
    INSTANCE._update();
  }

  private double lastTimeS = Double.NaN;
  private volatile double dtS = 0.0;

  private static final double MAX_DT_S = 0.1;

  private DeltaTime() {}

  private void _update() {
    final double nowS = Timer.getFPGATimestamp();

    if (Double.isNaN(lastTimeS)) {
      dtS = 0.0;
    } else {
      double dt = nowS - lastTimeS;

      if (dt < 0.0) dt = 0.0;

      if (dt > MAX_DT_S) dt = MAX_DT_S;

      dtS = dt;
    }

    lastTimeS = nowS;
  }

  private double _get() {
    return dtS;
  }
}
