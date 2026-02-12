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

package org.curtinfrc.frc2026.util.Repulsor.Setpoints;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;

public enum HeightSetpoint {
  L1(Meters.of(0.46)),
  L2(Meters.of(0.81)),
  L3(Meters.of(1.21)),
  L4(Meters.of(1.83)),
  PROCESSOR(Meters.of(0.18)),
  DEEP_CAGE(Meters.of(0.75)),
  SHALLOW_CAGE(Meters.of(0.09)),
  CORAL_STATION(Meters.of(0.95)),
  NET(Meters.of(1.93)),
  NONE(null);

  private final Distance height;

  HeightSetpoint(Distance height) {
    this.height = height;
  }

  public Distance getHeight() {
    return height;
  }
}
