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

package org.curtinfrc.frc2026.util.Repulsor.Predictive.Model;

import edu.wpi.first.math.geometry.Translation2d;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.RepulsorSetpoint;

public class Candidate {
  public final RepulsorSetpoint setpoint;
  public final Translation2d targetXY;
  public final double ourEtaS;
  public final double enemyEtaS;
  public final double allyEtaS;
  public final double congestion;
  public final double pressure;
  public final double score;

  public Candidate(
      RepulsorSetpoint sp,
      Translation2d xy,
      double ourEtaS,
      double enemyEtaS,
      double allyEtaS,
      double congestion,
      double pressure,
      double score) {
    this.setpoint = sp;
    this.targetXY = xy;
    this.ourEtaS = ourEtaS;
    this.enemyEtaS = enemyEtaS;
    this.allyEtaS = allyEtaS;
    this.congestion = congestion;
    this.pressure = pressure;
    this.score = score;
  }
}
