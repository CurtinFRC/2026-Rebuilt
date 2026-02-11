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

package org.curtinfrc.frc2026.util.Repulsor.Predictive.Core.Model;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class PointCandidate {
  public final Translation2d point;
  public final double ourEtaS;
  public final double value;
  public final double enemyPressure;
  public final double allyCongestion;
  public final double enemyIntent;
  public final double allyIntent;
  public final double score;
  public final Rotation2d rotation;

  public PointCandidate(
      Translation2d point,
      Rotation2d rot,
      double ourEtaS,
      double value,
      double enemyPressure,
      double allyCongestion,
      double enemyIntent,
      double allyIntent,
      double score) {
    this.point = point;
    this.ourEtaS = ourEtaS;
    this.value = value;
    this.enemyPressure = enemyPressure;
    this.allyCongestion = allyCongestion;
    this.enemyIntent = enemyIntent;
    this.allyIntent = allyIntent;
    this.score = score;
    this.rotation = rot;
  }
}
