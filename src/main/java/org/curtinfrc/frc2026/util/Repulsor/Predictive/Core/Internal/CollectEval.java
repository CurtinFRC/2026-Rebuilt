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
package org.curtinfrc.frc2026.util.Repulsor.Predictive.Core.Internal;

import edu.wpi.first.math.geometry.Translation2d;

public final class CollectEval {
  public Translation2d p;
  public double eta;
  public double units;
  public int count;
  public double evidence;
  public double value;
  public double enemyPressure;
  public double allyCongestion;
  public double enemyIntent;
  public double allyIntent;
  public double localAvoid;
  public double activity;
  public double depleted;
  public double overlap;
  public int coreCount;
  public double coreDist;
  public double robustPenalty;
  public double score;
  public double regionUnits;
  public double banditBonus;
}
