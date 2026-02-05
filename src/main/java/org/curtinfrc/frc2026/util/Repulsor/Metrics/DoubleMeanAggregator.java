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

package org.curtinfrc.frc2026.util.Repulsor.Metrics;

public class DoubleMeanAggregator implements MetricAggregator<Double> {
  private double sum = 0.0;
  private long n = 0;

  @Override
  public void addSample(Double value) {
    if (value == null) return;
    sum += value;
    n += 1;
  }

  @Override
  public Double getOverall() {
    return n == 0 ? null : (sum / n);
  }

  @Override
  public void reset() {
    sum = 0.0;
    n = 0;
  }
}
