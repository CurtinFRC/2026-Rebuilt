/*
 * =============================================================
 *  Repulsor Library
 *  Copyright (c) 2026 Paul Hodges
 *
 *  Licensed under the MIT License.
 *  SPDX-License-Identifier: MIT
 * =============================================================
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
