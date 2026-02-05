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

import java.util.Objects;
import java.util.function.BinaryOperator;

public class ReducerAggregator<T> implements MetricAggregator<T> {
  private final BinaryOperator<T> reducer;
  private T overall;

  public ReducerAggregator(BinaryOperator<T> reducer) {
    this.reducer = Objects.requireNonNull(reducer);
  }

  @Override
  public void addSample(T value) {
    if (value == null) return;
    overall = (overall == null) ? value : reducer.apply(overall, value);
  }

  @Override
  public T getOverall() {
    return overall;
  }

  @Override
  public void reset() {
    overall = null;
  }
}
