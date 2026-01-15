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
