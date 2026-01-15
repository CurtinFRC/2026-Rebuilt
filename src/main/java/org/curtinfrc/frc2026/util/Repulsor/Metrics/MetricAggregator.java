package org.curtinfrc.frc2026.util.Repulsor.Metrics;

public interface MetricAggregator<T> {
  void addSample(T value);

  T getOverall();

  void reset();
}
