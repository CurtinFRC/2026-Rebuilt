package org.curtinfrc.frc2026.util.Repulsor.Metrics;

import org.curtinfrc.frc2026.util.Repulsor.Metrics.MetricCodecs.DoubleCodec;

public class DoubleMeanNTRecorder extends NetworkTablesMetricRecorder<Double> {
  public DoubleMeanNTRecorder(String name) {
    super(name, new DoubleMeanAggregator(), new DoubleCodec());
  }
}
