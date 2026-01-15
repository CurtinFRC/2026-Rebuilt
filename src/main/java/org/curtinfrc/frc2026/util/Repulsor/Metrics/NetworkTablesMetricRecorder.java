package org.curtinfrc.frc2026.util.Repulsor.Metrics;

import edu.wpi.first.networktables.*;
import java.util.Objects;

public class NetworkTablesMetricRecorder<T> extends MetricRecorder<T> {
  private final MetricCodec<T> codec;
  private final NetworkTable table;
  private final BooleanPublisher enabledPub;
  private final IntegerPublisher countPub;
  private final StringPublisher lastPub;
  private final StringPublisher overallPub;

  public NetworkTablesMetricRecorder(
      String metricName, MetricAggregator<T> aggregator, MetricCodec<T> codec) {
    super(metricName, aggregator);
    this.codec = Objects.requireNonNull(codec);
    NetworkTableInstance nti = NetworkTableInstance.getDefault();
    this.table = nti.getTable("metrics").getSubTable(metricName);
    this.enabledPub = table.getBooleanTopic("enabled").publish();
    this.countPub = table.getIntegerTopic("count").publish();
    this.lastPub = table.getStringTopic("last").publish();
    this.overallPub = table.getStringTopic("overall").publish();
    emit(getLastRecord(), getOverall(), getCount(), isEnabled());
  }

  @Override
  protected void emit(T latest, T overall, long count, boolean enabled) {
    enabledPub.set(enabled);
    countPub.set(count);
    lastPub.set(codec.encode(latest));
    overallPub.set(codec.encode(overall));
  }

  @Override
  public void close() {
    enabledPub.close();
    countPub.close();
    lastPub.close();
    overallPub.close();
  }
}
