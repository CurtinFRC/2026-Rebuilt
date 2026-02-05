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

import java.util.Objects;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicReference;

public abstract class MetricRecorder<T> {
  private final String name;
  private final MetricAggregator<T> aggregator;
  private final AtomicBoolean enabled = new AtomicBoolean(true);
  private final AtomicReference<T> last = new AtomicReference<>(null);
  private final AtomicLong count = new AtomicLong(0);

  protected MetricRecorder(String name, MetricAggregator<T> aggregator) {
    this.name = Objects.requireNonNull(name);
    this.aggregator = Objects.requireNonNull(aggregator);
  }

  public final void record(T data) {
    if (!enabled.get()) return;
    last.set(data);
    aggregator.addSample(data);
    count.incrementAndGet();
    emit(data, aggregator.getOverall(), count.get(), enabled.get());
  }

  protected abstract void emit(T latest, T overall, long count, boolean enabled);

  public void close() {}

  public String getName() {
    return name;
  }

  public void setEnabled(boolean on) {
    enabled.set(on);
    emit(last.get(), aggregator.getOverall(), count.get(), enabled.get());
  }

  public boolean isEnabled() {
    return enabled.get();
  }

  public T getLastRecord() {
    return last.get();
  }

  public T getOverall() {
    return aggregator.getOverall();
  }

  public long getCount() {
    return count.get();
  }

  public void reset() {
    last.set(null);
    aggregator.reset();
    count.set(0);
    emit(null, aggregator.getOverall(), 0, enabled.get());
  }
}
