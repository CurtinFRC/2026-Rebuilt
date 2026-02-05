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

