/*
 * =============================================================
 *  Repulsor Library
 *  Copyright (c) 2026 Paul Hodges
 *
 *  Licensed under the Repulsor Non-Commercial License (RNC-1.0).
 * =============================================================
 */
package org.curtinfrc.frc2026.util.Repulsor.Metrics;

import java.util.Optional;

public interface MetricCodec<T> {
  String encode(T value);

  default Optional<T> decode(String raw) {
    return Optional.empty();
  }
}

