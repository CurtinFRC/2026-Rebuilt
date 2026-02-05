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

import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

public final class HPStationMetrics {
  private static final Map<String, MetricRecorder<Double>> byKey = new ConcurrentHashMap<>();

  public static MetricRecorder<Double> recorder(String stationKey) {
    return byKey.computeIfAbsent(
        stationKey, k -> new DoubleMeanNTRecorder("hp/" + k + "/pickupTimeSeconds"));
  }

  private HPStationMetrics() {}
}
