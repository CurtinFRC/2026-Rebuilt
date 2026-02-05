/*
 * =============================================================
 *  Repulsor Library
 *  Copyright (c) 2026 Paul Hodges
 *
 *  Licensed under the Repulsor Non-Commercial License (RNC-1.0).
 * =============================================================
 */
package org.curtinfrc.frc2026.util.Repulsor.Reasoning;

import java.util.Optional;

public interface Signals {
  <T> void put(SignalKey<T> key, T value);

  <T> Optional<T> get(SignalKey<T> key);

  <T> T getOr(SignalKey<T> key, T fallback);

  boolean has(SignalKey<?> key);

  void clear();

  default void flush() {}
}

