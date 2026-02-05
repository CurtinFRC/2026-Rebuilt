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

package org.curtinfrc.frc2026.util.Repulsor.Reasoning;

import java.util.HashMap;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;

public final class Blackboard implements Signals {
  private final Map<SignalKey<?>, Object> map = new HashMap<>();

  @Override
  public <T> void put(SignalKey<T> key, T value) {
    Objects.requireNonNull(key);
    if (value == null) {
      map.remove(key);
      return;
    }
    if (!key.type().isInstance(value)) {
      throw new IllegalArgumentException(
          "Value for "
              + key
              + " must be "
              + key.type().getName()
              + ", got "
              + value.getClass().getName());
    }
    map.put(key, value);
  }

  @Override
  public <T> Optional<T> get(SignalKey<T> key) {
    Objects.requireNonNull(key);
    Object v = map.get(key);
    if (v == null) return Optional.empty();
    return Optional.of(key.type().cast(v));
  }

  @Override
  public <T> T getOr(SignalKey<T> key, T fallback) {
    return get(key).orElse(fallback);
  }

  @Override
  public boolean has(SignalKey<?> key) {
    return map.containsKey(key);
  }

  @Override
  public void clear() {
    map.clear();
  }
}
