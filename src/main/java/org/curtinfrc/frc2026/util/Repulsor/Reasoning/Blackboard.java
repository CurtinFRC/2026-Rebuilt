// File: src/main/java/org/curtinfrc/frc2026/util/Repulsor/Reasoning/Blackboard.java
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
