// File: src/main/java/org/curtinfrc/frc2026/util/Repulsor/Reasoning/SignalKey.java
package org.curtinfrc.frc2026.util.Repulsor.Reasoning;

import java.util.Objects;

public final class SignalKey<T> {
  private final String name;
  private final Class<T> type;

  public SignalKey(String name, Class<T> type) {
    this.name = Objects.requireNonNull(name);
    this.type = Objects.requireNonNull(type);
  }

  public String name() {
    return name;
  }

  public Class<T> type() {
    return type;
  }

  @Override
  public boolean equals(Object o) {
    if (this == o) return true;
    if (!(o instanceof SignalKey<?> k)) return false;
    return name.equals(k.name) && type.equals(k.type);
  }

  @Override
  public int hashCode() {
    return Objects.hash(name, type);
  }

  @Override
  public String toString() {
    return "SignalKey(" + name + ":" + type.getSimpleName() + ")";
  }
}
