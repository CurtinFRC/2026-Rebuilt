package org.curtinfrc.frc2026.util.Repulsor.Reasoning;

public final class ReasoningKeys {
  private ReasoningKeys() {}

  public static final SignalKey<Boolean> ENABLED = new SignalKey<>("enabled", Boolean.class);
  public static final SignalKey<Boolean> TELEOP = new SignalKey<>("teleop", Boolean.class);
  public static final SignalKey<Boolean> AUTO = new SignalKey<>("auto", Boolean.class);
  public static final SignalKey<Boolean> ENDGAME = new SignalKey<>("endgame", Boolean.class);

  public static SignalKey<Boolean> boolKey(String name) {
    return new SignalKey<>(name, Boolean.class);
  }

  public static SignalKey<Double> doubleKey(String name) {
    return new SignalKey<>(name, Double.class);
  }

  public static SignalKey<Integer> intKey(String name) {
    return new SignalKey<>(name, Integer.class);
  }

  public static SignalKey<String> stringKey(String name) {
    return new SignalKey<>(name, String.class);
  }
}
