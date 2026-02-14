package org.curtinfrc.frc2026.util.Repulsor.State;

public interface State extends Cloneable {
  void update(double dt);

  default void reset() {}

  State copy();

  default void set(State other) {
    throw new UnsupportedOperationException("set(State) not implemented");
  }
}
