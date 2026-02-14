package org.curtinfrc.frc2026.util.Repulsor.State;

public abstract class StaticState implements State {
  @Override
  public abstract void update(double dt);

  @Override
  public State copy() {
    try {
      return (State) super.clone();
    } catch (CloneNotSupportedException e) {
      throw new AssertionError("Clone not supported", e);
    }
  }
}
