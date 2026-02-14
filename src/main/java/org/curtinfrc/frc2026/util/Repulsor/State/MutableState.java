package org.curtinfrc.frc2026.util.Repulsor.State;

public abstract class MutableState implements State {
  @Override
  public abstract void update(double dt);

  @Override
  public State copy() {
    return this;
  }
}
