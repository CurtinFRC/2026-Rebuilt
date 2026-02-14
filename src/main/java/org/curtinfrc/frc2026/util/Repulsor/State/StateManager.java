package org.curtinfrc.frc2026.util.Repulsor.State;

import java.util.Collection;
import java.util.HashMap;
import java.util.Map;

public class StateManager {
  public static final StateManager INSTANCE = new StateManager();

  static class StateRegistry {
    private static final Map<String, State> states = new HashMap<>();

    public static void registerState(State state) {
      states.put(state.getClass().getName(), state);
    }

    public static State getState(String name) {
      return states.get(name);
    }

    public static Collection<State> getAllStates() {
      return states.values();
    }
  }

  static {
    StateRegistry.registerState(new GameState());
  }

  private StateManager() {}

  public static void registerState(State state) {
    StateRegistry.registerState(state);
  }

  public static State getState(String name) {
    return StateRegistry.getState(name);
  }

  public static <T extends State> T getState(Class<T> type) {
    State state = StateRegistry.getState(type.getName());
    if (state == null) {
      return null;
    }
    return type.cast(state);
  }

  public static void update(double dt) {
    for (State state : StateRegistry.getAllStates()) {
      state.update(dt);
    }
  }
}
