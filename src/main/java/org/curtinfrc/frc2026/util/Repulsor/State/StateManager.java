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
