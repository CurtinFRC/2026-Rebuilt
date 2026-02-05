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
