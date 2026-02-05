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


package org.curtinfrc.frc2026.util.Repulsor.Flags;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.EnumMap;
import java.util.EnumSet;
import java.util.Set;

public class NetworkTableFlagManager<E extends Enum<E>> extends FlagManager<E> {
  private final NetworkTable table;
  private final EnumMap<E, NetworkTableEntry> entries;

  public NetworkTableFlagManager(Class<E> enumClass, String tablePath) {
    this.table = NetworkTableInstance.getDefault().getTable(tablePath);
    this.entries = new EnumMap<>(enumClass);
    for (E f : enumClass.getEnumConstants()) {
      entries.put(f, table.getEntry(f.name()));
      entries.get(f).setDefaultBoolean(false);
    }
  }

  @Override
  public EnumSet<E> getActiveFlags() {
    EnumSet<E> active = EnumSet.noneOf(entries.keySet().iterator().next().getDeclaringClass());
    for (var e : entries.entrySet()) {
      if (e.getValue().getBoolean(false)) active.add(e.getKey());
    }
    return active;
  }

  @Override
  public void addFlag(E flag) {
    setFlag(flag, true);
  }

  @Override
  public void removeFlag(E flag) {
    setFlag(flag, false);
  }

  @Override
  public void toggleFlag(E flag) {
    setFlag(flag, !entries.get(flag).getBoolean(false));
  }

  @Override
  public void clearFlags() {
    for (E f : entries.keySet()) setFlag(f, false);
  }

  @Override
  public void addAll(Set<E> flags) {
    for (E f : flags) setFlag(f, true);
  }

  @Override
  public void removeAll(Set<E> flags) {
    for (E f : flags) setFlag(f, false);
  }

  public void setFlag(E flag, boolean value) {
    entries.get(flag).setBoolean(value);
  }

  public NetworkTable getTable() {
    return table;
  }
}

