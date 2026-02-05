/*
 * =============================================================
 *  Repulsor Library
 *  Copyright (c) 2026 Paul Hodges
 *
 *  Licensed under the MIT License.
 *  SPDX-License-Identifier: MIT
 * =============================================================
 */


package org.curtinfrc.frc2026.util.Repulsor.Flags;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.EnumMap;
import java.util.EnumSet;
import java.util.Set;
import org.curtinfrc.frc2026.util.Repulsor.Behaviours.BehaviourFlag;

public class BehaviourFlagManager extends FlagManager<BehaviourFlag> {
  private final NetworkTable table;
  private final EnumMap<BehaviourFlag, NetworkTableEntry> entries =
      new EnumMap<>(BehaviourFlag.class);

  public BehaviourFlagManager() {
    this("Repulsor/Behaviours");
  }

  public BehaviourFlagManager(String tablePath) {
    this.table = NetworkTableInstance.getDefault().getTable(tablePath);
    for (BehaviourFlag f : BehaviourFlag.values()) {
      entries.put(f, table.getEntry(f.name()));
      entries.get(f).setDefaultBoolean(false);
    }
  }

  @Override
  public EnumSet<BehaviourFlag> getActiveFlags() {
    EnumSet<BehaviourFlag> active = EnumSet.noneOf(BehaviourFlag.class);
    for (var e : entries.entrySet()) {
      if (e.getValue().getBoolean(false)) {
        active.add(e.getKey());
      }
    }
    return active;
  }

  @Override
  public void addFlag(BehaviourFlag flag) {
    setFlag(flag, true);
  }

  @Override
  public void removeFlag(BehaviourFlag flag) {
    setFlag(flag, false);
  }

  @Override
  public void toggleFlag(BehaviourFlag flag) {
    boolean cur = entries.get(flag).getBoolean(false);
    setFlag(flag, !cur);
  }

  @Override
  public void clearFlags() {
    for (BehaviourFlag f : BehaviourFlag.values()) {
      setFlag(f, false);
    }
  }

  @Override
  public void addAll(Set<BehaviourFlag> flags) {
    for (BehaviourFlag f : flags) setFlag(f, true);
  }

  @Override
  public void removeAll(Set<BehaviourFlag> flags) {
    for (BehaviourFlag f : flags) setFlag(f, false);
  }

  public void setFlag(BehaviourFlag flag, boolean value) {
    entries.get(flag).setBoolean(value);
  }

  public NetworkTable getTable() {
    return table;
  }
}
