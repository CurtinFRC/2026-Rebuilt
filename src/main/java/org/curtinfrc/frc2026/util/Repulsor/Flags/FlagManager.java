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

import java.util.EnumSet;
import java.util.Set;

/**
 * Abstract base class for managing flags of any enum type.
 *
 * @param <E> The enum type of the flags
 */
public abstract class FlagManager<E extends Enum<E>> {

  /**
   * Returns the current active flags. Subclasses must provide how the flags are stored or computed.
   */
  public abstract EnumSet<E> getActiveFlags();

  /** Checks if a given flag is currently active. */
  public boolean hasFlag(E flag) {
    return getActiveFlags().contains(flag);
  }

  /** Enables a specific flag. Subclasses should override if flags are mutable. */
  public void addFlag(E flag) {
    getActiveFlags().add(flag);
  }

  /** Disables a specific flag. */
  public void removeFlag(E flag) {
    getActiveFlags().remove(flag);
  }

  /** Toggles a specific flag. */
  public void toggleFlag(E flag) {
    EnumSet<E> flags = getActiveFlags();
    if (flags.contains(flag)) {
      flags.remove(flag);
    } else {
      flags.add(flag);
    }
  }

  /** Clears all active flags. */
  public void clearFlags() {
    getActiveFlags().clear();
  }

  /** Adds multiple flags at once. */
  public void addAll(Set<E> flags) {
    getActiveFlags().addAll(flags);
  }

  /** Removes multiple flags at once. */
  public void removeAll(Set<E> flags) {
    getActiveFlags().removeAll(flags);
  }

  /** Checks if all of the given flags are active. */
  public boolean hasAll(Set<E> flags) {
    return getActiveFlags().containsAll(flags);
  }

  /** Checks if any of the given flags are active. */
  public boolean hasAny(Set<E> flags) {
    for (E f : flags) {
      if (getActiveFlags().contains(f)) return true;
    }
    return false;
  }

  /** Returns a copy of the current active flags. */
  public EnumSet<E> snapshot() {
    return EnumSet.copyOf(getActiveFlags());
  }

  /** Returns a string listing all active flags. */
  @Override
  public String toString() {
    return "ActiveFlags=" + getActiveFlags();
  }
}
