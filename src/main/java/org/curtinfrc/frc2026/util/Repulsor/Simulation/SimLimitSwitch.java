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

package org.curtinfrc.frc2026.util.Repulsor.Simulation;

/** Simulated limit switch with configurable hysteresis. */
public final class SimLimitSwitch {
  private final double triggerPosition;
  private final boolean lowerLimit;
  private final double hysteresis;
  private boolean triggered;

  private SimLimitSwitch(double triggerPosition, boolean lowerLimit, double hysteresis) {
    this.triggerPosition = triggerPosition;
    this.lowerLimit = lowerLimit;
    this.hysteresis = Math.max(0.0, hysteresis);
  }

  /**
   * Creates a lower limit switch.
   *
   * @param triggerPosition trigger position in output units
   * @param hysteresis hysteresis in output units
   * @return lower limit switch
   */
  public static SimLimitSwitch lowerLimit(double triggerPosition, double hysteresis) {
    return new SimLimitSwitch(triggerPosition, true, hysteresis);
  }

  /**
   * Creates an upper limit switch.
   *
   * @param triggerPosition trigger position in output units
   * @param hysteresis hysteresis in output units
   * @return upper limit switch
   */
  public static SimLimitSwitch upperLimit(double triggerPosition, double hysteresis) {
    return new SimLimitSwitch(triggerPosition, false, hysteresis);
  }

  /**
   * Updates the switch state based on the mechanism position.
   *
   * @param positionUnits position in output units
   */
  public void update(double positionUnits) {
    if (lowerLimit) {
      if (triggered) {
        triggered = positionUnits <= triggerPosition + hysteresis;
      } else {
        triggered = positionUnits <= triggerPosition;
      }
    } else {
      if (triggered) {
        triggered = positionUnits >= triggerPosition - hysteresis;
      } else {
        triggered = positionUnits >= triggerPosition;
      }
    }
  }

  /**
   * Returns whether the switch is triggered.
   *
   * @return true if triggered
   */
  public boolean isTriggered() {
    return triggered;
  }

  /**
   * Sets the triggered state.
   *
   * @param triggered triggered state
   */
  public void reset(boolean triggered) {
    this.triggered = triggered;
  }
}
