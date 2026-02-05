/*
 * =============================================================
 *  Repulsor Library
 *  Copyright (c) 2026 Paul Hodges
 *
 *  Licensed under the MIT License.
 *  SPDX-License-Identifier: MIT
 * =============================================================
 */


package org.curtinfrc.frc2026.util.Repulsor.Reasoning;

import edu.wpi.first.wpilibj.Timer;

public final class WpiClock implements Clock {
  @Override
  public double nowSec() {
    return Timer.getFPGATimestamp();
  }
}
