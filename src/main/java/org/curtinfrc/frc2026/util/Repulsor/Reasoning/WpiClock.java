/*
 * =============================================================
 *  Repulsor Library
 *  Copyright (c) 2026 Paul Hodges
 *
 *  Licensed under the Repulsor Non-Commercial License (RNC-1.0).
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

