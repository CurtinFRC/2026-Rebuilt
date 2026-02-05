/*
 * =============================================================
 *  Repulsor Library
 *  Copyright (c) 2026 Paul Hodges
 *
 *  Licensed under the MIT License.
 *  SPDX-License-Identifier: MIT
 * =============================================================
 */


package org.curtinfrc.frc2026.util.Repulsor.DriverStation;

import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.Objects;

public final class RepulsorDriverStationBootstrap {
  private RepulsorDriverStationBootstrap() {}

  public static void useDefaultNt() {
    useNt(NetworkTableInstance.getDefault(), "/Repulsor/DriverStation");
  }

  public static void useNt(NetworkTableInstance inst, String root) {
    Objects.requireNonNull(inst, "inst");
    if (root == null || root.isEmpty()) throw new IllegalArgumentException("root");
    RepulsorDriverStation.setInstance(new DefaultNtRepulsorDriverStation(inst, root));
  }
}
