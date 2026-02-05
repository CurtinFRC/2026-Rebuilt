/*
 * =============================================================
 *  Repulsor Library
 *  Copyright (c) 2026 Paul Hodges
 *
 *  Licensed under the Repulsor Non-Commercial License (RNC-1.0).
 * =============================================================
 */
package org.curtinfrc.frc2026.util.Repulsor.DriverStation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;

final class DefaultNtRepulsorDriverStation extends NtRepulsorDriverStation {
  DefaultNtRepulsorDriverStation(NetworkTableInstance inst, String root) {
    super(inst, root);
  }

  @Override
  protected void declareSharedConfig(Schema schema) {
    schema.configDouble("clearance_scale", 1);
    schema.configDouble("repulsion_scale", 1);

    schema.configBool("force_controller_override", false);

    schema.poseOverrideCommand("main", new Pose2d(), false);
    schema.poseResetCommand("main", new Pose2d());

    schema.goalSetpointCommand("main", new Pose2d(), false);
  }
}

