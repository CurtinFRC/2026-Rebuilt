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

