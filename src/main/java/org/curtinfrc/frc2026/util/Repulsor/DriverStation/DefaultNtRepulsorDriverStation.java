package org.curtinfrc.frc2026.util.Repulsor.DriverStation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;

final class DefaultNtRepulsorDriverStation extends NtRepulsorDriverStation {
  DefaultNtRepulsorDriverStation(NetworkTableInstance inst, String root) {
    super(inst, root);
  }

  @Override
  protected void declareSharedConfig(Schema schema) {
    schema.poseOverrideCommand("main", new Pose2d(), false);
    schema.poseResetCommand("main", new Pose2d());

    schema.goalSetpointCommand("main", new Pose2d(), false);
  }
}
