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

package org.curtinfrc.frc2026.util.Repulsor;

import org.curtinfrc.frc2026.util.Repulsor.Fields.Rebuilt2026;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldMapBuilder.CategorySpec;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.HeightSetpoint;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.RepulsorSetpoint;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.SetpointType;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.StaticPoseSetpoint;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class DriveRepulsor extends SubsystemBase {
  public abstract void runVelocity(ChassisSpeeds speeds);

  public abstract Pose2d getPose();

  public abstract PIDController getOmegaPID();

  public Command alignTo(Pose2d targetPose) {
    Repulsor re = StaticInstance.getInstance();

    RepulsorSetpoint setpoint = new RepulsorSetpoint(new StaticPoseSetpoint("ALIGN_TARGET_" + targetPose.hashCode(), SetpointType.kOther, targetPose), HeightSetpoint.L1);

    assert re != null : "Repulsor instance is not initialized";
    assert re.isSameDrive(this) : "Repulsor instance does not match this DriveRepulsor";

    return re.alignTo(setpoint, CategorySpec.kEndgame);
  }
}
