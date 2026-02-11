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

package org.curtinfrc.frc2026.util.Repulsor.FieldPlanner;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;

public class RepulsorSample {
  private Translation2d m_goal;
  private LinearVelocity m_vx;
  private LinearVelocity m_vy;
  private Angle m_omega;

  public RepulsorSample(Translation2d goal, double vx, double vy, Angle omega) {
    m_goal = goal;
    m_vx = MetersPerSecond.of(vx);
    m_vy = MetersPerSecond.of(vy);
    m_omega = omega;
  }

  public RepulsorSample(Translation2d goal, ChassisSpeeds v) {
    m_goal = goal;
    m_vx = MetersPerSecond.of(v.vxMetersPerSecond);
    m_vy = MetersPerSecond.of(v.vyMetersPerSecond);
  }

  public RepulsorSample(Translation2d goal, ChassisSpeeds v, Angle omega) {
    this(goal, v);
    m_omega = omega;
  }

  public ChassisSpeeds asChassisSpeeds(PIDController omegaPID, Rotation2d currentRot) {
    double desiredYaw = (m_omega == null) ? currentRot.getRadians() : m_omega.in(Radians);
    double omegaCmd = omegaPID.calculate(currentRot.getRadians(), desiredYaw);
    return ChassisSpeeds.fromFieldRelativeSpeeds(
        m_vx, m_vy, RadiansPerSecond.of(omegaCmd), currentRot);
  }

  public Translation2d goal() {
    return m_goal;
  }
}
