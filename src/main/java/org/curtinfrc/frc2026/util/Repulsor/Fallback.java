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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class Fallback {
  public abstract class PlannerFallback {
    public abstract ChassisSpeeds calculate(Translation2d currentPose, Translation2d target);

    public boolean within(Translation2d err) {
      if (_withinDist() < 0.04) {
        return false;
      }
      return err.getNorm() < _withinDist();
    }

    protected abstract double _withinDist();
  }

  public class PID extends PlannerFallback {
    private PIDController xController;
    private PIDController yController;

    public PID(double kP, double kI, double kD) {
      xController = new PIDController(kP, kI, kD);
      yController = new PIDController(kP, kI, kD);
    }

    @Override
    public ChassisSpeeds calculate(Translation2d currentPose, Translation2d target) {
      return new ChassisSpeeds(
          xController.calculate(currentPose.getX(), target.getX()),
          yController.calculate(currentPose.getY(), target.getY()),
          0);
    }

    @Override
    protected double _withinDist() {
      return 0.02;
    }
  }
}
