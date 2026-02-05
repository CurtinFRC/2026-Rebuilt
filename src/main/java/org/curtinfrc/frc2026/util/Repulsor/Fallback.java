/*
 * =============================================================
 *  Repulsor Library
 *  Copyright (c) 2026 Paul Hodges
 *
 *  Licensed under the Repulsor Non-Commercial License (RNC-1.0).
 * =============================================================
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

