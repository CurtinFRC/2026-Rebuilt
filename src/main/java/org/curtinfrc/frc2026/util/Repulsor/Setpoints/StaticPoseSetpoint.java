package org.curtinfrc.frc2026.util.Repulsor.Setpoints;

import edu.wpi.first.math.geometry.Pose2d;

public class StaticPoseSetpoint extends GameSetpoint {
    private final Pose2d bluePose;

    public StaticPoseSetpoint(String name, SetpointType type, Pose2d bluePose) {
      super(name, type);
      this.bluePose = bluePose == null ? Pose2d.kZero : bluePose;
    }

    @Override
    public Pose2d bluePose(SetpointContext ctx) {
      return bluePose;
    }
  }
