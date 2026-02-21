package org.curtinfrc.frc2026.sim;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class BallSim {
  private Transform3d lastVelocity = Transform3d.kZero;
  private Pose3d lastPosition = Pose3d.kZero;
  private static final Transform3d acceleration = new Transform3d(0, 0, -9.8, Rotation3d.kZero);

  public BallSim(double launchVelocity, Rotation2d launchAngle, Pose3d launchPosition) {
    lastVelocity =
        new Transform3d(
            launchAngle.getCos() * launchVelocity,
            0,
            launchAngle.getSin() * launchVelocity,
            launchPosition.getRotation());
    lastPosition = launchPosition;
  }

  public Pose3d update(double dt) {
    var velocity = lastVelocity.plus(acceleration.times(dt));
    lastVelocity = velocity;
    var pose = lastPosition.plus(velocity.times(dt));
    lastPosition = pose;
    return pose;
  }
}
