package org.curtinfrc.frc2026.util.Repulsor.Behaviours;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.function.Supplier;
import org.curtinfrc.frc2026.util.Repulsor.DriveRepulsor;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner;
import org.curtinfrc.frc2026.util.Repulsor.Repulsor;
import org.curtinfrc.frc2026.util.Repulsor.VisionPlanner;

public class BehaviourContext {
  public final Repulsor repulsor;
  public final FieldPlanner planner;
  public final VisionPlanner vision;
  public final DriveRepulsor drive;
  public final double robot_x, robot_y, coral_offset, algae_offset;
  public final Supplier<Pose2d> robotPose;

  public BehaviourContext(
      Repulsor repulsor,
      FieldPlanner planner,
      VisionPlanner vision,
      DriveRepulsor drive,
      double robot_x,
      double robot_y,
      double coral_offset,
      double algae_offset,
      Supplier<Pose2d> robotPose) {
    this.repulsor = repulsor;
    this.planner = planner;
    this.vision = vision;
    this.drive = drive;
    this.robot_x = robot_x;
    this.robot_y = robot_y;
    this.coral_offset = coral_offset;
    this.algae_offset = algae_offset;
    this.robotPose = robotPose;
  }
}
