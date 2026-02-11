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

package org.curtinfrc.frc2026.util.Repulsor.Behaviours;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.function.Supplier;
import org.curtinfrc.frc2026.util.Repulsor.DriveRepulsor;
import org.curtinfrc.frc2026.util.Repulsor.fieldplanner.FieldPlanner;
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

