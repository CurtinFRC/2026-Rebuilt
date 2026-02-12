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

package org.curtinfrc.frc2026.util.Repulsor.Setpoints;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.List;
import java.util.Optional;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacle;

public record SetpointContext(
    Optional<Pose2d> robotPose,
    double robotLengthMeters,
    double robotWidthMeters,
    double coralOffsetMeters,
    double algaeOffsetMeters,
    double shooterReleaseHeightMeters,
    List<? extends Obstacle> dynamicObstacles) {

  public static final SetpointContext EMPTY =
      new SetpointContext(Optional.empty(), 0.0, 0.0, 0.0, 0.0, 0.0, List.of());

  public SetpointContext {
    robotPose = robotPose == null ? Optional.empty() : robotPose;
    dynamicObstacles = dynamicObstacles == null ? List.of() : dynamicObstacles;
  }
}
