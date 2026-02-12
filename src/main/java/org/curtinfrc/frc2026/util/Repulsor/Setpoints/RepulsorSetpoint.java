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
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public final class RepulsorSetpoint {
  private final GameSetpoint point;
  private final HeightSetpoint height;

  public RepulsorSetpoint(GameSetpoint point, HeightSetpoint height) {
    if (point == null) throw new IllegalArgumentException("point cannot be null");
    this.point = point;
    this.height = height == null ? HeightSetpoint.NONE : height;
  }

  public GameSetpoint point() {
    return point;
  }

  public HeightSetpoint height() {
    return height;
  }

  public Pose2d getBlue(SetpointContext ctx) {
    return point.bluePose(ctx == null ? SetpointContext.EMPTY : ctx);
  }

  public Pose2d getRed(SetpointContext ctx) {
    return point.redPose(ctx == null ? SetpointContext.EMPTY : ctx);
  }

  public Pose2d getForAlliance(Alliance alliance, SetpointContext ctx) {
    return point.poseForAlliance(alliance == null ? Alliance.Blue : alliance, ctx);
  }

  public Pose2d get(SetpointContext ctx) {
    return point.poseForCurrentAlliance(ctx == null ? SetpointContext.EMPTY : ctx);
  }
}
