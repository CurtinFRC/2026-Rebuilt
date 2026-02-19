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

import choreo.util.ChoreoAllianceFlipUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public abstract class GameSetpoint {
  private final String name;
  private final SetpointType type;
  private final boolean canFlip;

  protected GameSetpoint(String name, SetpointType type) {
    this.name = name;
    this.type = type;
    this.canFlip = true;
  }

  protected GameSetpoint(String name, SetpointType type, boolean canFlip) {
    this.name = name;
    this.type = type;
    this.canFlip = canFlip;
  }

  public final String name() {
    return name;
  }

  public final SetpointType type() {
    return type;
  }

  public abstract Pose2d bluePose(SetpointContext ctx);

  public Pose2d redPose(SetpointContext ctx) {
    return ChoreoAllianceFlipUtil.flip(bluePose(ctx));
    // return canFlip ? SetpointUtil.flipToRed(bluePose(ctx)) : bluePose(ctx);
  }

  public final Pose2d poseForAlliance(Alliance alliance, SetpointContext ctx) {
    return alliance == Alliance.Red ? redPose(ctx) : bluePose(ctx);
  }

  public final Pose2d poseForCurrentAlliance(SetpointContext ctx) {
    return poseForAlliance(SetpointUtil.currentAllianceOrBlue(), ctx);
  }

  public Pose2d approximateBluePose() {
    return bluePose(SetpointContext.EMPTY);
  }

  public Pose2d approximateRedPose() {
    return redPose(SetpointContext.EMPTY);
  }
}
