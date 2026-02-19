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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public final class SetpointUtil {
  public static Alliance currentAllianceOrBlue() {
    // System.out.println("Current alliance: " + DriverStation.getAlliance());
    return DriverStation.getAlliance().orElse(Alliance.Blue);
  }

  private static Pose2d flipAcrossField(Pose2d p) {
    // if (p == null) return Pose2d.kZero;
    // double x = p.getX();
    // double y = p.getY();
    // double r = p.getRotation().getRadians();
    // double fx = Constants.FIELD_LENGTH - x;
    // double fr = Math.PI - r;
    // return new Pose2d(fx, y, Rotation2d.fromRadians(fr));
    return ChoreoAllianceFlipUtil.flip(p);
  }

  private static Translation2d flipAcrossField(Translation2d t) {
    // if (t == null) return new Translation2d(0.0, 0.0);
    // return new Translation2d(Constants.FIELD_LENGTH - t.getX(), t.getY());
    return ChoreoAllianceFlipUtil.flip(t);
  }

  public static Pose2d flipToRed(Pose2d bluePose) {
    return flipAcrossField(bluePose);
  }

  public static Translation2d flipToRed(Translation2d blue) {
    return flipAcrossField(blue);
  }

  public static Pose2d flipToBlue(Pose2d redPose) {
    return flipAcrossField(redPose);
  }

  public static Translation2d flipToBlue(Translation2d red) {
    return flipAcrossField(red);
  }

  public static Pose2d getSetPose(RepulsorSetpoint sp, SetpointContext ctx) {
    if (sp == null) return Pose2d.kZero;
    return sp.get(ctx);
  }
}
