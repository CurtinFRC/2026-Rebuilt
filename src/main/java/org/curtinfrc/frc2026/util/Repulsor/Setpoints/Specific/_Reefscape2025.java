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

package org.curtinfrc.frc2026.util.Repulsor.Setpoints.Specific;

import static org.curtinfrc.frc2026.util.Repulsor.Constants.aprilTagLayout;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.GameSetpoint;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.SetpointContext;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.SetpointType;

public class _Reefscape2025 {
  protected _Reefscape2025() {}

  public static final GameSetpoint A =
      new ReefTagScoreSetpoint("A", 18, true, 0.0); // i dont remember the offsets, idrc anyways
  public static final GameSetpoint B = new ReefTagScoreSetpoint("B", 18, false, 0.0);
  public static final GameSetpoint C = new ReefTagScoreSetpoint("C", 17, true, 0.0);
  public static final GameSetpoint D = new ReefTagScoreSetpoint("D", 17, false, 0.0);
  public static final GameSetpoint E = new ReefTagScoreSetpoint("E", 22, true, 0.0);
  public static final GameSetpoint F = new ReefTagScoreSetpoint("F", 22, false, 0.0);
  public static final GameSetpoint G = new ReefTagScoreSetpoint("G", 21, true, 0.0);
  public static final GameSetpoint H = new ReefTagScoreSetpoint("H", 21, false, 0.0);
  public static final GameSetpoint I = new ReefTagScoreSetpoint("I", 20, true, 0.0);
  public static final GameSetpoint J = new ReefTagScoreSetpoint("J", 20, false, 0.0);
  public static final GameSetpoint K = new ReefTagScoreSetpoint("K", 19, true, 0.0);
  public static final GameSetpoint L = new ReefTagScoreSetpoint("L", 19, false, 0.0);

  public static final GameSetpoint CLOSE = new ReefTagScoreSetpoint("CLOSE", 18, true, 0.0);
  public static final GameSetpoint CLOSE_LEFT =
      new ReefTagScoreSetpoint("CLOSE_LEFT", 19, true, 0.0);
  public static final GameSetpoint CLOSE_RIGHT =
      new ReefTagScoreSetpoint("CLOSE_RIGHT", 17, true, 0.0);
  public static final GameSetpoint FAR_RIGHT = new ReefTagScoreSetpoint("FAR_RIGHT", 22, true, 0.0);
  public static final GameSetpoint FAR_LEFT = new ReefTagScoreSetpoint("FAR_LEFT", 20, true, 0.0);
  public static final GameSetpoint FAR = new ReefTagScoreSetpoint("FAR", 21, true, 0.0);

  public static final GameSetpoint LEFT_HP =
      new StaticPoseSetpoint(
          "LEFT_HP",
          SetpointType.kHumanPlayer,
          new Pose2d(1.148711085319519, 7.199769020080566, Rotation2d.fromDegrees(125.989 + 180)));

  public static final GameSetpoint RIGHT_HP =
      new StaticPoseSetpoint(
          "RIGHT_HP",
          SetpointType.kHumanPlayer,
          new Pose2d(
              0.9220133423805237,
              0.9964936375617981,
              Rotation2d.fromDegrees(125.989 + 180).unaryMinus()));

  private static final class StaticPoseSetpoint extends GameSetpoint {
    private final Pose2d bluePose;

    StaticPoseSetpoint(String name, SetpointType type, Pose2d bluePose) {
      super(name, type);
      this.bluePose = bluePose == null ? Pose2d.kZero : bluePose;
    }

    @Override
    public Pose2d bluePose(SetpointContext ctx) {
      return bluePose;
    }
  }

  private static final class ReefTagScoreSetpoint extends GameSetpoint {
    private final int tagID;
    private final boolean isLeft;
    private final double offset;

    ReefTagScoreSetpoint(String name, int tagID, boolean isLeft, double offset) {
      super(name, SetpointType.kScore);
      this.tagID = tagID;
      this.isLeft = isLeft;
      this.offset = offset;
    }

    @Override
    public Pose2d bluePose(SetpointContext ctx) {
      var tagPose3dOpt = aprilTagLayout.getTagPose(tagID);
      if (tagPose3dOpt.isEmpty()) return Pose2d.kZero;

      Pose2d tag2d = tagPose3dOpt.get().toPose2d();

      double offset = this.offset;
      Pose2d mappedPose = standoffFromTag(tag2d, ctx.robotLengthMeters(), ctx.robotWidthMeters());

      double yaw = mappedPose.getRotation().getRadians();
      double xOffset = offset * Math.sin(yaw);
      double yOffset = offset * Math.cos(yaw);

      if (isLeft) {
        return new Pose2d(
            mappedPose.getX() + xOffset,
            mappedPose.getY() - yOffset,
            mappedPose.getRotation().plus(Rotation2d.kPi));
      }
      return new Pose2d(
          mappedPose.getX() - xOffset,
          mappedPose.getY() + yOffset,
          mappedPose.getRotation().plus(Rotation2d.kPi));
    }

    @Override
    public Pose2d approximateBluePose() {
      var tagPose3dOpt = aprilTagLayout.getTagPose(tagID);
      if (tagPose3dOpt.isEmpty()) return Pose2d.kZero;
      Pose2d tag2d = tagPose3dOpt.get().toPose2d();
      return new Pose2d(tag2d.getTranslation(), tag2d.getRotation().plus(Rotation2d.kPi));
    }

    private static Pose2d standoffFromTag(Pose2d tagPose, double robotLength, double robotWidth) {
      if (robotLength <= 0.0 && robotWidth <= 0.0) {
        return tagPose;
      }
      double halfL = Math.max(0.0, robotLength) / 2.0;
      double halfW = Math.max(0.0, robotWidth) / 2.0;

      double standoff = Math.hypot(halfL, halfW);
      Translation2d shifted =
          tagPose.getTranslation().plus(new Translation2d(standoff, tagPose.getRotation()));
      return new Pose2d(shifted, tagPose.getRotation());
    }
  }
}
