/*
 * =============================================================
 *  Repulsor Library
 *  Copyright (c) 2026 Paul Hodges
 *
 *  Licensed under the MIT License.
 *  SPDX-License-Identifier: MIT
 * =============================================================
 */


package org.curtinfrc.frc2026.util.Repulsor.Tuning;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public abstract class TurnTuning extends Tuning {
  protected TurnTuning(String key) {
    super(key);
  }

  public interface CollisionChecker {
    boolean intersects(Translation2d[] rect);
  }

  public static final class TurnResult {
    public final Rotation2d yaw;
    public final double speedScale;

    public TurnResult(Rotation2d yaw, double speedScale) {
      this.yaw = yaw;
      this.speedScale = speedScale;
    }
  }

  public abstract double maxOmegaRadPerSec();

  public abstract double turnMarginMeters();

  public abstract int turnSamples();

  public abstract double scoreTurnMarginMult();

  public abstract double scoreSafetyBubbleMeters();

  public abstract double dockBlendStartMeters();

  public abstract double dockBlendEndMeters();

  public abstract int dockBlendSamples();

  public abstract double scoreTurnSlowdownFactor();

  public abstract TurnResult plan(
      Pose2d pose,
      Pose2d goal,
      Rotation2d pathHeading,
      Translation2d stepVec,
      boolean isScoring,
      double robotX,
      double robotY,
      CollisionChecker checker);

  public static Translation2d[] robotRect(
      Translation2d center, Rotation2d yaw, double rx, double ry) {
    double hx = rx * 0.5, hy = ry * 0.5;
    Translation2d[] local =
        new Translation2d[] {
          new Translation2d(+hx, +hy),
          new Translation2d(+hx, -hy),
          new Translation2d(-hx, -hy),
          new Translation2d(-hx, +hy)
        };
    Translation2d[] world = new Translation2d[4];
    for (int i = 0; i < 4; i++) world[i] = local[i].rotateBy(yaw).plus(center);
    return world;
  }
}
