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

package org.curtinfrc.frc2026.util.Repulsor.Shooting;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.List;
import org.curtinfrc.frc2026.util.Repulsor.Profiler.Profiler;

public final class ShotLibraryBuilder {
  private final GamePiecePhysics gamePiece;
  private final Translation2d targetFieldPosition;
  private final double targetHeightMeters;
  private final double shooterReleaseHeightMeters;
  private final double robotHalfLengthMeters;
  private final double robotHalfWidthMeters;
  private final Constraints constraints;

  private final double minSpeed;
  private final double maxSpeed;
  private final double minAngleDeg;
  private final double maxAngleDeg;
  private final boolean fixedAngle;

  private final double speedStep;
  private final double angleStepDeg;
  private final double radialStep;
  private final double bearingStepDeg;

  private final ArrayList<ShotLibraryEntry> entries = new ArrayList<>(768);

  private double range;
  private double bearingDeg;
  private boolean done;

  private int publishCounter;

  public ShotLibraryBuilder(
      GamePiecePhysics gamePiece,
      Translation2d targetFieldPosition,
      double targetHeightMeters,
      double shooterReleaseHeightMeters,
      double robotHalfLengthMeters,
      double robotHalfWidthMeters,
      Constraints constraints,
      double speedStep,
      double angleStepDeg,
      double radialStep,
      double bearingStepDeg) {
    this.gamePiece = gamePiece;
    this.targetFieldPosition = targetFieldPosition;
    this.targetHeightMeters = targetHeightMeters;
    this.shooterReleaseHeightMeters = shooterReleaseHeightMeters;
    this.robotHalfLengthMeters = robotHalfLengthMeters;
    this.robotHalfWidthMeters = robotHalfWidthMeters;
    this.constraints = constraints;

    this.minSpeed = constraints.minLaunchSpeedMetersPerSecond();
    this.maxSpeed = constraints.maxLaunchSpeedMetersPerSecond();
    this.minAngleDeg = constraints.minLaunchAngleDeg();
    this.maxAngleDeg = constraints.maxLaunchAngleDeg();
    this.fixedAngle = Math.abs(maxAngleDeg - minAngleDeg) < 1e-6;

    this.speedStep = speedStep;
    this.angleStepDeg = angleStepDeg;
    this.radialStep = radialStep;
    this.bearingStepDeg = bearingStepDeg;

    this.range = DragShotPlannerConstants.MIN_RANGE_METERS;
    this.bearingDeg = 0.0;
    this.done = false;
    this.publishCounter = 0;

    Profiler.ensureInit();
    if (Profiler.enabled()) {
      Profiler.gaugeSet(
          "DragShotPlanner.profiler_file_hint_hash", Profiler.outputPathOrEmpty().hashCode());
    }
  }

  public boolean done() {
    return done;
  }

  public int size() {
    return entries.size();
  }

  public ShotLibrary snapshot(boolean completeFlag) {
    AutoCloseable _p = Profiler.section("DragShotPlanner.ShotLibraryBuilder.snapshot");
    try {
      return new ShotLibrary(
          targetFieldPosition,
          targetHeightMeters,
          shooterReleaseHeightMeters,
          robotHalfLengthMeters,
          robotHalfWidthMeters,
          constraints,
          List.copyOf(entries),
          completeFlag);
    } finally {
      DragShotPlannerUtil.closeQuietly(_p);
    }
  }

  public ShotLibrary maybeStep(long budgetNanos) {
    AutoCloseable _p = Profiler.section("DragShotPlanner.ShotLibraryBuilder.maybeStep");
    try {
      if (done) {
        return null;
      }
      long start = System.nanoTime();
      ShotLibrary publish = null;

      while (!done && (System.nanoTime() - start) < budgetNanos) {
        Translation2d shooterPos =
            targetFieldPosition.minus(new Translation2d(range, Rotation2d.fromDegrees(bearingDeg)));

        boolean ok;
        AutoCloseable _p1 = Profiler.section("DragShotPlanner.isShooterPoseValid.library");
        try {
          ok =
              DragShotPlannerObstacles.isShooterPoseValid(
                  shooterPos,
                  targetFieldPosition,
                  robotHalfLengthMeters,
                  robotHalfWidthMeters,
                  null);
        } finally {
          DragShotPlannerUtil.closeQuietly(_p1);
        }

        if (ok) {
          ShotSolution bestAtPos;
          AutoCloseable _p2 =
              Profiler.section("DragShotPlanner.solveBestAtShooterPosition.library");
          try {
            bestAtPos =
                DragShotPlannerSolveAtPosition.solveBestAtShooterPosition(
                    gamePiece,
                    shooterPos,
                    targetFieldPosition,
                    targetHeightMeters,
                    shooterReleaseHeightMeters,
                    minSpeed,
                    maxSpeed,
                    minAngleDeg,
                    maxAngleDeg,
                    fixedAngle,
                    DragShotPlannerConstants.ACCEPTABLE_VERTICAL_ERROR_METERS,
                    constraints.shotStyle(),
                    speedStep,
                    angleStepDeg);
          } finally {
            DragShotPlannerUtil.closeQuietly(_p2);
          }

          if (bestAtPos != null) {
            entries.add(
                new ShotLibraryEntry(
                    bestAtPos.shooterPosition(),
                    bestAtPos.shooterYaw().getRadians(),
                    bestAtPos.launchSpeedMetersPerSecond(),
                    bestAtPos.launchAngle().getRadians(),
                    bestAtPos.timeToPlaneSeconds(),
                    bestAtPos.verticalErrorMeters()));
            Profiler.counterAdd("DragShotPlanner.library.entries_added", 1);
            publishCounter++;
            if (publishCounter >= 8) {
              publishCounter = 0;
              publish = snapshot(false);
            }
          } else {
            Profiler.counterAdd("DragShotPlanner.library.no_solution", 1);
          }
        } else {
          Profiler.counterAdd("DragShotPlanner.library.pose_invalid", 1);
        }

        bearingDeg += bearingStepDeg;
        if (bearingDeg >= 360.0 - 1e-9) {
          bearingDeg = 0.0;
          range += radialStep;
          if (range > DragShotPlannerConstants.MAX_RANGE_METERS + 1e-6) {
            done = true;
            publish = snapshot(true);
            break;
          }
        }
      }

      Profiler.gaugeSet("DragShotPlanner.library.entries_size", entries.size());
      return publish;
    } finally {
      DragShotPlannerUtil.closeQuietly(_p);
    }
  }
}
