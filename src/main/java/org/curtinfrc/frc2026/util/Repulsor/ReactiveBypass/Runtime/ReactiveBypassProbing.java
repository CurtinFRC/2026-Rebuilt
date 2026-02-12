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

package org.curtinfrc.frc2026.util.Repulsor.ReactiveBypass.Runtime;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.function.Function;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.FieldPlanner;

final class ReactiveBypassProbing {
  private ReactiveBypassProbing() {}

  static boolean robotTouchDynamic(
      ReactiveBypassConfig cfg,
      Pose2d pose,
      Rotation2d heading,
      double robotX,
      double robotY,
      Function<Translation2d[], Boolean> intersectsDynamicOnly) {
    double rx = robotX + 2.0 * cfg.inflationMeters;
    double ry = robotY + 2.0 * cfg.inflationMeters;
    Translation2d[] rect = FieldPlanner.robotRect(pose.getTranslation(), heading, rx, ry);
    return intersectsDynamicOnly.apply(rect);
  }

  static boolean hysteresisBlocked(
      ReactiveBypassConfig cfg, boolean hasLatchedSubgoal, double occNow) {
    if (hasLatchedSubgoal) return occNow >= cfg.occLow;
    return occNow >= cfg.occHigh;
  }

  static double corridorOcc(
      ReactiveBypassConfig cfg,
      Pose2d pose,
      Rotation2d heading,
      double robotX,
      double robotY,
      Function<Translation2d[], Boolean> intersectsDynamicOnly) {
    double rx = robotX + 2.0 * cfg.inflationMeters;
    double ry = robotY + 2.0 * cfg.inflationMeters;
    Translation2d p = pose.getTranslation();
    Translation2d dir = new Translation2d(1.0, heading);
    Translation2d side = new Translation2d(1.0, heading.rotateBy(Rotation2d.kCCW_90deg));
    int hit = 0, tot = 0;
    for (int i = 1; i <= cfg.corridorSamples; i++) {
      double f = (cfg.triggerAheadMeters * i) / cfg.corridorSamples;
      for (int s = -1; s <= 1; s += 2) {
        double w = s * cfg.triggerWidthMeters * 0.5;
        Translation2d probe = p.plus(dir.times(f)).plus(side.times(w));
        Translation2d[] rect = FieldPlanner.robotRect(probe, heading, rx, ry);
        if (intersectsDynamicOnly.apply(rect)) hit++;
        tot++;
      }
    }
    return (tot == 0) ? 0.0 : ((double) hit / (double) tot);
  }

  static boolean lookAheadClear(
      ReactiveBypassConfig cfg,
      Pose2d pose,
      Rotation2d heading,
      double robotX,
      double robotY,
      Function<Translation2d[], Boolean> intersectsDynamicOnly,
      double aheadMeters,
      double widthMeters) {
    double rx = robotX + 2.0 * cfg.inflationMeters;
    double ry = robotY + 2.0 * cfg.inflationMeters;
    Translation2d p = pose.getTranslation();
    Translation2d dir = new Translation2d(1.0, heading);
    Translation2d side = new Translation2d(1.0, heading.rotateBy(Rotation2d.kCCW_90deg));
    int samples = Math.max(4, cfg.corridorSamples / 2);
    for (int i = 1; i <= samples; i++) {
      double f = (aheadMeters * i) / samples;
      for (int s = -1; s <= 1; s += 2) {
        double w = s * widthMeters * 0.5;
        Translation2d probe = p.plus(dir.times(f)).plus(side.times(w));
        Translation2d[] rect = FieldPlanner.robotRect(probe, heading, rx, ry);
        if (intersectsDynamicOnly.apply(rect)) return false;
      }
    }
    return true;
  }

  static double localOccAt(
      ReactiveBypassConfig cfg,
      Translation2d center,
      Rotation2d heading,
      double robotX,
      double robotY,
      Function<Translation2d[], Boolean> intersectsDynamicOnly) {
    double rx = robotX + 2.0 * cfg.inflationMeters;
    double ry = robotY + 2.0 * cfg.inflationMeters;
    Translation2d[] rect = FieldPlanner.robotRect(center, heading, rx, ry);
    return intersectsDynamicOnly.apply(rect) ? 1.0 : 0.0;
  }

  static int chooseFreerSide(
      ReactiveBypassConfig cfg,
      double sideConfidence,
      int preferredSide,
      Pose2d pose,
      Rotation2d heading,
      double robotX,
      double robotY,
      Function<Translation2d[], Boolean> intersectsDynamicOnly) {
    double ang = Math.toRadians(cfg.sideBiasProbeAngleDeg);
    Rotation2d leftHead = Rotation2d.fromRadians(heading.getRadians() + ang);
    Rotation2d rightHead = Rotation2d.fromRadians(heading.getRadians() - ang);
    double leftOcc = corridorOcc(cfg, pose, leftHead, robotX, robotY, intersectsDynamicOnly);
    double rightOcc = corridorOcc(cfg, pose, rightHead, robotX, robotY, intersectsDynamicOnly);
    double diff = leftOcc - rightOcc;
    if (Math.abs(diff) < 0.04)
      return (sideConfidence > 0.2 && preferredSide != 0) ? preferredSide : +1;
    return (diff < 0) ? +1 : -1;
  }

  static double legOcc(
      ReactiveBypassConfig cfg,
      Translation2d a,
      Translation2d b,
      double robotX,
      double robotY,
      Function<Translation2d[], Boolean> intersectsDynamicOnly) {
    double rx = robotX + 2.0 * cfg.inflationMeters;
    double ry = robotY + 2.0 * cfg.inflationMeters;
    Translation2d delta = b.minus(a);
    double L = Math.max(1e-6, delta.getNorm());
    Rotation2d dir = delta.getAngle();
    Translation2d step = new Translation2d(cfg.probeOccLegStep, dir);
    int steps = Math.max(1, (int) Math.ceil(L / cfg.probeOccLegStep));
    int hit = 0;
    for (int i = 1; i <= steps; i++) {
      Translation2d p = a.plus(step.times(i));
      Translation2d[] rect = FieldPlanner.robotRect(p, dir, rx, ry);
      if (intersectsDynamicOnly.apply(rect)) hit++;
    }
    return (double) hit / (double) steps;
  }
}
