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
import java.util.ArrayList;
import java.util.List;

final class ReactiveBypassWaypointPlanner {
  private ReactiveBypassWaypointPlanner() {}

  static List<Pose2d> generateCandidates(
      ReactiveBypassConfig cfg,
      double lastOcc,
      int preferredSide,
      Pose2d pose,
      Rotation2d heading,
      int biasSide,
      Pose2d goal) {
    double distToGoal = pose.getTranslation().getDistance(goal.getTranslation());
    double nearNorm =
        ReactiveBypassMath.clamp01(distToGoal / Math.max(1e-6, cfg.nearGoalDistMeters));
    double fwdScaleGoal = ReactiveBypassMath.lerp(cfg.nearGoalForwardScale, 1.0, nearNorm);
    double latScaleGoal = ReactiveBypassMath.lerp(cfg.nearGoalLateralScale, 1.0, nearNorm);
    double baseLat = Math.max(cfg.minLateralMeters, cfg.lateralMeters * latScaleGoal);
    double baseFwd = cfg.forwardMeters * fwdScaleGoal;
    double occScale =
        ReactiveBypassMath.clamp01(
            (lastOcc - cfg.occLow) / Math.max(1e-6, (cfg.occHigh - cfg.occLow)));
    double latBoost = 1.0 + 0.6 * occScale;
    double fwdBoost = 1.0 + 0.4 * occScale;
    double robotWall = wallPenalty(cfg, pose.getTranslation());
    boolean nearWall = robotWall > cfg.cornerWallThresh;
    if (nearWall) {
      latBoost *= cfg.cornerEscapeLatBoost;
      fwdBoost *= cfg.cornerEscapeFwdBoost;
    }
    double[] latScales = new double[] {1.0, 1.25, 1.5, latBoost};
    double[] fwdScales = new double[] {1.0, 1.25, fwdBoost};
    ArrayList<Pose2d> out = new ArrayList<>();
    Rotation2d[] headings;
    if (nearWall) {
      Rotation2d wallNormal = nearestWallNormal(cfg, pose.getTranslation());
      double blend = ReactiveBypassMath.clamp01(cfg.cornerEscapeBlend);
      double ax = Math.cos(heading.getRadians());
      double ay = Math.sin(heading.getRadians());
      double bx = Math.cos(wallNormal.getRadians());
      double by = Math.sin(wallNormal.getRadians());
      double cx = blend * ax + (1.0 - blend) * bx;
      double cy = blend * ay + (1.0 - blend) * by;
      Rotation2d escapeHeading = new Rotation2d(cx, cy);
      if (Math.abs(ReactiveBypassMath.radDiff(heading.getRadians(), escapeHeading.getRadians()))
          < Math.toRadians(2.0)) {
        headings = new Rotation2d[] {heading};
      } else {
        headings = new Rotation2d[] {heading, escapeHeading};
      }
    } else {
      headings = new Rotation2d[] {heading};
    }
    for (Rotation2d basis : headings) {
      int[] sides =
          (preferredSide != 0)
              ? new int[] {preferredSide, -preferredSide}
              : new int[] {biasSide, -biasSide};
      for (int sideSign : sides) {
        for (double ls : latScales) {
          double lat =
              ReactiveBypassMath.clamp(baseLat * ls, cfg.minLateralMeters, cfg.lateralMaxMeters);
          for (double fs : fwdScales) {
            double fwd =
                ReactiveBypassMath.clamp(baseFwd * fs, cfg.forwardMeters, cfg.forwardMaxMeters);
            Pose2d wp = makeWaypoint(cfg, pose, basis, sideSign, lat, fwd);
            if (!goesBehindGoal(wp, goal, basis)) out.add(wp);
          }
        }
      }
      double arcDeg = cfg.arcAngleDeg;
      int steps = cfg.arcAngularStepsPerSide;
      int rSteps = Math.max(1, cfg.arcRadialSteps);
      for (int sideSign : sides) {
        for (int ai = 1; ai <= steps; ai++) {
          double a = Math.toRadians((arcDeg * ai) / steps) * sideSign;
          Rotation2d dir = Rotation2d.fromRadians(basis.getRadians() + a);
          for (int ri = 0; ri < rSteps; ri++) {
            double r =
                cfg.arcRMin + (cfg.arcRMax - cfg.arcRMin) * (ri / Math.max(1.0, rSteps - 1.0));
            Translation2d tgt = pose.getTranslation().plus(new Translation2d(r, dir));
            Pose2d wp = new Pose2d(clampToField(cfg, tgt), basis);
            if (!goesBehindGoal(wp, goal, basis)) out.add(wp);
          }
        }
      }
    }
    return out;
  }

  static Pose2d makeWaypoint(
      ReactiveBypassConfig cfg,
      Pose2d pose,
      Rotation2d heading,
      int sideSign,
      double lateral,
      double forward) {
    Translation2d p = pose.getTranslation();
    Translation2d dir = new Translation2d(1.0, heading);
    Translation2d lat =
        new Translation2d(1.0, heading.rotateBy(Rotation2d.kCCW_90deg)).times(sideSign);
    Translation2d tgt = p.plus(dir.times(forward)).plus(lat.times(lateral));
    tgt = clampToField(cfg, tgt);
    return new Pose2d(tgt, heading);
  }

  static Pose2d slewSubgoal(
      ReactiveBypassConfig cfg,
      Pose2d currentLatchedSubgoal,
      Pose2d target,
      Pose2d pose,
      double dt) {
    if (target == null) return null;
    if (currentLatchedSubgoal == null) return target;
    Translation2d cur = currentLatchedSubgoal.getTranslation();
    Translation2d des = target.getTranslation();
    Translation2d delta = des.minus(cur);
    double maxStep = Math.max(0.02, cfg.subgoalSlewRateMps * dt);
    double d = delta.getNorm();
    Translation2d next = (d <= maxStep) ? des : cur.plus(delta.times(maxStep / d));
    if (next.getDistance(des) <= cfg.subgoalJitterMeters) next = des;
    double projCur =
        ReactiveBypassMath.projectAlong(
            pose.getTranslation(), ReactiveBypassMath.headingOf(pose), cur);
    double projNext =
        ReactiveBypassMath.projectAlong(
            pose.getTranslation(), ReactiveBypassMath.headingOf(pose), next);
    if (projNext < projCur && projCur - projNext < 0.12) next = cur;
    return new Pose2d(next, target.getRotation());
  }

  static double wallPenalty(ReactiveBypassConfig cfg, Translation2d p) {
    double dx = Math.min(p.getX(), cfg.fieldLen - p.getX());
    double dy = Math.min(p.getY(), cfg.fieldWid - p.getY());
    double d = Math.min(dx, dy);
    return 1.0 / (0.18 + d);
  }

  static Pose2d alignedPose(Pose2d p, Rotation2d desiredHeading) {
    return new Pose2d(p.getTranslation(), desiredHeading);
  }

  static Translation2d clampToField(ReactiveBypassConfig cfg, Translation2d t) {
    double x = ReactiveBypassMath.clamp(t.getX(), 0.05, cfg.fieldLen - 0.05);
    double y = ReactiveBypassMath.clamp(t.getY(), 0.05, cfg.fieldWid - 0.05);
    return new Translation2d(x, y);
  }

  private static boolean goesBehindGoal(Pose2d wp, Pose2d goal, Rotation2d heading) {
    Translation2d toGoal = goal.getTranslation().minus(wp.getTranslation());
    double cos =
        Math.cos(heading.getRadians()) * toGoal.getX()
            + Math.sin(heading.getRadians()) * toGoal.getY();
    return cos < -0.15;
  }

  private static Rotation2d nearestWallNormal(ReactiveBypassConfig cfg, Translation2d p) {
    double left = p.getX();
    double right = cfg.fieldLen - p.getX();
    double bottom = p.getY();
    double top = cfg.fieldWid - p.getY();
    double min = left;
    int idx = 0;
    if (right < min) {
      min = right;
      idx = 1;
    }
    if (bottom < min) {
      min = bottom;
      idx = 2;
    }
    if (top < min) {
      idx = 3;
    }
    switch (idx) {
      case 0:
        return new Rotation2d();
      case 1:
        return Rotation2d.fromRadians(Math.PI);
      case 2:
        return Rotation2d.kCCW_90deg;
      default:
        return Rotation2d.kCW_90deg;
    }
  }
}
