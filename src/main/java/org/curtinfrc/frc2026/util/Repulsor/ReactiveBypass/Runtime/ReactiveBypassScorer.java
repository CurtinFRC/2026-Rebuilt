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
import java.util.List;
import java.util.function.Function;
import org.curtinfrc.frc2026.util.Repulsor.ExtraPathing;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacle;

final class ReactiveBypassScorer {
  private ReactiveBypassScorer() {}

  static ReactiveBypassScore scoreCandidate(
      ReactiveBypassConfig cfg,
      int preferredSide,
      double timeSinceSideSwitchS,
      boolean stuckNow,
      Pose2d pose,
      Pose2d waypoint,
      Pose2d goal,
      Rotation2d headingTowardGoal,
      double robotX,
      double robotY,
      List<? extends Obstacle> dynamicObstacles,
      Function<Translation2d[], Boolean> intersectsDynamicOnly) {

    boolean ok1 =
        ExtraPathing.isClearPath(
            "Repulsor/Bypass/Leg1",
            pose.getTranslation(),
            waypoint.getTranslation(),
            dynamicObstacles,
            robotX,
            robotY,
            true);

    boolean ok2 =
        ok1
            && ExtraPathing.isClearPath(
                "Repulsor/Bypass/Leg2",
                waypoint.getTranslation(),
                goal.getTranslation(),
                dynamicObstacles,
                robotX,
                robotY,
                true);

    double len =
        pose.getTranslation().getDistance(waypoint.getTranslation())
            + waypoint.getTranslation().getDistance(goal.getTranslation());

    double angErr =
        Math.abs(
            ReactiveBypassMath.radDiff(
                headingTowardGoal.getRadians(),
                waypoint.getTranslation().minus(pose.getTranslation()).getAngle().getRadians()));
    double ang =
        (angErr <= Math.toRadians(cfg.headingDeadbandDeg))
            ? 0.0
            : cfg.angleCostWeight * (angErr - Math.toRadians(cfg.headingDeadbandDeg));

    double leg1 = waypoint.getTranslation().minus(pose.getTranslation()).getAngle().getRadians();
    double leg2 = goal.getTranslation().minus(waypoint.getTranslation()).getAngle().getRadians();
    double curv = cfg.curvatureWeight * Math.abs(ReactiveBypassMath.radDiff(leg1, leg2));

    double wall =
        cfg.wallPenaltyGain
            * ReactiveBypassWaypointPlanner.wallPenalty(cfg, waypoint.getTranslation());

    double occLeg =
        cfg.occCostGain
            * (ReactiveBypassProbing.legOcc(
                    cfg,
                    pose.getTranslation(),
                    waypoint.getTranslation(),
                    robotX,
                    robotY,
                    intersectsDynamicOnly)
                + ReactiveBypassProbing.legOcc(
                    cfg,
                    waypoint.getTranslation(),
                    goal.getTranslation(),
                    robotX,
                    robotY,
                    intersectsDynamicOnly));

    double localOcc =
        cfg.occCostGain
            * 0.5
            * ReactiveBypassProbing.localOccAt(
                cfg,
                waypoint.getTranslation(),
                headingTowardGoal,
                robotX,
                robotY,
                intersectsDynamicOnly);

    int side = ReactiveBypassMath.sideOf(pose, headingTowardGoal, waypoint.getTranslation());
    double sw = sideSwitchPenalty(cfg, preferredSide, timeSinceSideSwitchS, side, stuckNow);

    double yawDeltaDeg =
        Math.abs(Math.toDegrees(ReactiveBypassMath.radDiff(headingTowardGoal.getRadians(), leg1)));
    double zzz = yawDeltaDeg > cfg.zzzMaxYawDeltaDeg ? cfg.zzzPenalty : 0.0;

    Translation2d delta = waypoint.getTranslation().minus(pose.getTranslation());
    double forwardProgress =
        delta.getX() * Math.cos(headingTowardGoal.getRadians())
            + delta.getY() * Math.sin(headingTowardGoal.getRadians());
    double requiredForward =
        cfg.minForwardProgressMeters
            * (pose.getTranslation().getDistance(goal.getTranslation()) > cfg.nearGoalDistMeters
                ? 1.0
                : 0.5);
    double progressDeficit = Math.max(0.0, requiredForward - forwardProgress);
    double prog = cfg.progressCostWeight * progressDeficit;

    double wallHere = ReactiveBypassWaypointPlanner.wallPenalty(cfg, pose.getTranslation());
    double wallWp = ReactiveBypassWaypointPlanner.wallPenalty(cfg, waypoint.getTranslation());
    double cornerReward = 0.0;
    if (wallHere > cfg.cornerWallThresh) {
      double improve = wallHere - wallWp;
      if (improve > 0.0) cornerReward = -cfg.cornerRewardGain * improve;
    }

    double total =
        (ok1 && ok2 ? len : 1e9)
            + ang
            + curv
            + wall
            + occLeg
            + localOcc
            + sw
            + zzz
            + prog
            + cornerReward;

    return new ReactiveBypassScore(
        waypoint,
        ok1,
        ok2,
        len,
        ang,
        curv,
        wall,
        occLeg,
        sw,
        zzz,
        prog,
        localOcc,
        cornerReward,
        total);
  }

  private static double sideSwitchPenalty(
      ReactiveBypassConfig cfg,
      int preferredSide,
      double timeSinceSideSwitchS,
      int candidateSide,
      boolean stuckNow) {
    if (preferredSide == 0) return 0.0;
    if (candidateSide == preferredSide) return 0.0;
    if (stuckNow) {
      return cfg.sideSwitchPenalty * cfg.sideSwitchStuckPenaltyScale;
    }
    if (timeSinceSideSwitchS < cfg.sideStickSeconds) return 1e9;
    return cfg.sideSwitchPenalty;
  }
}
