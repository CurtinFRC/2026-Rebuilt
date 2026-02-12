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
package org.curtinfrc.frc2026.util.Repulsor.Tracking.Collect.Runtime;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.CollectProbe;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Collect.FieldTrackerCollectObjectiveLoop;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Collect.FieldTrackerCollectObjectiveMath;

public final class FieldTrackerCollectPassDriveStep {
  private FieldTrackerCollectPassDriveStep() {}

  public static Pose2d driveAndFinish(
      FieldTrackerCollectObjectiveLoop loop,
      FieldTrackerCollectPassContext ctx,
      FieldTrackerCollectPassCandidateResult cand,
      FieldTrackerCollectPassStickyResult sticky,
      int pass) {
    Translation2d desiredCollectPoint = sticky.desiredCollectPoint();
    Translation2d desiredDriveTarget = sticky.desiredDriveTarget();

    double dToTarget = ctx.robotPos().getDistance(desiredDriveTarget);

    if (dToTarget <= FieldTrackerCollectObjectiveLoop.COLLECT_STICKY_REACHED_M) {
      if (loop.collectStickyReachedTsNs == 0L) loop.collectStickyReachedTsNs = ctx.nowNs();
    } else {
      loop.collectStickyReachedTsNs = 0L;
    }

    double dToCollect = ctx.robotPos().getDistance(desiredCollectPoint);

    boolean snapNow = loop.collectSnapActive;
    if (!snapNow) {
      if (dToCollect <= FieldTrackerCollectObjectiveLoop.COLLECT_SNAP_TO_POINT_M) snapNow = true;
    } else {
      if (dToCollect
          >= (FieldTrackerCollectObjectiveLoop.COLLECT_SNAP_TO_POINT_M
              + FieldTrackerCollectObjectiveLoop.COLLECT_SNAP_HYST_M)) snapNow = false;
    }
    loop.collectSnapActive = snapNow;

    if (loop.collectSnapActive) {
      desiredDriveTarget = desiredCollectPoint;
      loop.collectStickyDriveTarget = desiredDriveTarget;
    }

    Translation2d forced =
        loop.forceDriveTargetOntoFuel(
            ctx.robotPos(),
            ctx.cap(),
            desiredCollectPoint,
            desiredDriveTarget,
            ctx.clampToFieldRobotSafe(),
            ctx.inForbidden(),
            ctx.violatesWall(),
            ctx.nudgeOutOfForbidden());

    Translation2d chosenForced = null;

    if (forced != null) {
      double dCand =
          (loop.collectForcedDriveCand != null)
              ? loop.collectForcedDriveCand.getDistance(forced)
              : 1e9;

      if (loop.collectForcedDriveCand == null || dCand > 0.16) {
        loop.collectForcedDriveCand = forced;
        loop.collectForcedDriveSinceNs = ctx.nowNs();
      }

      double forcedStableS = (ctx.nowNs() - loop.collectForcedDriveSinceNs) / 1e9;

      double jump =
          (desiredDriveTarget != null)
              ? desiredDriveTarget.getDistance(loop.collectForcedDriveCand)
              : 0.0;

      boolean allowBigJump = loop.collectStickyReachedTsNs != 0L || dToTarget <= 0.90;
      boolean stableEnough = forcedStableS >= 0.32;

      if (stableEnough && (allowBigJump || jump <= 0.35)) {
        chosenForced = loop.collectForcedDriveCand;
      }
    } else {
      loop.collectForcedDriveCand = null;
      loop.collectForcedDriveSinceNs = 0L;
    }

    if (chosenForced != null) {
      desiredDriveTarget = chosenForced;
      loop.collectStickyDriveTarget = chosenForced;
    }

    if (desiredDriveTarget != null) {
      if (loop.collectDriveLastTarget == null
          || loop.collectDriveLastTarget.getDistance(desiredDriveTarget) > 0.12) {
        loop.collectDriveLastTarget = desiredDriveTarget;
        loop.collectDriveLastTargetNs = ctx.nowNs();
        loop.collectStickyNoProgressSinceNs = ctx.nowNs();
        loop.collectStickyLastDistM = ctx.robotPos().getDistance(desiredDriveTarget);
      }
    }

    double distNow = ctx.robotPos().getDistance(desiredDriveTarget);
    if (distNow + 1e-6
        < loop.collectStickyLastDistM
            - FieldTrackerCollectObjectiveLoop.COLLECT_STICKY_NO_PROGRESS_DROP_M) {
      loop.collectStickyLastDistM = distNow;
      loop.collectStickyNoProgressSinceNs = ctx.nowNs();
    } else {
      double sinceS =
          FieldTrackerCollectObjectiveMath.nowSFromNs(
              ctx.nowNs() - loop.collectStickyNoProgressSinceNs);
      if (sinceS >= FieldTrackerCollectObjectiveLoop.COLLECT_STICKY_NO_PROGRESS_S
          && distNow <= FieldTrackerCollectObjectiveLoop.COLLECT_STICKY_NO_PROGRESS_MIN_DIST_M) {

        double sinceTargetChangeS =
            FieldTrackerCollectObjectiveMath.nowSFromNs(
                ctx.nowNs() - loop.collectDriveLastTargetNs);
        if (sinceTargetChangeS < 0.25) {
          loop.collectStickyNoProgressSinceNs = ctx.nowNs();
          loop.collectStickyLastDistM = distNow;
        } else {
          double cooldownS =
              FieldTrackerCollectObjectiveMath.nowSFromNs(
                  ctx.nowNs() - loop.collectStickyLastSwitchNs);
          if (cooldownS >= FieldTrackerCollectObjectiveLoop.COLLECT_STICKY_FLAP_COOLDOWN_S) {
            Translation2d hat =
                loop.collectStickyApproachHat != null
                    ? loop.collectStickyApproachHat
                    : FieldTrackerCollectObjectiveMath.unit(
                        desiredCollectPoint.minus(ctx.robotPos()));
            Translation2d sideHat = FieldTrackerCollectObjectiveMath.perp(hat);

            int nextSide =
                loop.collectStickySide == 0
                    ? (FieldTrackerCollectObjectiveMath.sideSign(desiredCollectPoint) >= 0 ? 1 : -1)
                    : -loop.collectStickySide;
            loop.collectStickySide = nextSide;
            loop.collectStickyLastSwitchNs = ctx.nowNs();
            loop.collectStickyLastSwitchRobotPos = ctx.robotPos();

            Translation2d candStep =
                desiredCollectPoint
                    .plus(hat.times(loop.collectStickyPushM))
                    .plus(sideHat.times((double) nextSide * 0.22));

            candStep = ctx.clampToFieldRobotSafe().apply(candStep);
            if (ctx.inForbidden().test(candStep))
              candStep = ctx.nudgeOutOfForbidden().apply(candStep);
            candStep = ctx.clampToFieldRobotSafe().apply(candStep);

            if (!ctx.inForbidden().test(candStep) && !ctx.violatesWall().test(candStep)) {
              loop.collectStickyDriveTarget = candStep;
              desiredDriveTarget = candStep;
            }

            loop.collectStickyNoProgressSinceNs = ctx.nowNs();
            loop.collectStickyLastDistM = ctx.robotPos().getDistance(desiredDriveTarget);
          }
        }
      }
    }

    desiredDriveTarget = ctx.clampToFieldRobotSafe().apply(desiredDriveTarget);
    if (ctx.inForbidden().test(desiredDriveTarget))
      desiredDriveTarget = ctx.nudgeOutOfForbidden().apply(desiredDriveTarget);
    desiredDriveTarget = ctx.clampToFieldRobotSafe().apply(desiredDriveTarget);

    if (ctx.inForbidden().test(desiredDriveTarget) || ctx.violatesWall().test(desiredDriveTarget)) {
      Translation2d escape = ctx.nudgeOutOfForbidden().apply(desiredDriveTarget);
      escape = ctx.clampToFieldRobotSafe().apply(escape);
      if (!ctx.inForbidden().test(escape) && !ctx.violatesWall().test(escape)) {
        desiredDriveTarget = escape;
        loop.collectStickyDriveTarget = escape;
      } else {
        double escapeX;
        double safePad = 0.60;
        if (ctx.robotPos().getX() < ctx.midX()) {
          escapeX = ctx.leftBandX0() - safePad;
        } else {
          escapeX = ctx.rightBandX1() + safePad;
        }
        Translation2d escape2 =
            ctx.clampToFieldRobotSafe().apply(new Translation2d(escapeX, ctx.robotPos().getY()));
        if (!ctx.inForbidden().test(escape2) && !ctx.violatesWall().test(escape2)) {
          desiredDriveTarget = escape2;
          loop.collectStickyDriveTarget = escape2;
        } else {
          loop.clearCollectSticky();
          return loop.fallbackCollectPose(ctx.robotPoseBlue());
        }
      }
    }

    CollectProbe drivePr =
        loop.predictor.probeCollect(
            desiredDriveTarget, FieldTrackerCollectObjectiveLoop.COLLECT_EMPTY_DRIVE_PROBE_R_M);

    boolean driveLooksEmpty =
        drivePr == null
            || drivePr.count <= 0
            || drivePr.units < FieldTrackerCollectObjectiveLoop.COLLECT_EMPTY_DRIVE_MIN_UNITS;

    int liveFuelNearTarget =
        loop.countLiveCollectResourcesWithin(
            ctx.dynUse(),
            desiredDriveTarget,
            FieldTrackerCollectObjectiveLoop.COLLECT_LIVE_FUEL_NEAR_TARGET_R_M);

    boolean nearTarget =
        ctx.robotPos().getDistance(desiredDriveTarget)
            <= FieldTrackerCollectObjectiveLoop.COLLECT_REACHED_EMPTY_NEAR_TARGET_M;

    boolean reachedOrNear = (loop.collectStickyReachedTsNs != 0L) || nearTarget;

    if (reachedOrNear && liveFuelNearTarget <= 0 && driveLooksEmpty) {
      loop.collectReachedEmptySec += ctx.dt();
    } else {
      loop.collectReachedEmptySec = 0.0;
    }

    if (loop.collectReachedEmptySec
            >= FieldTrackerCollectObjectiveLoop.COLLECT_REACHED_EMPTY_FORCE_DROP_SEC
        && pass == 0) {
      Pose2d hold = ctx.holdPose().apply(desiredDriveTarget);
      if (desiredCollectPoint != null) {
        loop.predictor.markCollectDepleted(
            desiredCollectPoint, FieldTrackerCollectObjectiveLoop.COLLECT_CELL_M, 1.0);
      }
      loop.clearCollectSticky();
      return hold;
    }

    if (ctx.robotPos().getDistance(desiredDriveTarget)
        <= FieldTrackerCollectObjectiveLoop.COLLECT_EMPTY_DRIVE_NEAR_ROBOT_M) {
      if (driveLooksEmpty) loop.collectEmptyDriveSec += ctx.dt();
      else loop.collectEmptyDriveSec = 0.0;
    } else {
      loop.collectEmptyDriveSec = 0.0;
    }

    if (loop.collectEmptyDriveSec >= FieldTrackerCollectObjectiveLoop.COLLECT_EMPTY_DRIVE_DONE_SEC
        && pass == 0) {
      Pose2d hold = ctx.holdPose().apply(desiredDriveTarget);
      loop.clearCollectSticky();
      return hold;
    }

    double stepDist = 0.0;
    double speed = 0.0;

    if (loop.collectStuckAnchorNs == 0L) {
      loop.collectStuckAnchorNs = ctx.nowNs();
      loop.collectStuckAnchorPos = ctx.robotPos();
      loop.collectStuckSec = 0.0;
      loop.lastRobotPosForStuck = ctx.robotPos();
    } else {
      stepDist = ctx.robotPos().getDistance(loop.lastRobotPosForStuck);
      speed = stepDist / Math.max(1e-3, ctx.dt());
      loop.lastRobotPosForStuck = ctx.robotPos();

      double movedFromAnchor = ctx.robotPos().getDistance(loop.collectStuckAnchorPos);

      if (movedFromAnchor >= FieldTrackerCollectObjectiveLoop.COLLECT_STUCK_RESET_MOVE_M) {
        loop.collectStuckAnchorNs = ctx.nowNs();
        loop.collectStuckAnchorPos = ctx.robotPos();
        loop.collectStuckSec = 0.0;
      } else {
        boolean lowSpeed = speed <= FieldTrackerCollectObjectiveLoop.COLLECT_STUCK_SPEED_MPS;
        boolean nearAnchor =
            movedFromAnchor <= FieldTrackerCollectObjectiveLoop.COLLECT_STUCK_RADIUS_M;

        if (lowSpeed && nearAnchor) loop.collectStuckSec += ctx.dt();
        else loop.collectStuckSec = 0.0;
      }
    }

    if (ctx.nearbyFuelCount() < FieldTrackerCollectObjectiveLoop.COLLECT_NEARBY_MIN_COUNT)
      loop.collectNoFuelSec += ctx.dt();
    else loop.collectNoFuelSec = 0.0;

    boolean patchDone =
        loop.collectNoFuelSec >= FieldTrackerCollectObjectiveLoop.COLLECT_DONE_NO_FUEL_SEC;
    boolean stuckTooLong =
        loop.collectStuckSec >= FieldTrackerCollectObjectiveLoop.COLLECT_DONE_STUCK_SEC;

    if ((patchDone || stuckTooLong) && pass == 0) {
      Pose2d hold = ctx.holdPose().apply(desiredDriveTarget);
      if (stuckTooLong && desiredCollectPoint != null) {
        loop.predictor.markCollectDepleted(
            desiredCollectPoint, FieldTrackerCollectObjectiveLoop.COLLECT_CELL_M, 0.65);
      }
      loop.clearCollectSticky();
      return hold;
    }

    return new Pose2d(
        desiredDriveTarget, cand.best() != null ? cand.best().rotation : Rotation2d.kZero);
  }
}
