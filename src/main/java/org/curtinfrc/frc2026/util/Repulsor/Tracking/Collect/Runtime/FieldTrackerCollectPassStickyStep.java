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

import static org.curtinfrc.frc2026.util.Repulsor.Tracking.Collect.FieldTrackerCollectObjectiveMath.dot;
import static org.curtinfrc.frc2026.util.Repulsor.Tracking.Collect.FieldTrackerCollectObjectiveMath.holdSForDist;
import static org.curtinfrc.frc2026.util.Repulsor.Tracking.Collect.FieldTrackerCollectObjectiveMath.nowSFromNs;
import static org.curtinfrc.frc2026.util.Repulsor.Tracking.Collect.FieldTrackerCollectObjectiveMath.sideSign;
import static org.curtinfrc.frc2026.util.Repulsor.Tracking.Collect.FieldTrackerCollectObjectiveMath.sideSignXBand;
import static org.curtinfrc.frc2026.util.Repulsor.Tracking.Collect.FieldTrackerCollectObjectiveMath.stickySame;
import static org.curtinfrc.frc2026.util.Repulsor.Tracking.Collect.FieldTrackerCollectObjectiveMath.stickySwitched;
import static org.curtinfrc.frc2026.util.Repulsor.Tracking.Collect.FieldTrackerCollectObjectiveMath.switchMarginForDist;
import static org.curtinfrc.frc2026.util.Repulsor.Tracking.Collect.FieldTrackerCollectObjectiveMath.unit;
import static org.curtinfrc.frc2026.util.Repulsor.Tracking.Collect.FieldTrackerCollectObjectiveMath.unitOrDefault;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.function.ToDoubleBiFunction;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Collect.FieldTrackerCollectObjectiveLoop;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Collect.FieldTrackerCollectObjectiveMath;

public final class FieldTrackerCollectPassStickyStep {
  private FieldTrackerCollectPassStickyStep() {}

  public static FieldTrackerCollectPassStickyResult selectAndPrime(
      FieldTrackerCollectObjectiveLoop loop,
      FieldTrackerCollectPassContext ctx,
      FieldTrackerCollectPassCandidateResult cand,
      int pass) {
    Translation2d bestCandidate = cand.bestCandidate();
    Translation2d prevSticky = loop.collectStickyPoint;

    double distToCand = ctx.robotPos().getDistance(bestCandidate);
    double distToCur =
        loop.collectStickyPoint != null
            ? ctx.robotPos().getDistance(loop.collectStickyPoint)
            : Double.NaN;
    double holdS = holdSForDist(distToCand);
    double keepMargin = switchMarginForDist(distToCand);
    double immediateDelta = Math.max(keepMargin * 1.85, keepMargin + 0.28);

    if (Double.isFinite(distToCur) && distToCur - distToCand > 0.35) {
      holdS = Math.min(holdS, 0.35);
      keepMargin *= 0.70;
      immediateDelta *= 0.70;
    }

    if (ctx.robotInCenterBand()) {
      holdS = Math.max(holdS, 0.85);
      keepMargin *= 1.12;
      immediateDelta *= 1.08;
    }

    final double keepMarginF = keepMargin;
    final boolean centerF = ctx.robotInCenterBand();
    final int lockHalfF = ctx.lockHalf();

    ToDoubleBiFunction<Translation2d, Translation2d> transitionExtra =
        (from, to) -> {
          if (from == null || to == null) return 0.0;
          double extra = 0.0;

          int p = sideSignXBand(from.getX(), 0.10);
          int q = sideSignXBand(to.getX(), 0.10);

          if (centerF && p != 0 && q != 0 && p != q) extra += keepMarginF * 0.90 + 0.18;
          if (lockHalfF != 0 && q != 0 && q != lockHalfF) extra += keepMarginF * 0.55 + 0.10;
          if (from.getDistance(to) <= FieldTrackerCollectObjectiveLoop.COLLECT_SWITCH_CLOSE_M)
            extra += keepMarginF * 1.55 + 0.18;

          return extra;
        };

    Translation2d selectedResource;
    boolean forcedFuelReplacement = false;

    if (pass == 0) {
      boolean prevHasFuel = prevSticky != null && cand.footprintHasFuel().test(prevSticky);
      boolean bestHasFuel =
          bestCandidate != null
              && cand.footprintHasFuel().test(bestCandidate)
              && cand.collectValid().test(bestCandidate);

      if (prevSticky != null && !prevHasFuel && bestHasFuel) {
        selectedResource = bestCandidate;
        loop.collectStickySelector.force(bestCandidate);
        forcedFuelReplacement = true;
      } else {
        selectedResource =
            loop.collectStickySelector.update(
                bestCandidate,
                cand.scoreResource().apply(bestCandidate),
                cand.scoreResource()::apply,
                cand.collectValid(),
                holdS,
                keepMargin,
                immediateDelta,
                transitionExtra,
                (p, q) -> p.getDistance(q),
                FieldTrackerCollectObjectiveLoop.COLLECT_STICKY_SAME_M,
                loop.collectStickyStillSec,
                loop.collectStickyRobotFlickerSec);
      }
    } else {
      Translation2d cur = loop.collectStickyPoint;
      selectedResource = (cur != null && cand.collectValid().test(cur)) ? cur : bestCandidate;
    }

    if (pass == 0
        && !forcedFuelReplacement
        && prevSticky != null
        && selectedResource != null
        && !stickySame(prevSticky, selectedResource)) {
      Translation2d toPrev = prevSticky.minus(ctx.robotPos());
      Translation2d toNext = selectedResource.minus(ctx.robotPos());

      Translation2d prevHat = unitOrDefault(toPrev, new Translation2d(1.0, 0.0));
      Translation2d nextHat = unitOrDefault(toNext, prevHat);

      boolean opposite = dot(prevHat, nextHat) < 0.15;
      boolean tooSoon = nowSFromNs(ctx.nowNs() - loop.collectStickyLastSwitchNs) < 0.55;

      if ((opposite && !ctx.robotInCenterBand())
          || (tooSoon && loop.collectStickyStillSec < 0.10)) {
        selectedResource = prevSticky;
        loop.collectStickySelector.force(prevSticky);
      }
    }

    boolean switched = stickySwitched(prevSticky, selectedResource);

    if (switched && !forcedFuelReplacement && loop.collectStickyLastSwitchNs != 0L) {
      double sinceLastSwitchS = nowSFromNs(ctx.nowNs() - loop.collectStickyLastSwitchNs);
      double movedSinceLastSwitch =
          loop.collectStickyLastSwitchRobotPos != null
              ? ctx.robotPos().getDistance(loop.collectStickyLastSwitchRobotPos)
              : Double.POSITIVE_INFINITY;
      if (sinceLastSwitchS < FieldTrackerCollectObjectiveLoop.COLLECT_SWITCH_COOLDOWN_S
          && movedSinceLastSwitch < FieldTrackerCollectObjectiveLoop.COLLECT_SWITCH_MIN_MOVE_M) {
        if (prevSticky != null) {
          selectedResource = prevSticky;
          loop.collectStickySelector.force(prevSticky);
          switched = false;
        }
      }
    }

    if (ctx.robotInCenterBand() && loop.collectStickyHalfLock == 0) {
      int s = sideSign(selectedResource);
      if (s != 0) loop.collectStickyHalfLock = s;
    }

    loop.collectStickyPoint = selectedResource;
    loop.collectStickyScore = cand.scoreResource().apply(selectedResource);
    loop.collectStickyTsNs = ctx.nowNs();

    if (switched) {
      loop.collectStickyReachedTsNs = 0L;
      loop.collectStickyLastSwitchNs = ctx.nowNs();
      loop.collectStickyLastSwitchRobotPos = ctx.robotPos();
      loop.collectStickyNoProgressSinceNs = ctx.nowNs();
      loop.collectStickyLastDistM = Double.POSITIVE_INFINITY;
      loop.collectSnapActive = false;

      Translation2d dir = selectedResource.minus(ctx.robotPos());
      Translation2d hat = unitOrDefault(dir, new Translation2d(1.0, 0.0));

      double dist = ctx.robotPos().getDistance(selectedResource);
      double pushT = FieldTrackerCollectObjectiveMath.clamp01((dist - 0.35) / (2.50 - 0.35));
      double pushM =
          Math.min(
              FieldTrackerCollectObjectiveMath.lerp(0.42, 0.18, pushT),
              FieldTrackerCollectObjectiveLoop.COLLECT_MAX_DRIVE_OFFSET_FROM_FUEL_M);

      loop.collectStickyApproachHat = hat;
      loop.collectStickyPushM = pushM;
      loop.collectStickySide = 0;

      Translation2d frozen =
          loop.computeFrozenDriveTarget(
              selectedResource,
              loop.collectStickyApproachHat,
              loop.collectStickyPushM,
              ctx.clampToFieldRobotSafe(),
              ctx.inForbidden(),
              ctx.violatesWall(),
              ctx.nudgeOutOfForbidden());
      loop.collectStickyDriveTarget = frozen;
      loop.collectForcedDriveCand = null;
      loop.collectForcedDriveSinceNs = 0L;
    }

    Translation2d desiredCollectPoint = loop.collectStickyPoint;

    if (desiredCollectPoint == null) {
      loop.clearCollectSticky();
      return new FieldTrackerCollectPassStickyResult(
          null, null, loop.fallbackCollectPose(ctx.robotPoseBlue()));
    }
    Translation2d desiredDriveTarget = loop.collectStickyDriveTarget;

    if (!cand.collectValid().test(desiredCollectPoint)) {
      if (!cand.footprintHasFuel().test(desiredCollectPoint)) {
        loop.predictor.markCollectDepleted(
            desiredCollectPoint, FieldTrackerCollectObjectiveLoop.COLLECT_CELL_M, 1.0);
        loop.clearCollectSticky();
        loop.collectStickySelector.forceInvalidate();
        Translation2d fallbackTarget =
            (bestCandidate != null && cand.collectValid().test(bestCandidate))
                ? bestCandidate
                : null;
        if (fallbackTarget != null) {
          loop.collectStickyPoint = fallbackTarget;
          loop.collectStickyScore = cand.scoreResource().apply(fallbackTarget);
          loop.collectStickyTsNs = ctx.nowNs();
          loop.collectStickyInvalidSec = 0.0;
          loop.collectStickySelector.force(fallbackTarget);
          desiredCollectPoint = fallbackTarget;
        } else {
          return new FieldTrackerCollectPassStickyResult(
              null,
              null,
              ctx.holdPose()
                  .apply(
                      desiredDriveTarget != null
                          ? desiredDriveTarget
                          : ctx.robotPoseBlue().getTranslation()));
        }
      }
      loop.collectStickyInvalidSec += ctx.dt();

      if (loop.collectStickyInvalidSec >= 0.55 && pass == 0) {
        loop.predictor.markCollectDepleted(
            desiredCollectPoint, FieldTrackerCollectObjectiveLoop.COLLECT_CELL_M, 1.0);
        loop.clearCollectSticky();
      }

      return new FieldTrackerCollectPassStickyResult(
          null,
          null,
          ctx.holdPose()
              .apply(
                  desiredDriveTarget != null
                      ? desiredDriveTarget
                      : ctx.robotPoseBlue().getTranslation()));
    } else {
      loop.collectStickyInvalidSec = 0.0;
    }

    if (desiredDriveTarget == null) {
      desiredDriveTarget =
          loop.computeFrozenDriveTarget(
              desiredCollectPoint,
              loop.collectStickyApproachHat != null
                  ? loop.collectStickyApproachHat
                  : unit(desiredCollectPoint.minus(ctx.robotPos())),
              loop.collectStickyPushM > 1e-6 ? loop.collectStickyPushM : 0.18,
              ctx.clampToFieldRobotSafe(),
              ctx.inForbidden(),
              ctx.violatesWall(),
              ctx.nudgeOutOfForbidden());
      loop.collectStickyDriveTarget = desiredDriveTarget;
    }

    desiredDriveTarget = ctx.clampToFieldRobotSafe().apply(desiredDriveTarget);
    if (ctx.inForbidden().test(desiredDriveTarget))
      desiredDriveTarget = ctx.nudgeOutOfForbidden().apply(desiredDriveTarget);
    desiredDriveTarget = ctx.clampToFieldRobotSafe().apply(desiredDriveTarget);

    if (ctx.inForbidden().test(desiredDriveTarget) || ctx.violatesWall().test(desiredDriveTarget)) {
      desiredDriveTarget =
          loop.computeFrozenDriveTarget(
              desiredCollectPoint,
              loop.collectStickyApproachHat != null
                  ? loop.collectStickyApproachHat
                  : unit(desiredCollectPoint.minus(ctx.robotPos())),
              loop.collectStickyPushM > 1e-6 ? loop.collectStickyPushM : 0.18,
              ctx.clampToFieldRobotSafe(),
              ctx.inForbidden(),
              ctx.violatesWall(),
              ctx.nudgeOutOfForbidden());
      loop.collectStickyDriveTarget = desiredDriveTarget;
    }

    return new FieldTrackerCollectPassStickyResult(desiredCollectPoint, desiredDriveTarget, null);
  }
}
