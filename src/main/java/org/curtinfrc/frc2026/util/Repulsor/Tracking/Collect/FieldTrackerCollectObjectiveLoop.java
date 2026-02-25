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
package org.curtinfrc.frc2026.util.Repulsor.Tracking.Collect;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import java.util.function.Predicate;
import java.util.function.Supplier;

import org.curtinfrc.frc2026.Constants;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldMapBuilder.CategorySpec;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.DynamicObject;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.PointCandidate;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.PredictiveFieldStateRuntime;
import org.curtinfrc.frc2026.util.Repulsor.Target.StickyTarget;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Collect.Runtime.FieldTrackerCollectPassCandidateResult;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Collect.Runtime.FieldTrackerCollectPassCandidateStep;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Collect.Runtime.FieldTrackerCollectPassContext;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Collect.Runtime.FieldTrackerCollectPassDriveStep;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Collect.Runtime.FieldTrackerCollectPassSetup;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Collect.Runtime.FieldTrackerCollectPassSetupResult;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Collect.Runtime.FieldTrackerCollectPassStickyResult;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Collect.Runtime.FieldTrackerCollectPassStickyStep;

public final class FieldTrackerCollectObjectiveLoop {
  public final PredictiveFieldStateRuntime predictor;
  public final Supplier<Translation2d[]> collectObjectivePoints;
  final Supplier<List<DynamicObject>> dynamicsSupplier;
  final Predicate<String> collectTypePredicate;

  FieldTrackerCollectObjectiveLoop(
      PredictiveFieldStateRuntime predictor,
      Supplier<Translation2d[]> collectObjectivePoints,
      Supplier<List<DynamicObject>> dynamicsSupplier,
      Predicate<String> collectTypePredicate) {
    this.predictor = predictor;
    this.collectObjectivePoints = collectObjectivePoints;
    this.dynamicsSupplier = dynamicsSupplier;
    this.collectTypePredicate = collectTypePredicate;
  }

  public List<DynamicObject> snapshotDynamicObjects() {
    List<DynamicObject> dyn = dynamicsSupplier.get();
    return dyn != null ? dyn : List.of();
  }

  public boolean isCollectType(String type) {
    return collectTypePredicate.test(type);
  }

  public StickyTarget<Translation2d> collectStickySelector = new StickyTarget<>(0.22, 1.25, 1.80);

  void resetAll() {
    collectStickySelector = new StickyTarget<>(0.22, 1.25, 1.80);
    clearCollectSticky();
  }

  public volatile long collectStickyReachedTsNs = 0L;

  public volatile Translation2d collectStickyApproachHat = null;
  public volatile double collectStickyPushM = 0.0;
  public volatile long collectStickyNoProgressSinceNs = 0L;
  public volatile double collectStickyLastDistM = Double.POSITIVE_INFINITY;

  public volatile Translation2d collectStickyDriveTarget = null;
  public volatile int collectStickySide = 0;
  public volatile long collectStickyLastSwitchNs = 0L;
  public volatile Translation2d collectDriveLastTarget = null;
  public volatile long collectDriveLastTargetNs = 0L;
  static final double COLLECT_GROUP_CELL_M = 0.40;
  static final double COLLECT_GROUP_R1_M = 0.95;
  static final double COLLECT_GROUP_R2_M = 1.70;
  static final double COLLECT_GROUP_MIN_COUNT = 2.0;
  static final double COLLECT_RELOCK_ENABLE_DIST_M = 1.6;
  public static final double COLLECT_HALF_KEEP_MID_BAND_M = 1.2;
  public volatile double collectStickyInvalidSec = 0.0;

  public volatile Translation2d collectForcedDriveCand = null;
  public volatile long collectForcedDriveSinceNs = 0L;

  public static final double COLLECT_STUCK_RADIUS_M = 0.10;
  public static final double COLLECT_STUCK_RESET_MOVE_M = 0.22;

  public Translation2d collectStuckAnchorPos = new Translation2d();
  public volatile long collectStuckAnchorNs = 0L;

  public volatile int collectStickyHalfLock = 0;

  public PointCandidate lastBest;

  static final double COLLECT_GROUP_W_C1 = 1.00;
  static final double COLLECT_GROUP_W_C2 = 0.35;
  static final double COLLECT_GROUP_W_ETA = 1.25;
  static final double COLLECT_GROUP_W_SPREAD = 0.60;
  static final double COLLECT_GROUP_W_CENTER = 0.18;
  public static final double COLLECT_NEARBY_RADIUS_M = 2.2;
  public static final int COLLECT_NEARBY_MIN_COUNT = 1;

  public static final double COLLECT_LIVE_FUEL_NEAR_TARGET_R_M = 0.65;
  public static final double COLLECT_REACHED_EMPTY_FORCE_DROP_SEC = 0.18;
  public static final double COLLECT_REACHED_EMPTY_NEAR_TARGET_M = 0.95;
  public double collectReachedEmptySec = 0.0;

  public static final double COLLECT_DONE_NO_FUEL_SEC = 0.35;
  public static final double COLLECT_DONE_STUCK_SEC = 0.35;
  public static final double COLLECT_STUCK_SPEED_MPS = 0.15;

  public static final double COLLECT_EMPTY_DRIVE_DONE_SEC = 0.35;
  public static final double COLLECT_EMPTY_DRIVE_NEAR_ROBOT_M = 1.05;
  public static final double COLLECT_EMPTY_DRIVE_PROBE_R_M = 0.55;
  public static final double COLLECT_EMPTY_DRIVE_MIN_UNITS = 0.06;

  public double collectEmptyDriveSec = 0.0;

  public static final double COLLECT_CELL_M = 0.14;
  public static final int COLLECT_COARSE_TOPK = 4;
  public static final int COLLECT_REFINE_GRID = 3;

  public static final double COLLECT_DRIVE_PROBE_R_M = 0.55;
  public static final double COLLECT_DRIVE_MIN_UNITS = 0.045;
  public static final double COLLECT_DRIVE_SEARCH_STEP_M = 0.14;
  public static final int COLLECT_DRIVE_SEARCH_GRID = 3;

  public Translation2d collectStickyStillFiltPos = new Translation2d();
  public Translation2d collectStickyStillFiltLastPos = new Translation2d();
  public int collectStickyRobotHalfLast = 0;
  public double collectStickyRobotFlickerSec = 0.0;

  public static final double COLLECT_STICKY_SAME_M = 0.45;
  public static final double COLLECT_SWITCH_CLOSE_M = 0.85;

  public static final double COLLECT_STICKY_REACHED_M = 0.22;
  public static final double COLLECT_STICKY_TARGET_RECALC_EPS_M = 0.14;

  public static final double COLLECT_STICKY_NO_PROGRESS_S = 0.40;
  public static final double COLLECT_STICKY_NO_PROGRESS_DROP_M = 0.06;
  public static final double COLLECT_STICKY_NO_PROGRESS_MIN_DIST_M = 0.50;

  public static final double COLLECT_STICKY_FLAP_COOLDOWN_S = 0.95;
  public static final double COLLECT_SWITCH_MIN_MOVE_M = 0.18;
  public static final double COLLECT_SWITCH_COOLDOWN_S = 0.70;

  public double collectNoFuelSec = 0.0;
  public double collectStuckSec = 0.0;

  public Translation2d lastRobotPosForStuck = new Translation2d();

  volatile double collectStickyEtaS = 0.0;
  public volatile long lastObjectiveTickNs = 0L;

  static final double COLLECT_RESOURCE_SNAP_MAX_DIST_M = 0.55;
  static final double COLLECT_RESOURCE_SNAP_TINY_M = 0.14;
  public static final double COLLECT_VALID_NEAR_FUEL_M = 0.25;
  public static final double COLLECT_RESOURCE_SNAP_MIN_UNITS = 0.07;

  public static final double COLLECT_SNAP_TO_POINT_M = 0.14;
  public static final double COLLECT_SNAP_HYST_M = 0.06;
  public boolean collectSnapActive = false;
  public Translation2d collectStickyStillLastPos = new Translation2d();
  public double collectStickyStillSec = 0.0;

  public Translation2d collectStickyLastSwitchRobotPos = null;

  public volatile Translation2d collectStickyPoint = null;
  public volatile double collectStickyScore = -1e18;
  public volatile long collectStickyTsNs = 0L;

  static final double COLLECT_EMPTY_SPACE_MAX_DIST_TO_FUEL_M = 0.18;
  public static final double COLLECT_HOTSPOT_SNAP_RADIUS_M = 0.85;
  public static final double COLLECT_MAX_DRIVE_OFFSET_FROM_FUEL_M = 0.12;
  public static final double COLLECT_SNAP_TO_NEAREST_FUEL_M = 0.22;

  public void clearCollectSticky() {
    collectStickyPoint = null;
    collectStickyScore = -1e18;
    collectStickyTsNs = 0L;
    collectStickyReachedTsNs = 0L;
    collectStickyApproachHat = null;
    collectStickyPushM = 0.0;
    collectStickyDriveTarget = null;
    collectStickyNoProgressSinceNs = 0L;
    collectStickyLastDistM = Double.POSITIVE_INFINITY;
    collectStickyLastSwitchNs = 0L;
    collectStickyLastSwitchRobotPos = null;
    collectStickyEtaS = 0.0;
    collectStickySide = 0;
    collectSnapActive = false;
    collectStickyHalfLock = 0;
    collectNoFuelSec = 0.0;
    collectStuckSec = 0.0;
    lastRobotPosForStuck = new Translation2d();
    collectEmptyDriveSec = 0.0;
    collectStuckAnchorPos = new Translation2d();
    collectStuckAnchorNs = 0L;
    collectReachedEmptySec = 0.0;
    collectStickySelector.clear();
    collectStickyStillLastPos = new Translation2d();
    collectStickyStillSec = 0.0;
    collectStickyStillFiltPos = new Translation2d();
    collectStickyStillFiltLastPos = new Translation2d();
    collectStickyRobotHalfLast = 0;
    collectStickyRobotFlickerSec = 0.0;
    collectStickyInvalidSec = 0.0;
    collectDriveLastTarget = null;
    collectDriveLastTargetNs = 0L;
  }

  public int countLiveCollectResourcesWithin(
      List<DynamicObject> dyn, Translation2d center, double r) {
    if (dyn == null || dyn.isEmpty() || center == null) return 0;
    double r2 = r * r;
    int n = 0;
    for (int i = 0; i < dyn.size(); i++) {
      DynamicObject o = dyn.get(i);
      if (o == null || o.pos == null || o.type == null) continue;
      if (!isCollectType(o.type)) continue;
      double dx = o.pos.getX() - center.getX();
      double dy = o.pos.getY() - center.getY();
      if ((dx * dx + dy * dy) <= r2) n++;
    }
    return n;
  }

  public Translation2d relockCollectPointToLiveFuel(
      Translation2d desiredCollectPoint,
      java.util.function.Function<Translation2d, Translation2d> clampToFieldRobotSafe,
      java.util.function.Predicate<Translation2d> inForbidden,
      java.util.function.Predicate<Translation2d> violatesWall,
      java.util.function.Function<Translation2d, Translation2d> nudgeOutOfForbidden) {

    if (desiredCollectPoint == null) return null;

    Translation2d p = desiredCollectPoint;

    Translation2d nearTiny = predictor.nearestCollectResource(p, COLLECT_RESOURCE_SNAP_TINY_M);
    if (nearTiny != null) {
      p = nearTiny;
    } else {
      Translation2d near = predictor.nearestCollectResource(p, COLLECT_RESOURCE_SNAP_MAX_DIST_M);
      if (near != null) {
        p = near;
      } else {
        Translation2d centroid = predictor.snapToCollectCentroid(p, 0.75, 0.15);
        if (centroid != null) p = centroid;
      }
    }

    p = clampToFieldRobotSafe.apply(p);
    if (inForbidden.test(p)) p = nudgeOutOfForbidden.apply(p);
    p = clampToFieldRobotSafe.apply(p);

    if (inForbidden.test(p) || violatesWall.test(p)) return desiredCollectPoint;
    return p;
  }

  public Translation2d computeFrozenDriveTarget(
      Translation2d resource,
      Translation2d approachHat,
      double pushM,
      java.util.function.Function<Translation2d, Translation2d> clampToFieldRobotSafe,
      java.util.function.Predicate<Translation2d> inForbidden,
      java.util.function.Predicate<Translation2d> violatesWall,
      java.util.function.Function<Translation2d, Translation2d> nudgeOutOfForbidden) {

    Translation2d t = clampToFieldRobotSafe.apply(resource.plus(approachHat.times(pushM)));
    if (inForbidden.test(t)) t = nudgeOutOfForbidden.apply(t);
    t = clampToFieldRobotSafe.apply(t);

    if (inForbidden.test(t) || violatesWall.test(t)) {
      Translation2d base = clampToFieldRobotSafe.apply(resource);
      if (inForbidden.test(base)) base = nudgeOutOfForbidden.apply(base);
      base = clampToFieldRobotSafe.apply(base);
      if (!inForbidden.test(base) && !violatesWall.test(base)) return base;
    }
    return t;
  }

  public Pose2d fallbackCollectPose(Pose2d robotPoseBlue) {
    // return new Pose2d(Constants.FIELD_LENGTH / 2, Constants.FIELD_WIDTH / 2, new Rotation2d());
    Translation2d p =
        collectStickyDriveTarget != null ? collectStickyDriveTarget : collectStickyPoint;
    if (p == null && lastBest != null) p = lastBest.point;
    if (p == null && robotPoseBlue != null) p = robotPoseBlue.getTranslation();
    if (p == null) p = new Translation2d();
    Rotation2d rot = robotPoseBlue != null ? robotPoseBlue.getRotation() : new Rotation2d();
    return new Pose2d(p, rot);
  }

  public Pose2d nextObjectiveGoalBlue(
      Pose2d robotPoseBlue, double ourSpeedCap, int goalUnits, CategorySpec cat) {
    if (robotPoseBlue == null) return Pose2d.kZero;
    double cap = Math.max(0.2, ourSpeedCap);

    for (int pass = 0; pass < 2; pass++) {
      FieldTrackerCollectPassSetupResult setup =
          FieldTrackerCollectPassSetup.prepare(this, robotPoseBlue, cap);
      if (setup.immediatePose() != null) return setup.immediatePose();

      FieldTrackerCollectPassContext ctx = setup.context();
      FieldTrackerCollectPassCandidateResult candidate =
          FieldTrackerCollectPassCandidateStep.choose(this, ctx, goalUnits);
      if (candidate.immediatePose() != null) return candidate.immediatePose();

      FieldTrackerCollectPassStickyResult sticky =
          FieldTrackerCollectPassStickyStep.selectAndPrime(this, ctx, candidate, pass);
      if (sticky.immediatePose() != null) return sticky.immediatePose();

      return FieldTrackerCollectPassDriveStep.driveAndFinish(this, ctx, candidate, sticky, pass);
    }

    clearCollectSticky();
    return fallbackCollectPose(robotPoseBlue);
  }

  public Translation2d forceDriveTargetOntoFuel(
      Translation2d robotPos,
      double cap,
      Translation2d collectPoint,
      Translation2d driveSeed,
      java.util.function.Function<Translation2d, Translation2d> clampToFieldRobotSafe,
      java.util.function.Predicate<Translation2d> inForbidden,
      java.util.function.Predicate<Translation2d> violatesWall,
      java.util.function.Function<Translation2d, Translation2d> nudgeOutOfForbidden) {

    Translation2d drive = driveSeed != null ? driveSeed : collectPoint;
    if (drive == null) return null;

    drive = clampToFieldRobotSafe.apply(drive);
    if (inForbidden.test(drive)) drive = nudgeOutOfForbidden.apply(drive);
    drive = clampToFieldRobotSafe.apply(drive);

    if (inForbidden.test(drive) || violatesWall.test(drive)) return null;

    Translation2d near = predictor.nearestCollectResource(drive, COLLECT_SNAP_TO_NEAREST_FUEL_M);
    if (near != null) {
      Translation2d snapped = clampToFieldRobotSafe.apply(near);
      if (inForbidden.test(snapped)) snapped = nudgeOutOfForbidden.apply(snapped);
      snapped = clampToFieldRobotSafe.apply(snapped);
      if (!inForbidden.test(snapped) && !violatesWall.test(snapped)) return snapped;
    }

    Translation2d centroid = predictor.snapToCollectCentroid(drive, 0.75, 0.15);
    if (centroid != null) {
      centroid = clampToFieldRobotSafe.apply(centroid);
      if (inForbidden.test(centroid)) centroid = nudgeOutOfForbidden.apply(centroid);
      centroid = clampToFieldRobotSafe.apply(centroid);
      if (!inForbidden.test(centroid) && !violatesWall.test(centroid)) {
        Translation2d near2 =
            predictor.nearestCollectResource(centroid, COLLECT_SNAP_TO_NEAREST_FUEL_M);
        if (near2 != null) {
          Translation2d snapped2 = clampToFieldRobotSafe.apply(near2);
          if (inForbidden.test(snapped2)) snapped2 = nudgeOutOfForbidden.apply(snapped2);
          snapped2 = clampToFieldRobotSafe.apply(snapped2);
          if (!inForbidden.test(snapped2) && !violatesWall.test(snapped2)) return snapped2;
        }
        return centroid;
      }
    }

    Translation2d fallback = clampToFieldRobotSafe.apply(collectPoint);
    if (inForbidden.test(fallback)) fallback = nudgeOutOfForbidden.apply(fallback);
    fallback = clampToFieldRobotSafe.apply(fallback);
    if (!inForbidden.test(fallback) && !violatesWall.test(fallback)) return fallback;

    return drive;
  }

  public void clearState() {
    clearCollectSticky();
  }
}
