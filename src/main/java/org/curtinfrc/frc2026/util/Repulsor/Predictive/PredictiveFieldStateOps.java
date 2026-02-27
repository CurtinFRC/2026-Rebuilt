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

package org.curtinfrc.frc2026.util.Repulsor.Predictive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.function.Predicate;
import java.util.function.Supplier;
import org.curtinfrc.frc2026.util.Repulsor.Constants;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldMapBuilder.CategorySpec;
import org.curtinfrc.frc2026.util.Repulsor.IntakeFootprint;
import org.curtinfrc.frc2026.util.Repulsor.Interval;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Internal.CollectEval;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Internal.FootprintEval;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Internal.HeadingPick;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Internal.IntentAgg;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Internal.IntentAggCont;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Internal.ResourceRegions;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Internal.Track;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.Candidate;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.CollectProbe;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.DynamicObject;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.PointCandidate;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.ResourceSpec;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Runtime.*;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.RepulsorSetpoint;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Model.Alliance;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Model.GameElement;

public final class PredictiveFieldStateOps {

  public static boolean error() {
    return false;
  }

  public CollectProbe probeCollect(Translation2d p) {
    return probeCollect(p, 0.75);
  }

  public CollectProbe probeCollect(Translation2d p, double countR) {
    if (p == null) return new CollectProbe(0, 0.0);
    SpatialDyn dyn = cachedDyn();
    if (dyn == null) return new CollectProbe(0, 0.0);
    int c = dyn.countResourcesWithin(p, Math.max(0.05, countR));
    double u = dyn.valueAt(p);
    return new CollectProbe(c, u);
  }

  public boolean footprintHasCollectResource(Translation2d center, double cellM) {
    if (center == null) return false;
    SpatialDyn dyn = cachedDyn();
    if (dyn == null || dyn.resources.isEmpty()) return false;
    if (dyn != lastFootprintDyn) {
      footprintCache.clear();
      lastFootprintDyn = dyn;
    }
    long key = footprintKey(center, cellM);
    Boolean cached = footprintCache.get(key);
    if (cached != null) return cached;
    double rCore = coreRadiusFor(cellM);
    double searchR = Math.max(0.70, rCore * 3.0);
    Translation2d nearest = dyn.nearestResourceTo(center, searchR);
    if (nearest == null) {
      footprintCache.put(key, false);
      return false;
    }
    double minUnits =
        Math.max(
            0.02, Math.min(COLLECT_FINE_MIN_UNITS, dynamicMinUnits(dyn.totalEvidence()) * 0.75));
    Rotation2d base = face(center, nearest, Rotation2d.kZero);
    HeadingPick pick = bestHeadingForFootprint(dyn, center, nearest, base, rCore, minUnits);
    boolean ok = pick != null;
    if (footprintCache.size() > 2048) footprintCache.clear();
    footprintCache.put(key, ok);
    return ok;
  }

  public boolean footprintHasFuel(Translation2d center, double cellM) {
    return footprintHasCollectResource(center, cellM);
  }

  public void markCollectDepleted(Translation2d p, double cellM, double strength) {
    addDepletedMark(p, Math.max(0.10, cellM * 2.0), strength, DEPLETED_TTL_S, false);
  }

  public static final double MIN_DT = 0.02;
  public static final double MAX_MEAS_DT = 0.20;
  public static final double ETA_FLOOR = 0.05;
  public static final double DEFAULT_ENEMY_SPEED = 2.2;
  public static final double DEFAULT_ALLY_SPEED = 3.0;
  public static final double DEFAULT_OUR_SPEED = 3.5;
  public static final double ACC_LIMIT = 2.5;
  public static final double VEL_EMA = 0.35;
  public static final double POS_EMA = 0.15;
  static final double SOFTMAX_TEMP = 1.35;
  public static final double RESERVATION_RADIUS = 0.85;
  public static final double KERNEL_SIGMA = 0.95;
  public static final double HYST_PERSIST_S = 0.8;
  public static final double HYST_BONUS = 0.22;
  public static final double ADV_GAIN = 1.1;
  public static final double DIST_COST = 0.10;
  public static final double PRESSURE_GAIN = 0.72;
  public static final double CONGEST_COST = 0.95;
  public static final double CAPACITY_GAIN = 0.45;
  public static final double HEADING_GAIN = 0.18;

  public static final double COLLECT_VALUE_GAIN = 1.10;
  public static final double COLLECT_ETA_COST = 1.05;
  public static final double COLLECT_ENEMY_PRESS_COST = 1.15;
  public static final double COLLECT_ALLY_CONGEST_COST = 0.95;
  public static final double COLLECT_ENEMY_INTENT_COST = 0.75;
  public static final double COLLECT_ALLY_INTENT_COST = 0.55;
  public static final double COLLECT_VALUE_SAT_K = 0.75;
  public static final double COLLECT_AGE_DECAY = 1.25;
  public static final double COLLECT_LOCAL_AVOID_R = 0.9;
  public static final double COLLECT_ACTIVITY_SIGMA = 1.05;
  public static final double COLLECT_ACTIVITY_ALLY_W = 0.80;
  public static final double COLLECT_ACTIVITY_ENEMY_W = 0.55;
  public static final double COLLECT_ACTIVITY_DYN_W = 0.60;
  public static final double COLLECT_REGION_SAMPLES_W = 2.70;
  public static final double COLLECT_CELL_M = 0.10;
  public static final double COLLECT_NEAR_BONUS = 0.85;
  public static final double COLLECT_NEAR_DECAY = 1.1; // 1.35;
  public static final double COLLECT_SPREAD_SCORE_R = 0.85;
  public static final double COLLECT_SPREAD_MIN = 0.30;
  public static final double COLLECT_SPREAD_MAX = 0.65;
  public static final double SHOOT_X_END_BAND_M = 12.5631260802;
  public static final double BAND_WIDTH_M = 2.167294751;
  public static final Interval<Double> X_LEFT_BAND =
      Interval.closed(SHOOT_X_END_BAND_M - BAND_WIDTH_M, SHOOT_X_END_BAND_M);
  public static final Interval<Double> X_RIGHT_BAND =
      Interval.closed(
          Constants.FIELD_LENGTH - SHOOT_X_END_BAND_M,
          Constants.FIELD_LENGTH - (SHOOT_X_END_BAND_M - BAND_WIDTH_M));
  public static final String DEFAULT_COLLECT_RESOURCE_TYPE = "fuel";

  public static final double COLLECT_CORE_R_MIN = 0.05;
  public static final double COLLECT_CORE_R_MAX = 0.12;
  public static final double COLLECT_SNAP_R_MIN = 0.35;
  public static final double COLLECT_SNAP_R_MAX = 0.60;
  public static final double COLLECT_MICRO_CENTROID_R_MIN = 0.18;
  public static final double COLLECT_MICRO_CENTROID_R_MAX = 0.35;
  public static final double COLLECT_JITTER_R_MIN = 0.06;
  public static final double COLLECT_JITTER_R_MAX = 0.14;
  public static final double COLLECT_HOLE_PENALTY = 2.35;
  public static final double COLLECT_EDGE_PENALTY = 0.85;
  public static final double COLLECT_NOFUEL_PENALTY = 3.00;

  static final int COLLECT_GRID_TOPK = 420;
  static final int COLLECT_RESOURCE_SEEDS_MAX = 64;
  static final double COLLECT_CAND_GATE_R = 0.60;
  static final double COLLECT_PEAK_GATE_R = 0.60;
  static final double COLLECT_PEAK_SCORE_R = 0.85;

  public static final double COLLECT_COARSE_MIN_REGION_UNITS = 0.12;
  public static final double COLLECT_FINE_MIN_UNITS = 0.08;

  static final double RESOURCE_SIGMA_ABS_MAX = 0.45;
  static final double RESOURCE_SIGMA_REL_MAX = 1.25;
  static final double RESOURCE_SIGMA_MIN = 0.06;

  static final int COLLECT_CLUSTER_MAX = 72;
  static final double COLLECT_CLUSTER_BIN_M = 0.14;
  static final int COLLECT_GRID_FALLBACK_MAX = 1400;

  public static final double DEPLETED_TTL_S = 3.25;
  public static final double DEPLETED_PEN_W = 2.75;
  public static final double DEPLETED_MARK_NEAR_M = 0.65;
  public static final double DEPLETED_MARK_EMPTY_UNITS = 0.07;

  static final double RESOURCE_HARD_MAX_AGE_S = 0.95;

  static final int COLLECT_SHORTLIST_ETA = 36;
  static final int COLLECT_SHORTLIST_COARSE = 28;
  public static final int COLLECT_FINE_OFFSETS_GRID = 3;
  public static final double COLLECT_FINE_OFFSETS_SCALE = 0.65;

  public static final double COLLECT_COMMIT_MIN_S = 0.12;
  public static final double COLLECT_COMMIT_MAX_S = 0.34;
  public static final double COLLECT_SWITCH_BASE = 0.12;
  public static final double COLLECT_SWITCH_ETA_W = 0.07;

  public static final double COLLECT_PROGRESS_MIN_DROP_M = 0.25;
  public static final double COLLECT_PROGRESS_WINDOW_S = 0.70;

  public static final double COLLECT_ARRIVE_R = 0.75;
  public static final double COLLECT_ARRIVE_VERIFY_S = 0.28;
  public static final double COLLECT_FAIL_COOLDOWN_S = 2.10;

  public static final double EVIDENCE_R = 0.85;
  public static final double EVIDENCE_MIN_BASE = 0.05;
  public static final double EVIDENCE_MIN_MAX = 0.18;

  public static final double ACTIVITY_CAP = 1.05;

  static final int PATH_SAMPLES = 7;
  static final double PATH_COST_W = 0.60;

  static final int ENEMY_REGIONS_MAX = 24;
  static final double ENEMY_REGION_SIGMA = 0.85;

  public static final double RES_OVERLAP_R = 0.75;
  public static final double RES_OVERLAP_GAIN = 1.05;

  static final int PEAK_FINDER_TOPN = 28;

  double[] scratchLogits = new double[0];
  double[] scratchLogits2 = new double[0];

  volatile List<DynamicObject> lastDynRef = null;
  volatile SpatialDyn lastDyn = null;
  public volatile int specsVersion = 0;
  volatile int lastDynSpecsVersion = -1;
  volatile SpatialDyn lastFootprintDyn = null;
  final HashMap<Long, Boolean> footprintCache = new HashMap<>(512);

  void ensureScratch(int n) {
    if (scratchLogits.length < n)
      scratchLogits = new double[Math.max(n, scratchLogits.length * 2 + 8)];
    if (scratchLogits2.length < n)
      scratchLogits2 = new double[Math.max(n, scratchLogits2.length * 2 + 8)];
  }

  public SpatialDyn cachedDyn() {
    List<DynamicObject> ref = dynamicObjects;
    int sv = specsVersion;
    SpatialDyn d = lastDyn;
    if (d != null && ref == lastDynRef && sv == lastDynSpecsVersion) return d;
    SpatialDyn nd =
        new SpatialDyn(
            ref,
            resourceSpecs,
            otherTypeWeights,
            collectResourceTypes,
            collectResourcePositionFilter);
    lastDynRef = ref;
    lastDyn = nd;
    lastDynSpecsVersion = sv;
    return nd;
  }

  public void invalidateDynCache() {
    lastDynRef = null;
    lastDyn = null;
    lastFootprintDyn = null;
    footprintCache.clear();
  }

  public final HashMap<Integer, Track> allyMap = new HashMap<>();
  public final HashMap<Integer, Track> enemyMap = new HashMap<>();

  public volatile List<GameElement> worldElements = List.of();
  public volatile Alliance ourAlliance =
      DriverStation.getAlliance()
          .map(al -> (al == DriverStation.Alliance.Blue) ? Alliance.kBlue : Alliance.kRed)
          .orElse(Alliance.kRed);

  public volatile RepulsorSetpoint lastChosen = null;
  public volatile double lastChosenTs = 0.0;

  public volatile List<DynamicObject> dynamicObjects = List.of();
  public final HashMap<String, ResourceSpec> resourceSpecs = new HashMap<>();
  public final HashMap<String, Double> otherTypeWeights = new HashMap<>();
  public final HashSet<String> collectResourceTypes =
      new HashSet<>(Set.of(DEFAULT_COLLECT_RESOURCE_TYPE));
  public volatile Predicate<Translation2d> collectResourcePositionFilter =
      PredictiveFieldStateOps::defaultCollectResourcePositionFilter;

  public final PredictiveCollectPenaltyTracker penaltyTracker =
      new PredictiveCollectPenaltyTracker();

  public volatile Translation2d lastReturnedCollect = null;
  public volatile double lastReturnedCollectTs = 0.0;

  public volatile Translation2d currentCollectTarget = null;
  public volatile double currentCollectChosenTs = 0.0;
  public volatile double currentCollectScore = -1e18;
  public double currentCollectUnits = 0.0;
  public double currentCollectEta = 0.0;

  public double collectProgressLastTs = 0.0;
  public double collectProgressLastDist = Double.POSITIVE_INFINITY;

  public double collectArrivalTs = -1.0;

  public volatile Translation2d lastOurPosForCollect = null;
  public double lastOurCapForCollect = DEFAULT_OUR_SPEED;
  public int lastGoalUnitsForCollect = 1;
  public double lastCellMForCollect = COLLECT_CELL_M;

  final PredictiveCollectSecondaryRankers.Api secondaryCollectApi =
      new PredictiveSecondaryCollectApi(this);

  public void registerResourceSpec(String type, ResourceSpec spec) {
    PredictiveCollectConfigRuntime.registerResourceSpec(this, type, spec);
  }

  public void registerOtherTypeWeight(String type, double weight) {
    PredictiveCollectConfigRuntime.registerOtherTypeWeight(this, type, weight);
  }

  public void setCollectResourceTypes(Set<String> types) {
    PredictiveCollectConfigRuntime.setCollectResourceTypes(this, types);
  }

  public Set<String> getCollectResourceTypes() {
    return PredictiveCollectConfigRuntime.getCollectResourceTypes(this);
  }

  public void addCollectResourceType(String type) {
    PredictiveCollectConfigRuntime.addCollectResourceType(this, type);
  }

  public void removeCollectResourceType(String type) {
    PredictiveCollectConfigRuntime.removeCollectResourceType(this, type);
  }

  public boolean isCollectResourceType(String type) {
    return PredictiveCollectConfigRuntime.isCollectResourceType(this, type);
  }

  public void setCollectResourcePositionFilter(Predicate<Translation2d> filter) {
    PredictiveCollectConfigRuntime.setCollectResourcePositionFilter(this, filter);
  }

  public void setDynamicObjects(List<DynamicObject> objs) {
    PredictiveCollectConfigRuntime.setDynamicObjects(this, objs);
  }

  public void setWorld(List<GameElement> elements, Alliance ours) {
    PredictiveCollectConfigRuntime.setWorld(this, elements, ours);
  }

  public void updateAlly(int id, Translation2d pos, Translation2d velHint, Double speedCap) {
    PredictiveFieldStateTrackingRuntime.updateAlly(this, id, pos, velHint, speedCap);
  }

  public void updateEnemy(int id, Translation2d pos, Translation2d velHint, Double speedCap) {
    PredictiveFieldStateTrackingRuntime.updateEnemy(this, id, pos, velHint, speedCap);
  }

  public void clearStale(double maxAgeS) {
    PredictiveFieldStateTrackingRuntime.clearStale(this, maxAgeS);
  }

  public List<Candidate> rank(
      Translation2d ourPos, double ourSpeedCap, CategorySpec cat, int limit) {
    return PredictiveFieldStateTrackingRuntime.rank(this, ourPos, ourSpeedCap, cat, limit);
  }

  static void sortIdxByKey(double[] key, int[] idx) {
    PredictiveFieldStateTrackingRuntime.sortIdxByKey(key, idx);
  }

  static void quickSortIdx(double[] key, int[] idx, int lo, int hi) {
    PredictiveFieldStateTrackingRuntime.quickSortIdx(key, idx, lo, hi);
  }

  public List<RepulsorSetpoint> rankSetpoints(
      Translation2d ourPos, double ourSpeedCap, CategorySpec cat, int limit) {
    return PredictiveFieldStateTrackingRuntime.rankSetpoints(this, ourPos, ourSpeedCap, cat, limit);
  }

  public int resourceObservationCount() {
    return PredictiveFieldStateTrackingRuntime.resourceObservationCount(this);
  }

  public Translation2d snapToCollectCentroid(Translation2d seed, double r, double minMass) {
    return PredictiveFieldStateTrackingRuntime.snapToCollectCentroid(this, seed, r, minMass);
  }

  public Translation2d nearestCollectResource(Translation2d p, double maxDist) {
    return PredictiveFieldStateTrackingRuntime.nearestCollectResource(this, p, maxDist);
  }

  Translation2d[] peakFinder(Translation2d[] gridPoints, SpatialDyn dyn, int topN) {
    return PredictiveCollectCandidateBuilder.peakFinder(gridPoints, dyn, topN);
  }

  public static final Supplier<IntakeFootprint> COLLECT_INTAKE = IntakeFootprint::getFootprint;
  public static final Translation2d[] COLLECT_FOOTPRINT_SAMPLES =
      new Translation2d[] {
        new Translation2d(0.0, 0.0),
        new Translation2d(0.24, 0.0),
        new Translation2d(-0.24, 0.0),
        new Translation2d(0.0, 0.24),
        new Translation2d(0.0, -0.24),
        new Translation2d(0.18, 0.18),
        new Translation2d(0.18, -0.18),
        new Translation2d(-0.18, 0.18),
        new Translation2d(-0.18, -0.18)
      };
  public static final double COLLECT_KEEP_BONUS = 0.20;

  public Rotation2d currentCollectHeading = new Rotation2d();
  public volatile Translation2d currentCollectTouch = null;

  public static Rotation2d face(Translation2d from, Translation2d to, Rotation2d fallback) {
    return PredictiveCollectFootprintRuntime.face(from, to, fallback);
  }

  public Translation2d resolveCollectTouch(
      SpatialDyn dyn,
      Translation2d center,
      Rotation2d heading,
      double rCore,
      double rSnap,
      double rCentroid) {
    return PredictiveCollectFootprintRuntime.resolveCollectTouch(
        this, dyn, center, heading, rCore, rSnap, rCentroid);
  }

  FootprintEval evalFootprint(
      SpatialDyn dyn, Translation2d center, Rotation2d heading, double rCore) {
    return PredictiveCollectFootprintRuntime.evalFootprint(this, dyn, center, heading, rCore);
  }

  void fillFootprintEvidence(
      SpatialDyn dyn, Translation2d center, Rotation2d heading, FootprintEval e) {
    PredictiveCollectFootprintRuntime.fillFootprintEvidence(this, dyn, center, heading, e);
  }

  public boolean footprintOk(
      SpatialDyn dyn, Translation2d center, Rotation2d heading, double rCore, double minUnits) {
    return PredictiveCollectFootprintRuntime.footprintOk(
        this, dyn, center, heading, rCore, minUnits);
  }

  public HeadingPick bestHeadingForFootprint(
      SpatialDyn dyn,
      Translation2d desiredCenter,
      Translation2d fuelTouch,
      Rotation2d baseHeading,
      double rCore,
      double minUnits) {
    return PredictiveCollectFootprintRuntime.bestHeadingForFootprint(
        this, dyn, desiredCenter, fuelTouch, baseHeading, rCore, minUnits);
  }

  boolean compareFootprintEvidence(
      SpatialDyn dyn, Translation2d p, Rotation2d h, FootprintEval fp, HeadingPick best) {
    return PredictiveCollectFootprintRuntime.compareFootprintEvidence(this, dyn, p, h, fp, best);
  }

  static long footprintKey(Translation2d center, double cellM) {
    return PredictiveCollectFootprintRuntime.footprintKey(center, cellM);
  }

  public PointCandidate rankCollectNearest(
      Translation2d ourPos,
      double ourSpeedCap,
      Translation2d[] points,
      double cellM,
      int goalUnits,
      int limit) {

    if (ourPos == null) return null;

    SpatialDyn dyn = cachedDyn();
    if (dyn == null || dyn.resources.isEmpty()) return null;

    final double robotHalf =
        0.5
            * Math.max(
                org.curtinfrc.frc2026.Constants.ROBOT_X, org.curtinfrc.frc2026.Constants.ROBOT_Y);
    final double robotWallMargin = robotHalf + 0.03;
    final double wallClearMin = robotWallMargin + 0.05;
    final java.util.function.ToDoubleFunction<Translation2d> wallPenalty =
        (pt) -> {
          if (pt == null) return 0.0;
          double d = wallDistance(pt);
          if (d >= wallClearMin) return 0.0;
          double t = (wallClearMin - d) / Math.max(1e-6, wallClearMin);
          return 1.4 * t * t;
        };

    final java.util.function.Predicate<Translation2d> inShootBand =
        (pt) -> {
          if (pt == null) return true;
          double x = pt.getX();
          double y = pt.getY();
          return x < robotWallMargin
              || x > (Constants.FIELD_LENGTH - robotWallMargin)
              || y < robotWallMargin
              || y > (Constants.FIELD_WIDTH - robotWallMargin);
        };

    lastOurPosForCollect = ourPos;
    lastOurCapForCollect = ourSpeedCap > 0.0 ? ourSpeedCap : DEFAULT_OUR_SPEED;
    lastGoalUnitsForCollect = Math.max(1, goalUnits);
    lastCellMForCollect = Math.max(0.10, cellM);

    sweepDepletedMarks();

    Translation2d[] seeds = buildCollectCandidates(points, dyn);
    if (seeds.length == 0) return null;

    double cap = lastOurCapForCollect;
    int goal = lastGoalUnitsForCollect;
    double half = Math.max(0.05, cellM * 0.5);

    double totalEv = dyn.totalEvidence();
    double minUnits = dynamicMinUnits(totalEv);
    int minCount = dynamicMinCount(totalEv);
    double minEv = minEvidence(totalEv);

    double nearHalf = Math.max(0.16, Math.max(half, cellM * 0.85));
    double nearR = Math.max(COLLECT_ARRIVE_R, 0.60);

    double onHalf = Math.max(0.10, Math.min(nearHalf * 0.60, 0.22));
    double onR = Math.max(0.20, Math.min(0.30, Math.max(0.20, COLLECT_ARRIVE_R * 0.70)));

    double rCore = coreRadiusFor(cellM);
    double rSnap = snapRadiusFor(cellM);
    double rCentroid = microCentroidRadiusFor(cellM);
    double rJitter = jitterRadiusFor(cellM);

    double minHardUnits = Math.max(0.025, Math.min(COLLECT_FINE_MIN_UNITS, minUnits * 0.80));
    double footprintMinUnits = Math.max(minHardUnits, minUnits * 0.95);

    if (lastReturnedCollect != null) {
      double now = Timer.getFPGATimestamp();
      double age = now - lastReturnedCollectTs;
      if (age >= 0.0
          && age <= 0.55
          && ourPos.getDistance(lastReturnedCollect) <= DEPLETED_MARK_NEAR_M) {
        double u = dyn.valueAt(lastReturnedCollect);
        if (u < DEPLETED_MARK_EMPTY_UNITS) {
          addDepletedMark(lastReturnedCollect, 0.65, 1.35, DEPLETED_TTL_S, false);
          addDepletedRing(lastReturnedCollect, 0.35, 0.95, 0.85, DEPLETED_TTL_S);
        }
      }
    }

    int n = seeds.length;
    double[] etaKey = new double[n];
    double[] coarseKey = new double[n];
    int[] orderEta = new int[n];
    int[] orderCoarse = new int[n];

    ResourceRegions enemyRegions = buildResourceRegions(dyn, ENEMY_REGIONS_MAX);
    IntentAggCont enemyIntent = enemyIntentToRegions(enemyMap, enemyRegions);
    IntentAggCont allyIntent = allyIntentToRegions(allyMap, enemyRegions);

    for (int i = 0; i < n; i++) {
      Translation2d p = seeds[i];
      orderEta[i] = i;
      orderCoarse[i] = i;

      if (p == null || inShootBand.test(p)) {
        etaKey[i] = Double.POSITIVE_INFINITY;
        coarseKey[i] = 1e18;
        continue;
      }

      double eta = estimateTravelTime(ourPos, p, cap);
      etaKey[i] = eta;

      double regionUnits = dyn.valueInSquare(p, half);
      double dep = depletedPenaltySoft(p);
      double ev = dyn.evidenceMassWithin(p, EVIDENCE_R);
      double activity =
          Math.min(
              ACTIVITY_CAP,
              COLLECT_ACTIVITY_ALLY_W * radialDensity(allyMap, p, COLLECT_ACTIVITY_SIGMA)
                  + COLLECT_ACTIVITY_ENEMY_W * radialDensity(enemyMap, p, COLLECT_ACTIVITY_SIGMA)
                  + COLLECT_ACTIVITY_DYN_W * dyn.otherDensity(p, COLLECT_ACTIVITY_SIGMA));
      double wallPen = wallPenalty.applyAsDouble(p);
      int coarseCore = dyn.countResourcesWithin(p, Math.max(0.04, rCore));

      double evGate = ev < minEv ? -2.25 : 0.0;
      double coarseScore =
          regionUnits * COLLECT_REGION_SAMPLES_W
              - eta * 0.55
              - activity * 0.70
              - dep * DEPLETED_PEN_W
              - wallPen
              + evGate
              + Math.min(0.6, 0.25 * coarseCore);

      coarseKey[i] = -coarseScore;
    }

    sortIdxByKey(etaKey, orderEta);
    sortIdxByKey(coarseKey, orderCoarse);

    int shortlistCap =
        Math.max(
            12,
            Math.min(
                n,
                Math.max(
                    Math.min(COLLECT_SHORTLIST_ETA, n), Math.min(COLLECT_SHORTLIST_COARSE, n))));

    boolean[] chosen = new boolean[n];
    ArrayList<Integer> shortlist = new ArrayList<>(shortlistCap * 2);

    int kEta = Math.min(n, Math.max(1, Math.min(COLLECT_SHORTLIST_ETA, shortlistCap + 8)));
    int kCoarse = Math.min(n, Math.max(1, Math.min(COLLECT_SHORTLIST_COARSE, shortlistCap + 8)));

    for (int i = 0; i < kEta && shortlist.size() < shortlistCap; i++) {
      int idx = orderEta[i];
      if (!chosen[idx]) {
        chosen[idx] = true;
        shortlist.add(idx);
      }
    }
    for (int i = 0; i < kCoarse && shortlist.size() < shortlistCap; i++) {
      int idx = orderCoarse[i];
      if (!chosen[idx]) {
        chosen[idx] = true;
        shortlist.add(idx);
      }
    }

    int maxCheck =
        limit > 0 ? Math.min(Math.max(1, limit), Math.max(1, shortlist.size())) : shortlist.size();

    PredictiveCollectNearestSearchResult search =
        PredictiveCollectNearestSearchStep.search(
            this,
            seeds,
            shortlist,
            maxCheck,
            ourPos,
            cap,
            goal,
            cellM,
            dyn,
            enemyIntent,
            allyIntent,
            inShootBand,
            wallPenalty,
            nearHalf,
            nearR,
            onHalf,
            onR,
            minHardUnits,
            minUnits,
            minCount,
            minEv,
            rCore,
            rSnap,
            rCentroid,
            footprintMinUnits);

    Translation2d bestP = search.bestPoint();
    Translation2d bestTouch = search.bestTouch();
    Rotation2d bestHeading = search.bestHeading();
    CollectEval bestE = search.bestEval();

    if (bestP == null || bestE == null || bestTouch == null || inShootBand.test(bestP)) {
      return null;
    }

    Translation2d chosenPt = bestP;
    Translation2d chosenTouch = bestTouch;
    Rotation2d chosenHeading = bestHeading;
    CollectEval chosenE = bestE;

    return PredictiveCollectNearestResolutionStep.resolve(
        this,
        ourPos,
        cap,
        goal,
        cellM,
        dyn,
        enemyIntent,
        allyIntent,
        inShootBand,
        chosenPt,
        chosenTouch,
        chosenHeading,
        chosenE,
        totalEv,
        minUnits,
        minCount,
        minEv,
        onHalf,
        onR,
        minHardUnits,
        footprintMinUnits,
        rCore,
        rSnap,
        rCentroid);
  }

  public PointCandidate rankCollectHierarchical(
      Translation2d ourPos,
      double ourSpeedCap,
      Translation2d[] points,
      double cellM,
      int goalUnits,
      int coarseTopK,
      int refineGrid) {
    return PredictiveCollectSecondaryRankers.rankCollectHierarchical(
        secondaryCollectApi, ourPos, ourSpeedCap, points, cellM, goalUnits, coarseTopK, refineGrid);
  }

  public Translation2d bestCollectHotspot(Translation2d[] points, double cellM) {
    return PredictiveCollectSecondaryRankers.bestCollectHotspot(secondaryCollectApi, points, cellM);
  }

  public PointCandidate rankCollectPoints(
      Translation2d ourPos, double ourSpeedCap, Translation2d[] points, int goalUnits, int limit) {
    return PredictiveCollectSecondaryRankers.rankCollectPoints(
        secondaryCollectApi, ourPos, ourSpeedCap, points, goalUnits, limit);
  }

  public Translation2d[] buildCollectCandidates(Translation2d[] gridPoints, SpatialDyn dyn) {
    return PredictiveCollectCandidateBuilder.buildCollectCandidates(
        gridPoints, dyn, PEAK_FINDER_TOPN);
  }

  public ArrayList<Translation2d> dedupPoints(ArrayList<Translation2d> in, double dupSkipM) {
    return PredictiveCollectCandidateBuilder.dedupPoints(in, dupSkipM);
  }

  public ArrayList<Translation2d> spreadCollectPoints(ArrayList<Translation2d> in, SpatialDyn dyn) {
    return PredictiveCollectCandidateBuilder.spreadCollectPoints(in, dyn);
  }

  public double adaptiveDupSkip(SpatialDyn dyn) {
    return PredictiveCollectCandidateBuilder.adaptiveDupSkip(dyn);
  }

  public double adaptiveCollectSeparation(SpatialDyn dyn) {
    return PredictiveCollectCandidateBuilder.adaptiveCollectSeparation(dyn);
  }

  public Translation2d[] buildResourceClustersMulti(SpatialDyn dyn, int maxClusters) {
    return PredictiveCollectCandidateBuilder.buildResourceClustersMulti(dyn, maxClusters);
  }

  Translation2d meanShiftRefine(SpatialDyn dyn, Translation2d seed, double r, int iters) {
    return PredictiveCollectCandidateBuilder.meanShiftRefine(dyn, seed, r, iters);
  }

  public void sweepDepletedMarks() {
    penaltyTracker.sweepDepletedMarks(error());
  }

  public void addDepletedMark(
      Translation2d p, double radiusM, double strength, double ttlS, boolean merge) {
    penaltyTracker.addDepletedMark(p, radiusM, strength, ttlS, merge, error());
  }

  public void addDepletedRing(Translation2d p, double r0, double r1, double strength, double ttlS) {
    penaltyTracker.addDepletedRing(p, r0, r1, strength, ttlS, error());
  }

  public double depletedPenaltySoft(Translation2d p) {
    return penaltyTracker.depletedPenaltySoft(p, error());
  }

  public static Translation2d lerpVec(Translation2d a, Translation2d b, double alpha) {
    double t = Math.max(0.0, Math.min(1.0, alpha));
    return new Translation2d(
        a.getX() + (b.getX() - a.getX()) * t, a.getY() + (b.getY() - a.getY()) * t);
  }

  public static double emaAlpha(double baseAlpha, double dt) {
    double a = Math.max(0.0, Math.min(1.0, baseAlpha));
    double x = Math.max(0.0, dt / MIN_DT);
    return 1.0 - Math.pow(1.0 - a, x);
  }

  public static Translation2d clampDeltaV(
      Translation2d oldV, Translation2d newV, double aMax, double dt) {
    double lim = Math.max(0.0, aMax) * Math.max(0.0, dt);
    Translation2d dv = newV.minus(oldV);
    double n = dv.getNorm();
    if (n <= lim || n <= 1e-9) return newV;
    Translation2d dvClamped = dv.div(n).times(lim);
    return oldV.plus(dvClamped);
  }

  static double etaPath(Translation2d a, Translation2d b, double speed) {
    double d = a.getDistance(b);
    return Math.max(ETA_FLOOR, d / Math.max(0.1, speed));
  }

  public double estimateTravelTime(Translation2d a, Translation2d b, double speed) {
    return PredictiveCollectDynamics.estimateTravelTime(
        a, b, speed, cachedDyn(), allyMap, enemyMap);
  }

  public static double minEtaToTarget(HashMap<Integer, Track> map, Translation2d t) {
    return PredictiveCollectDynamics.minEtaToTarget(map, t);
  }

  static double dotNorm(Translation2d a, Translation2d b) {
    return PredictiveCollectDynamics.dotNorm(a, b);
  }

  double radialKernel(double dist) {
    return PredictiveCollectDynamics.radialKernel(dist);
  }

  static double densityKernel(double d, double sigma) {
    return PredictiveCollectDynamics.densityKernel(d, sigma);
  }

  public static double radialDensity(
      HashMap<Integer, Track> map, Translation2d target, double sigma) {
    return PredictiveCollectDynamics.radialDensity(map, target, sigma);
  }

  public static Translation2d predictAt(Track r, double horizonS) {
    return PredictiveCollectDynamics.predictAt(r, horizonS);
  }

  public double radialPressure(
      HashMap<Integer, Track> enemies,
      Translation2d target,
      double ourEtaS,
      double intentMass,
      int count) {
    return PredictiveCollectDynamics.radialPressure(
        enemies, target, ourEtaS, RESERVATION_RADIUS, intentMass, count);
  }

  public double radialCongestion(
      HashMap<Integer, Track> allies,
      Translation2d target,
      double ourEtaS,
      double intentMass,
      int count) {
    return PredictiveCollectDynamics.radialCongestion(
        allies, target, ourEtaS, RESERVATION_RADIUS, intentMass, count);
  }

  public double headingAffinity(
      Translation2d ourPos,
      Translation2d target,
      HashMap<Integer, Track> allies,
      HashMap<Integer, Track> enemies) {
    return PredictiveCollectDynamics.headingAffinity(ourPos, target, allies, enemies);
  }

  public IntentAgg softIntentAgg(HashMap<Integer, Track> map, List<Translation2d> targets) {
    return PredictiveCollectDynamics.softIntentAgg(map, targets);
  }

  public ResourceRegions buildResourceRegions(SpatialDyn dyn, int maxRegions) {
    return PredictiveCollectDynamics.buildResourceRegions(dyn, maxRegions);
  }

  public IntentAggCont enemyIntentToRegions(HashMap<Integer, Track> map, ResourceRegions regs) {
    return PredictiveCollectDynamics.enemyIntentToRegions(map, regs, ENEMY_REGION_SIGMA);
  }

  public IntentAggCont allyIntentToRegions(HashMap<Integer, Track> map, ResourceRegions regs) {
    return PredictiveCollectDynamics.allyIntentToRegions(map, regs, ENEMY_REGION_SIGMA);
  }

  static double clamp(double x, double lo, double hi) {
    return PredictiveCollectPlacementRuntime.clamp(x, lo, hi);
  }

  public static double coreRadiusFor(double cellM) {
    return PredictiveCollectPlacementRuntime.coreRadiusFor(cellM);
  }

  public static double snapRadiusFor(double cellM) {
    return PredictiveCollectPlacementRuntime.snapRadiusFor(cellM);
  }

  public static double microCentroidRadiusFor(double cellM) {
    return PredictiveCollectPlacementRuntime.microCentroidRadiusFor(cellM);
  }

  public static double jitterRadiusFor(double cellM) {
    return PredictiveCollectPlacementRuntime.jitterRadiusFor(cellM);
  }

  Translation2d snapToNearestThenMicroCentroid(
      Translation2d p, SpatialDyn dyn, double rSnap, double rCentroid, double minMass) {
    return PredictiveCollectPlacementRuntime.snapToNearestThenMicroCentroid(
        this, p, dyn, rSnap, rCentroid, minMass);
  }

  public Translation2d enforceHardStopOnFuel(
      SpatialDyn dyn,
      Translation2d p,
      double rCore,
      double rSnap,
      double rCentroid,
      double minMass) {
    return PredictiveCollectPlacementRuntime.enforceHardStopOnFuel(
        this, dyn, p, rCore, rSnap, rCentroid, minMass);
  }

  public double pickupRobustPenalty(SpatialDyn dyn, Translation2d p, double rCore, double jitterR) {
    return PredictiveCollectPlacementRuntime.pickupRobustPenalty(this, dyn, p, rCore, jitterR);
  }

  public CollectEval evalCollectPoint(
      Translation2d ourPos,
      double cap,
      Translation2d p,
      int goal,
      double cellM,
      SpatialDyn dyn,
      IntentAggCont enemyIntent,
      IntentAggCont allyIntent) {
    return PredictiveCollectScoringRuntime.evalCollectPoint(
        this, ourPos, cap, p, goal, cellM, dyn, enemyIntent, allyIntent);
  }

  double normalizeValue(SpatialDyn dyn, double value) {
    return PredictiveCollectScoringRuntime.normalizeValue(this, dyn, value);
  }

  double reservationOverlapPenalty(
      HashMap<Integer, Track> allies, Translation2d p, double ourEtaS) {
    return PredictiveCollectScoringRuntime.reservationOverlapPenalty(this, allies, p, ourEtaS);
  }

  public boolean shouldEscapeCurrentCollect(
      Translation2d ourPos,
      SpatialDyn dyn,
      double totalEv,
      double minUnits,
      int minCount,
      double cellM) {
    return PredictiveCollectScoringRuntime.shouldEscapeCurrentCollect(
        this, ourPos, dyn, totalEv, minUnits, minCount, cellM);
  }

  public double collectCommitWindow(double etaCurrent) {
    return PredictiveCollectScoringRuntime.collectCommitWindow(this, etaCurrent);
  }

  public double minEvidence(double totalEvidence) {
    return PredictiveCollectScoringRuntime.minEvidence(this, totalEvidence);
  }

  public double dynamicMinUnits(double totalEvidence) {
    return PredictiveCollectScoringRuntime.dynamicMinUnits(this, totalEvidence);
  }

  public int dynamicMinCount(double totalEvidence) {
    return PredictiveCollectScoringRuntime.dynamicMinCount(this, totalEvidence);
  }

  public void recordRegionAttempt(SpatialDyn dyn, Translation2d p, double now, boolean success) {
    PredictiveCollectScoringRuntime.recordRegionAttempt(this, dyn, p, now, success);
  }

  public double regionBanditBonus(SpatialDyn dyn, Translation2d p, double now) {
    return PredictiveCollectScoringRuntime.regionBanditBonus(this, dyn, p, now);
  }

  static double lerp(double a, double b, double t) {
    return PredictiveCollectScoringRuntime.lerp(null, a, b, t);
  }

  static double clamp01(double x) {
    return PredictiveCollectScoringRuntime.clamp01(null, x);
  }

  static double wallDistance(Translation2d p) {
    return PredictiveCollectScoringRuntime.wallDistance(null, p);
  }

  static boolean isInvalidFuelBand(Translation2d p) {
    return PredictiveCollectScoringRuntime.isInvalidFuelBand(null, p);
  }

  public static boolean defaultCollectResourcePositionFilter(Translation2d p) {
    return PredictiveCollectScoringRuntime.defaultCollectResourcePositionFilter(null, p);
  }
}
