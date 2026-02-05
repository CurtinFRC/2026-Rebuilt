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

package org.curtinfrc.frc2026.util.Repulsor;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Objects;
import org.curtinfrc.frc2026.util.Repulsor.FieldTracker.GameElement;
import org.curtinfrc.frc2026.util.Repulsor.FieldTracker.GameElement.Alliance;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldMapBuilder.CategorySpec;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.RepulsorSetpoint;
import org.littletonrobotics.junction.Logger;

public final class PredictiveFieldState {

  public static boolean error() {
    return false;
  }

  public static final class CollectProbe {
    public final int count;
    public final double units;

    public CollectProbe(int count, double units) {
      this.count = Math.max(0, count);
      this.units = Math.max(0.0, units);
    }
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

  public boolean footprintHasFuel(Translation2d center, double cellM) {
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

  public void markCollectDepleted(Translation2d p, double cellM, double strength) {
    addDepletedMark(p, Math.max(0.10, cellM * 2.0), strength, DEPLETED_TTL_S, false);
  }

  public static final class Candidate {
    public final RepulsorSetpoint setpoint;
    public final Translation2d targetXY;
    public final double ourEtaS;
    public final double enemyEtaS;
    public final double allyEtaS;
    public final double congestion;
    public final double pressure;
    public final double score;

    public Candidate(
        RepulsorSetpoint sp,
        Translation2d xy,
        double ourEtaS,
        double enemyEtaS,
        double allyEtaS,
        double congestion,
        double pressure,
        double score) {
      this.setpoint = sp;
      this.targetXY = xy;
      this.ourEtaS = ourEtaS;
      this.enemyEtaS = enemyEtaS;
      this.allyEtaS = allyEtaS;
      this.congestion = congestion;
      this.pressure = pressure;
      this.score = score;
    }
  }

  public static final class DynamicObject {
    public final String id;
    public final String type;
    public final Translation2d pos;
    public final Translation2d vel;
    public final double ageS;

    public DynamicObject(
        String id, String type, Translation2d pos, Translation2d vel, double ageS) {
      this.id = id;
      this.type = type != null ? type : "unknown";
      this.pos = pos != null ? pos : new Translation2d();
      this.vel = vel != null ? vel : new Translation2d();
      this.ageS = Math.max(0.0, ageS);
    }
  }

  public static final class ResourceSpec {
    public final double radiusM;
    public final double unitValue;
    public final double sigmaM;

    public ResourceSpec(double radiusM, double unitValue, double sigmaM) {
      this.radiusM = Math.max(0.01, radiusM);
      this.unitValue = unitValue;
      this.sigmaM = Math.max(0.02, sigmaM);
    }
  }

  public static final class PointCandidate {
    public final Translation2d point;
    public final double ourEtaS;
    public final double value;
    public final double enemyPressure;
    public final double allyCongestion;
    public final double enemyIntent;
    public final double allyIntent;
    public final double score;
    public final Rotation2d rotation;

    public PointCandidate(
        Translation2d point,
        Rotation2d rot,
        double ourEtaS,
        double value,
        double enemyPressure,
        double allyCongestion,
        double enemyIntent,
        double allyIntent,
        double score) {
      this.point = point;
      this.ourEtaS = ourEtaS;
      this.value = value;
      this.enemyPressure = enemyPressure;
      this.allyCongestion = allyCongestion;
      this.enemyIntent = enemyIntent;
      this.allyIntent = allyIntent;
      this.score = score;
      this.rotation = rot;
    }
  }

  private static final class Track {
    Translation2d pos;
    Translation2d vel;
    double speedCap;
    double lastTs;

    Track(Translation2d p, Translation2d v, double cap, double t) {
      pos = p != null ? p : new Translation2d();
      vel = v != null ? v : new Translation2d();
      speedCap = cap;
      lastTs = t;
    }
  }

  private static final double MIN_DT = 0.02;
  private static final double MAX_MEAS_DT = 0.20;
  private static final double ETA_FLOOR = 0.05;
  private static final double DEFAULT_ENEMY_SPEED = 2.2;
  private static final double DEFAULT_ALLY_SPEED = 3.0;
  private static final double DEFAULT_OUR_SPEED = 3.5;
  private static final double ACC_LIMIT = 2.5;
  private static final double VEL_EMA = 0.35;
  private static final double POS_EMA = 0.15;
  private static final double SOFTMAX_TEMP = 1.35;
  private static final double RESERVATION_RADIUS = 0.85;
  private static final double KERNEL_SIGMA = 0.95;
  private static final double HYST_PERSIST_S = 0.8;
  private static final double HYST_BONUS = 0.22;
  private static final double ADV_GAIN = 1.1;
  private static final double DIST_COST = 0.10;
  private static final double PRESSURE_GAIN = 0.72;
  private static final double CONGEST_COST = 0.95;
  private static final double CAPACITY_GAIN = 0.35;
  private static final double HEADING_GAIN = 0.18;

  private static final double COLLECT_VALUE_GAIN = 10.10;
  private static final double COLLECT_ETA_COST = 1.05;
  private static final double COLLECT_ENEMY_PRESS_COST = 1.15;
  private static final double COLLECT_ALLY_CONGEST_COST = 0.95;
  private static final double COLLECT_ENEMY_INTENT_COST = 0.75;
  private static final double COLLECT_ALLY_INTENT_COST = 0.55;
  private static final double COLLECT_VALUE_SAT_K = 0.75;
  private static final double COLLECT_AGE_DECAY = 1.25;
  private static final double COLLECT_LOCAL_AVOID_R = 0.9;
  private static final double COLLECT_ACTIVITY_SIGMA = 1.05;
  private static final double COLLECT_ACTIVITY_ALLY_W = 0.80;
  private static final double COLLECT_ACTIVITY_ENEMY_W = 0.55;
  private static final double COLLECT_ACTIVITY_DYN_W = 0.60;
  private static final double COLLECT_REGION_SAMPLES_W = 2.70;
  private static final double COLLECT_CELL_M = 0.10;
  private static final double COLLECT_NEAR_BONUS = 0.85;
  private static final double COLLECT_NEAR_DECAY = 1.35;
  private static final double COLLECT_SPREAD_SCORE_R = 0.85;
  private static final double COLLECT_SPREAD_MIN = 0.30;
  private static final double COLLECT_SPREAD_MAX = 0.65;
  private static final double SHOOT_X_END_BAND_M = 12.5631260802;
  private static final double BAND_WIDTH_M = 2.167294751;
  private static final Interval<Double> X_LEFT_BAND =
      Interval.closed(SHOOT_X_END_BAND_M - BAND_WIDTH_M, SHOOT_X_END_BAND_M);
  private static final Interval<Double> X_RIGHT_BAND =
      Interval.closed(
          Constants.FIELD_LENGTH - SHOOT_X_END_BAND_M,
          Constants.FIELD_LENGTH - (SHOOT_X_END_BAND_M - BAND_WIDTH_M));

  private static final double COLLECT_CORE_R_MIN = 0.05;
  private static final double COLLECT_CORE_R_MAX = 0.12;
  private static final double COLLECT_SNAP_R_MIN = 0.35;
  private static final double COLLECT_SNAP_R_MAX = 0.60;
  private static final double COLLECT_MICRO_CENTROID_R_MIN = 0.18;
  private static final double COLLECT_MICRO_CENTROID_R_MAX = 0.35;
  private static final double COLLECT_JITTER_R_MIN = 0.06;
  private static final double COLLECT_JITTER_R_MAX = 0.14;
  private static final double COLLECT_HOLE_PENALTY = 2.35;
  private static final double COLLECT_EDGE_PENALTY = 0.85;
  private static final double COLLECT_NOFUEL_PENALTY = 3.00;

  private static final int COLLECT_GRID_TOPK = 420;
  private static final int COLLECT_RESOURCE_SEEDS_MAX = 64;
  private static final double COLLECT_CAND_GATE_R = 0.60;
  private static final double COLLECT_PEAK_GATE_R = 0.60;
  private static final double COLLECT_PEAK_SCORE_R = 0.85;

  private static final double COLLECT_COARSE_MIN_REGION_UNITS = 0.12;
  private static final double COLLECT_FINE_MIN_UNITS = 0.08;

  private static final double RESOURCE_SIGMA_ABS_MAX = 0.45;
  private static final double RESOURCE_SIGMA_REL_MAX = 1.25;
  private static final double RESOURCE_SIGMA_MIN = 0.06;

  private static final int COLLECT_CLUSTER_MAX = 72;
  private static final double COLLECT_CLUSTER_BIN_M = 0.14;
  private static final int COLLECT_GRID_FALLBACK_MAX = 1400;

  private static final double DEPLETED_TTL_S = 3.25;
  private static final double DEPLETED_DECAY = 1.15;
  private static final double DEPLETED_PEN_W = 1.75;
  private static final double DEPLETED_MARK_NEAR_M = 0.65;
  private static final double DEPLETED_MARK_EMPTY_UNITS = 0.07;

  private static final double RESOURCE_HARD_MAX_AGE_S = 0.95;

  private static final int COLLECT_SHORTLIST_ETA = 36;
  private static final int COLLECT_SHORTLIST_COARSE = 28;
  private static final int COLLECT_FINE_OFFSETS_GRID = 3;
  private static final double COLLECT_FINE_OFFSETS_SCALE = 0.65;

  private static final double COLLECT_COMMIT_MIN_S = 0.42;
  private static final double COLLECT_COMMIT_MAX_S = 0.92;
  private static final double COLLECT_SWITCH_BASE = 0.25;
  private static final double COLLECT_SWITCH_ETA_W = 0.05;

  private static final double COLLECT_PROGRESS_MIN_DROP_M = 0.25;
  private static final double COLLECT_PROGRESS_WINDOW_S = 0.70;

  private static final double COLLECT_ARRIVE_R = 0.75;
  private static final double COLLECT_ARRIVE_VERIFY_S = 0.28;
  private static final double COLLECT_FAIL_COOLDOWN_S = 2.10;

  private static final double EVIDENCE_R = 0.85;
  private static final double EVIDENCE_MIN_BASE = 0.05;
  private static final double EVIDENCE_MIN_MAX = 0.18;

  private static final double ACTIVITY_CAP = 1.05;

  private static final int DEPLETED_MARKS_MAX = 128;
  private static final int REGION_STATS_MAX = 512;

  private static final int PATH_SAMPLES = 7;
  private static final double PATH_COST_W = 0.60;

  private static final int ENEMY_REGIONS_MAX = 24;
  private static final double ENEMY_REGION_SIGMA = 0.85;

  private static final double RES_OVERLAP_R = 0.75;
  private static final double RES_OVERLAP_GAIN = 1.05;

  private static final int PEAK_FINDER_TOPN = 28;

  private double[] scratchLogits = new double[0];
  private double[] scratchLogits2 = new double[0];

  private volatile List<DynamicObject> lastDynRef = null;
  private volatile SpatialDyn lastDyn = null;
  private volatile int specsVersion = 0;
  private volatile int lastDynSpecsVersion = -1;
  private volatile SpatialDyn lastFootprintDyn = null;
  private final HashMap<Long, Boolean> footprintCache = new HashMap<>(512);

  private void ensureScratch(int n) {
    if (scratchLogits.length < n)
      scratchLogits = new double[Math.max(n, scratchLogits.length * 2 + 8)];
    if (scratchLogits2.length < n)
      scratchLogits2 = new double[Math.max(n, scratchLogits2.length * 2 + 8)];
  }

  private SpatialDyn cachedDyn() {
    List<DynamicObject> ref = dynamicObjects;
    int sv = specsVersion;
    SpatialDyn d = lastDyn;
    if (d != null && ref == lastDynRef && sv == lastDynSpecsVersion) return d;
    SpatialDyn nd = new SpatialDyn(ref, resourceSpecs, otherTypeWeights);
    lastDynRef = ref;
    lastDyn = nd;
    lastDynSpecsVersion = sv;
    return nd;
  }

  private final HashMap<Integer, Track> allyMap = new HashMap<>();
  private final HashMap<Integer, Track> enemyMap = new HashMap<>();

  private List<GameElement> worldElements = List.of();
  private Alliance ourAlliance =
      DriverStation.getAlliance().get() == edu.wpi.first.wpilibj.DriverStation.Alliance.Blue
          ? Alliance.kBlue
          : Alliance.kRed;

  private RepulsorSetpoint lastChosen = null;
  private double lastChosenTs = 0.0;

  private volatile List<DynamicObject> dynamicObjects = List.of();
  private final HashMap<String, ResourceSpec> resourceSpecs = new HashMap<>();
  private final HashMap<String, Double> otherTypeWeights = new HashMap<>();

  private static final class DepletedMark {
    final Translation2d p;
    double t;
    double s;
    double r;
    double ttl;
    final boolean ring;
    final double ringR0;
    final double ringR1;

    DepletedMark(Translation2d p, double t, double s, double r, double ttl) {
      this.p = p != null ? p : new Translation2d();
      this.t = t;
      this.s = Math.max(0.0, s);
      this.r = Math.max(0.05, r);
      this.ttl = Math.max(0.1, ttl);
      this.ring = false;
      this.ringR0 = 0.0;
      this.ringR1 = 0.0;
    }

    DepletedMark(Translation2d p, double t, double s, double r0, double r1, double ttl) {
      this.p = p != null ? p : new Translation2d();
      this.t = t;
      this.s = Math.max(0.0, s);
      this.r = Math.max(0.05, 0.5 * (Math.max(r0, r1) - Math.min(r0, r1)));
      this.ttl = Math.max(0.1, ttl);
      this.ring = true;
      this.ringR0 = Math.max(0.05, Math.min(r0, r1));
      this.ringR1 = Math.max(this.ringR0 + 0.05, Math.max(r0, r1));
    }
  }

  private final ArrayList<DepletedMark> depletedMarks = new ArrayList<>(256);

  private Translation2d lastReturnedCollect = null;
  private double lastReturnedCollectTs = 0.0;

  private Translation2d currentCollectTarget = null;
  private double currentCollectChosenTs = 0.0;
  private double currentCollectScore = -1e18;
  private double currentCollectUnits = 0.0;
  private double currentCollectEta = 0.0;

  private double collectProgressLastTs = 0.0;
  private double collectProgressLastDist = Double.POSITIVE_INFINITY;

  private double collectArrivalTs = -1.0;

  private Translation2d lastOurPosForCollect = null;
  private double lastOurCapForCollect = DEFAULT_OUR_SPEED;
  private int lastGoalUnitsForCollect = 1;
  private double lastCellMForCollect = COLLECT_CELL_M;

  private static final class RegionStat {
    int attempts;
    int successes;
    double lastAttemptTs;
  }

  private final HashMap<Long, RegionStat> regionStats = new HashMap<>(512);

  public void registerResourceSpec(String type, ResourceSpec spec) {
    if (type == null || type.isEmpty() || spec == null) return;
    resourceSpecs.put(type.toLowerCase(), spec);
    specsVersion++;
  }

  public void registerOtherTypeWeight(String type, double weight) {
    if (type == null || type.isEmpty()) return;
    otherTypeWeights.put(type.toLowerCase(), Math.max(0.0, weight));
    specsVersion++;
  }

  public void setDynamicObjects(List<DynamicObject> objs) {
    dynamicObjects = (objs != null) ? List.copyOf(objs) : List.of();
    lastDynRef = null;
    lastDyn = null;
  }

  public void setWorld(List<GameElement> elements, Alliance ours) {
    worldElements = elements != null ? elements : List.of();
    ourAlliance = ours;
  }

  public void updateAlly(int id, Translation2d pos, Translation2d velHint, Double speedCap) {
    if (pos == null) return;

    double now = Timer.getFPGATimestamp();
    Track t =
        allyMap.getOrDefault(
            id,
            new Track(
                pos, new Translation2d(), speedCap != null ? speedCap : DEFAULT_ALLY_SPEED, now));

    double dtRaw = now - t.lastTs;
    double dt = Math.max(MIN_DT, dtRaw);
    double dtMeas = Math.min(MAX_MEAS_DT, Math.max(MIN_DT, dtRaw));

    Translation2d vMeas = velHint != null ? velHint : pos.minus(t.pos).div(dtMeas);

    double cap = speedCap != null ? Math.max(0.1, speedCap) : DEFAULT_ALLY_SPEED;
    double vMag = Math.min(vMeas.getNorm(), cap);
    Translation2d vClamped =
        vMeas.getNorm() > 1e-6 ? vMeas.div(vMeas.getNorm()).times(vMag) : new Translation2d();

    double aVel = emaAlpha(VEL_EMA, dt);
    double aPos = emaAlpha(POS_EMA, dt);

    Translation2d vEma = lerpVec(t.vel, vClamped, aVel);
    t.vel = clampDeltaV(t.vel, vEma, ACC_LIMIT, dt);

    t.pos = lerpVec(t.pos, pos, aPos);
    t.speedCap = cap;
    t.lastTs = now;

    allyMap.put(id, t);
  }

  public void updateEnemy(int id, Translation2d pos, Translation2d velHint, Double speedCap) {
    if (pos == null) return;

    double now = Timer.getFPGATimestamp();
    Track t =
        enemyMap.getOrDefault(
            id,
            new Track(
                pos, new Translation2d(), speedCap != null ? speedCap : DEFAULT_ENEMY_SPEED, now));

    double dtRaw = now - t.lastTs;
    double dt = Math.max(MIN_DT, dtRaw);
    double dtMeas = Math.min(MAX_MEAS_DT, Math.max(MIN_DT, dtRaw));

    Translation2d vMeas = velHint != null ? velHint : pos.minus(t.pos).div(dtMeas);

    double cap = speedCap != null ? Math.max(0.1, speedCap) : DEFAULT_ENEMY_SPEED;
    double vMag = Math.min(vMeas.getNorm(), cap);
    Translation2d vClamped =
        vMeas.getNorm() > 1e-6 ? vMeas.div(vMeas.getNorm()).times(vMag) : new Translation2d();

    double aVel = emaAlpha(VEL_EMA, dt);
    double aPos = emaAlpha(POS_EMA, dt);

    Translation2d vEma = lerpVec(t.vel, vClamped, aVel);
    t.vel = clampDeltaV(t.vel, vEma, ACC_LIMIT, dt);

    t.pos = lerpVec(t.pos, pos, aPos);
    t.speedCap = cap;
    t.lastTs = now;

    enemyMap.put(id, t);
  }

  public void clearStale(double maxAgeS) {
    double now = Timer.getFPGATimestamp();
    allyMap.entrySet().removeIf(e -> now - e.getValue().lastTs > maxAgeS);
    enemyMap.entrySet().removeIf(e -> now - e.getValue().lastTs > maxAgeS);
  }

  public List<Candidate> rank(
      Translation2d ourPos, double ourSpeedCap, CategorySpec cat, int limit) {
    Objects.requireNonNull(ourPos);
    List<GameElement> elems = new ArrayList<>();
    for (GameElement e : worldElements) {
      if (e.getAlliance() == ourAlliance
          && e.getCategory() == cat
          && e.getRelatedPoint().isPresent()
          && !e.isAtCapacity()) {
        elems.add(e);
      }
    }
    if (elems.isEmpty()) return List.of();

    List<Translation2d> targets = new ArrayList<>();
    List<RepulsorSetpoint> sps = new ArrayList<>();
    for (GameElement e : elems) {
      Translation2d p =
          new Translation2d(e.getModel().getPosition().getX(), e.getModel().getPosition().getY());
      targets.add(p);
      sps.add(e.getRelatedPoint().get());
    }

    IntentAgg allyAgg = softIntentAgg(allyMap, targets);
    IntentAgg enemyAgg = softIntentAgg(enemyMap, targets);

    double now = Timer.getFPGATimestamp();
    List<Candidate> out = new ArrayList<>();
    double cap = ourSpeedCap > 0 ? ourSpeedCap : DEFAULT_OUR_SPEED;

    for (int i = 0; i < targets.size(); i++) {
      Translation2d t = targets.get(i);
      RepulsorSetpoint sp = sps.get(i);

      double ourEta = estimateTravelTime(ourPos, t, cap);
      double enemyEta = minEtaToTarget(enemyMap, t);
      double allyEta = minEtaToTarget(allyMap, t);

      double pressure = radialPressure(enemyMap, t, ourEta, enemyAgg.intent[i], enemyAgg.count);
      double congestion = radialCongestion(allyMap, t, ourEta, allyAgg.intent[i], allyAgg.count);
      double distBias = ourPos.getDistance(t) * DIST_COST;

      double capacityFrac = 0.0;
      GameElement e = elems.get(i);
      if (e.getMaxContained() > 0)
        capacityFrac =
            1.0
                - Math.min(
                    1.0, (double) e.getContainedCount() / Math.max(1.0, e.getMaxContained()));

      double heading = headingAffinity(ourPos, t, allyMap, enemyMap);

      double score =
          (enemyEta - ourEta) * ADV_GAIN
              - congestion * CONGEST_COST
              - pressure * PRESSURE_GAIN
              - distBias
              + capacityFrac * CAPACITY_GAIN
              + heading * HEADING_GAIN;

      if (lastChosen != null && sp.equals(lastChosen) && now - lastChosenTs < HYST_PERSIST_S)
        score += HYST_BONUS;

      out.add(new Candidate(sp, t, ourEta, enemyEta, allyEta, congestion, pressure, score));
    }

    out.sort(Comparator.comparingDouble((Candidate c) -> -c.score));
    if (!out.isEmpty()) {
      lastChosen = out.get(0).setpoint;
      lastChosenTs = now;
    }
    if (limit > 0 && out.size() > limit) return new ArrayList<>(out.subList(0, limit));
    return out;
  }

  private static void sortIdxByKey(double[] key, int[] idx) {
    quickSortIdx(key, idx, 0, idx.length - 1);
  }

  private static void quickSortIdx(double[] key, int[] idx, int lo, int hi) {
    while (lo < hi) {
      int i = lo;
      int j = hi;
      double pivot = key[idx[(lo + hi) >>> 1]];

      while (i <= j) {
        while (key[idx[i]] < pivot) i++;
        while (key[idx[j]] > pivot) j--;
        if (i <= j) {
          int tmp = idx[i];
          idx[i] = idx[j];
          idx[j] = tmp;
          i++;
          j--;
        }
      }

      if (j - lo < hi - i) {
        if (lo < j) quickSortIdx(key, idx, lo, j);
        lo = i;
      } else {
        if (i < hi) quickSortIdx(key, idx, i, hi);
        hi = j;
      }
    }
  }

  public List<RepulsorSetpoint> rankSetpoints(
      Translation2d ourPos, double ourSpeedCap, CategorySpec cat, int limit) {
    List<Candidate> c = rank(ourPos, ourSpeedCap, cat, limit);
    List<RepulsorSetpoint> out = new ArrayList<>();
    for (Candidate k : c) out.add(k.setpoint);
    return out;
  }

  public int resourceObservationCount() {
    SpatialDyn d = cachedDyn();
    if (d == null) return 0;
    return d.resources.size();
  }

  public Translation2d snapToCollectCentroid(Translation2d seed, double r, double minMass) {
    if (seed == null) return null;
    SpatialDyn dyn = cachedDyn();
    if (dyn == null) return seed;

    Translation2d c = dyn.centroidResourcesWithin(seed, Math.max(0.05, r), Math.max(0.0, minMass));
    if (c == null) return seed;

    Translation2d ourPos = lastOurPosForCollect != null ? lastOurPosForCollect : seed;
    double cap = lastOurCapForCollect > 0.0 ? lastOurCapForCollect : DEFAULT_OUR_SPEED;
    int goal = Math.max(1, lastGoalUnitsForCollect);
    double cellM = Math.max(0.10, lastCellMForCollect);

    CollectEval a = evalCollectPoint(ourPos, cap, seed, goal, cellM, dyn, null, null);
    CollectEval b = evalCollectPoint(ourPos, cap, c, goal, cellM, dyn, null, null);

    if (b.score > a.score + 1e-9) return c;
    return seed;
  }

  public Translation2d nearestCollectResource(Translation2d p, double maxDist) {
    if (p == null) return null;
    SpatialDyn d = cachedDyn();
    if (d == null) return null;
    return d.nearestResourceTo(p, Math.max(0.01, maxDist));
  }

  private Translation2d[] peakFinder(Translation2d[] gridPoints, SpatialDyn dyn, int topN) {
    if (gridPoints == null || gridPoints.length == 0 || dyn == null) return new Translation2d[0];

    int K = Math.max(1, topN);
    Translation2d[] bestP = new Translation2d[K];
    double[] bestV = new double[K];
    for (int i = 0; i < K; i++) bestV[i] = -1e18;

    for (int i = 0; i < gridPoints.length; i++) {
      Translation2d p = gridPoints[i];
      if (p == null) continue;

      Translation2d near = dyn.nearestResourceTo(p, COLLECT_PEAK_GATE_R);
      if (near == null) continue;

      double v = dyn.evidenceMassWithin(p, COLLECT_PEAK_SCORE_R);
      if (v <= 1e-9) continue;

      int worstI = 0;
      double worst = bestV[0];
      for (int k = 1; k < K; k++) {
        if (bestV[k] < worst) {
          worst = bestV[k];
          worstI = k;
        }
      }
      if (v > worst) {
        bestV[worstI] = v;
        bestP[worstI] = p;
      }
    }

    ArrayList<Translation2d> tmp = new ArrayList<>(K);
    for (Translation2d p : bestP) if (p != null) tmp.add(p);

    tmp = dedupPoints(tmp, adaptiveDupSkip(dyn));
    return tmp.toArray(new Translation2d[0]);
  }

  private static final IntakeFootprint COLLECT_INTAKE = IntakeFootprint.frontRect(0.7, 0.175, 0.35);
  private static final Translation2d[] COLLECT_FOOTPRINT_SAMPLES =
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
  private static final double COLLECT_KEEP_BONUS = 0.20;

  private Rotation2d currentCollectHeading = new Rotation2d();
  private Translation2d currentCollectTouch = null;

  private static Rotation2d face(Translation2d from, Translation2d to, Rotation2d fallback) {
    Translation2d d = to.minus(from);
    if (d.getNorm() < 1e-9
        || (d.getMeasureX().baseUnitMagnitude() == 0 && d.getMeasureY().baseUnitMagnitude() == 0))
      return fallback;
    return d.getAngle();
  }

  private Translation2d resolveCollectTouch(
      SpatialDyn dyn,
      Translation2d center,
      Rotation2d heading,
      double rCore,
      double rSnap,
      double rCentroid) {
    if (center == null) return null;
    Translation2d front =
        center.plus(
            COLLECT_INTAKE.supportPointRobotFrame(new Translation2d(1.0, 0.0)).rotateBy(heading));
    return enforceHardStopOnFuel(dyn, front, rCore, rSnap, rCentroid, 0.10);
  }

  private static final class FootprintEval {
    int maxCount;
    double sumUnits;
    double avgEvidence;
    boolean hasEvidence;
  }

  private FootprintEval evalFootprint(
      SpatialDyn dyn, Translation2d center, Rotation2d heading, double rCore) {
    FootprintEval e = new FootprintEval();
    if (dyn == null || center == null) return e;
    for (Translation2d sample : COLLECT_FOOTPRINT_SAMPLES) {
      Translation2d field = center.plus(sample.rotateBy(heading));
      int c = dyn.countResourcesWithin(field, Math.max(0.04, rCore));
      double u = dyn.valueInSquare(field, Math.max(0.08, rCore));
      if (c > e.maxCount) e.maxCount = c;
      e.sumUnits += u;
    }
    return e;
  }

  private void fillFootprintEvidence(
      SpatialDyn dyn, Translation2d center, Rotation2d heading, FootprintEval e) {
    if (e == null || e.hasEvidence || dyn == null || center == null) return;
    double sumEv = 0.0;
    int n = 0;
    for (Translation2d sample : COLLECT_FOOTPRINT_SAMPLES) {
      Translation2d field = center.plus(sample.rotateBy(heading));
      double ev = dyn.evidenceMassWithin(field, EVIDENCE_R);
      sumEv += ev;
      n++;
    }
    e.avgEvidence = (n > 0) ? (sumEv / n) : 0.0;
    e.hasEvidence = true;
  }

  private boolean footprintOk(
      SpatialDyn dyn, Translation2d center, Rotation2d heading, double rCore, double minUnits) {
    FootprintEval fp = evalFootprint(dyn, center, heading, rCore);
    if (fp.maxCount < 1 || fp.sumUnits < minUnits) return false;
    fillFootprintEvidence(dyn, center, heading, fp);
    return fp.avgEvidence >= minUnits * 0.85;
  }

  private static final class HeadingPick {
    final Translation2d center;
    final Rotation2d heading;
    final FootprintEval eval;

    HeadingPick(Translation2d center, Rotation2d heading, FootprintEval eval) {
      this.center = center;
      this.heading = heading;
      this.eval = eval;
    }
  }

  private HeadingPick bestHeadingForFootprint(
      SpatialDyn dyn,
      Translation2d desiredCenter,
      Translation2d fuelTouch,
      Rotation2d baseHeading,
      double rCore,
      double minUnits) {
    if (dyn == null || desiredCenter == null || fuelTouch == null) return null;
    double[] deg = new double[] {-60, -40, -20, 0, 20, 40, 60};
    HeadingPick best = null;
    for (double d : deg) {
      Rotation2d h = baseHeading.plus(Rotation2d.fromDegrees(d));
      Translation2d p =
          COLLECT_INTAKE.snapCenterSoFootprintTouchesPoint(desiredCenter, h, fuelTouch);
      FootprintEval fp = evalFootprint(dyn, p, h, rCore);
      if (fp.maxCount < 1 || fp.sumUnits < minUnits) continue;
      if (best == null
          || fp.maxCount > best.eval.maxCount
          || (fp.maxCount == best.eval.maxCount && fp.sumUnits > best.eval.sumUnits)
          || (fp.maxCount == best.eval.maxCount
              && Math.abs(fp.sumUnits - best.eval.sumUnits) <= 1e-6
              && compareFootprintEvidence(dyn, p, h, fp, best))) {
        best = new HeadingPick(p, h, fp);
      }
    }
    if (best != null) {
      fillFootprintEvidence(dyn, best.center, best.heading, best.eval);
    }
    return best;
  }

  private boolean compareFootprintEvidence(
      SpatialDyn dyn, Translation2d p, Rotation2d h, FootprintEval fp, HeadingPick best) {
    if (best == null) return true;
    fillFootprintEvidence(dyn, p, h, fp);
    fillFootprintEvidence(dyn, best.center, best.heading, best.eval);
    return fp.avgEvidence > best.eval.avgEvidence;
  }

  private static long footprintKey(Translation2d center, double cellM) {
    long kx = Math.round(center.getX() * 1000.0);
    long ky = Math.round(center.getY() * 1000.0);
    long kc = Math.round(cellM * 1000.0);
    long key = kx * 0x9E3779B97F4A7C15L;
    key ^= Long.rotateLeft(ky * 0xC2B2AE3D27D4EB4FL, 1);
    key ^= Long.rotateLeft(kc * 0x165667B19E3779F9L, 2);
    return key;
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
    final double wallClearMin = robotWallMargin + 0.35;
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
          edu.wpi.first.wpilibj.DriverStation.Alliance alliance =
              edu.wpi.first.wpilibj.DriverStation.getAlliance().get();

          if (alliance == edu.wpi.first.wpilibj.DriverStation.Alliance.Blue) {
            double x = pt.getX();
            double y = pt.getY();
            if (x > SHOOT_X_END_BAND_M) return true;
          } else {
            double x = pt.getX();
            double y = pt.getY();
            if (x < (Constants.FIELD_LENGTH - SHOOT_X_END_BAND_M + BAND_WIDTH_M)) return true;
          }

          double x = pt.getX();
          double y = pt.getY();
          return X_LEFT_BAND.within(x)
              || X_RIGHT_BAND.within(x)
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

    FuelRegions enemyRegions = buildFuelRegions(dyn, ENEMY_REGIONS_MAX);
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

    Translation2d bestP = null;
    Translation2d bestTouch = null;
    Rotation2d bestHeading = new Rotation2d();
    CollectEval bestE = null;

    double[] topScore = new double[Math.min(10, Math.max(1, maxCheck))];
    double[] topEta = new double[topScore.length];
    double[] topUnits = new double[topScore.length];
    double[] topRegion = new double[topScore.length];
    double[] topDep = new double[topScore.length];
    double[] topAvoid = new double[topScore.length];
    double[] topEP = new double[topScore.length];
    double[] topAC = new double[topScore.length];
    double[] topAct = new double[topScore.length];
    double[] topEv = new double[topScore.length];
    int topN = 0;

    double now0 = Timer.getFPGATimestamp();

    for (int si = 0; si < maxCheck; si++) {
      int idx = shortlist.get(si);
      Translation2d center = seeds[idx];
      if (center == null || inShootBand.test(center)) continue;

      double regionUnitsNear = dyn.valueInSquare(center, nearHalf);
      int cNear0 = dyn.countResourcesWithin(center, nearR);
      if (regionUnitsNear < Math.min(COLLECT_COARSE_MIN_REGION_UNITS, minUnits * 0.90)
          || cNear0 < 1) continue;

      double step = Math.max(0.05, cellM * COLLECT_FINE_OFFSETS_SCALE);
      int g = Math.max(2, COLLECT_FINE_OFFSETS_GRID);
      double span = step * (g - 1);
      double start = -0.5 * span;

      Translation2d localBest = null;
      Translation2d localBestTouch = null;
      Rotation2d localBestHeading = new Rotation2d();
      CollectEval localBestE = null;
      boolean localHasCore = false;

      for (int ix = 0; ix < g; ix++) {
        double ox = start + ix * step;
        for (int iy = 0; iy < g; iy++) {
          double oy = start + iy * step;
          Translation2d p0 = new Translation2d(center.getX() + ox, center.getY() + oy);

          if (inShootBand.test(p0)) continue;

          double fuelNear = dyn.valueInSquare(p0, nearHalf);
          int cNear = dyn.countResourcesWithin(p0, nearR);
          if (fuelNear < minHardUnits || cNear < 1) continue;

          Translation2d fuelTouch = enforceHardStopOnFuel(dyn, p0, rCore, rSnap, rCentroid, 0.10);
          if (fuelTouch == null || inShootBand.test(fuelTouch)) continue;

          double fuelOn = dyn.valueInSquare(fuelTouch, onHalf);
          int cOn = dyn.countResourcesWithin(fuelTouch, onR);
          double evHard = dyn.evidenceMassWithin(fuelTouch, EVIDENCE_R);
          if (fuelOn < minHardUnits || cOn < 1 || evHard < minEv) continue;

          int cCore = dyn.countResourcesWithin(fuelTouch, rCore);
          if (cCore < 1) continue;

          Rotation2d baseHeading = face(p0, fuelTouch, new Rotation2d());
          HeadingPick pick =
              bestHeadingForFootprint(dyn, p0, fuelTouch, baseHeading, rCore, footprintMinUnits);
          if (pick == null || inShootBand.test(pick.center)) continue;
          Rotation2d heading = pick.heading;
          Translation2d p = pick.center;

          CollectEval e =
              evalCollectPoint(ourPos, cap, p, goal, cellM, dyn, enemyIntent, allyIntent);

          e.units = fuelOn;
          e.count = cOn;
          e.evidence = evHard;
          e.coreCount = cCore;
          Translation2d nn = dyn.nearestResourceTo(fuelTouch, rCore);
          e.coreDist = nn != null ? nn.getDistance(fuelTouch) : 1e9;

          e.regionUnits = regionUnitsNear;
          e.banditBonus = regionBanditBonus(dyn, fuelTouch, now0);
          e.score += e.banditBonus;
          e.score -= wallPenalty.applyAsDouble(fuelTouch);
          e.score -= wallPenalty.applyAsDouble(p) * 0.85;
          e.score -= depletedPenaltySoft(fuelTouch) * DEPLETED_PEN_W * 1.35;
          if (e.coreCount > 1) e.score += Math.min(0.6, 0.2 * (e.coreCount - 1));
          double coreDistNorm = Math.min(1.0, e.coreDist / Math.max(0.05, rCore));
          e.score -= 0.85 * coreDistNorm;
          FootprintEval fp = pick.eval;
          double fpBonus = Math.min(2.0, 0.50 * fp.maxCount + 0.85 * fp.sumUnits);
          e.score += fpBonus;
          if (fp.avgEvidence < minEv * 0.90) e.score -= 1.35;

          if (e.units < minUnits * 0.55 || e.count < Math.max(1, minCount - 1)) e.score -= 2.75;
          if (e.evidence < minEv) e.score -= 2.25;

          if (!localHasCore) {
            localHasCore = true;
            localBestE = e;
            localBest = p;
            localBestTouch = fuelTouch;
            localBestHeading = heading;
          } else {
            if (localBestE == null
                || e.score > localBestE.score + 1e-9
                || (Math.abs(e.score - localBestE.score) <= 0.03
                    && (e.count > localBestE.count + 1
                        || (e.count == localBestE.count && e.depleted < localBestE.depleted - 0.04)
                        || (e.count == localBestE.count && e.units > localBestE.units + 0.06)))) {
              localBestE = e;
              localBest = p;
              localBestTouch = fuelTouch;
              localBestHeading = heading;
            }
          }
        }
      }

      if (!localHasCore) {
        Translation2d anchor = enforceHardStopOnFuel(dyn, center, rCore, rSnap, rCentroid, 0.10);
        if (anchor != null && !inShootBand.test(anchor)) {
          double micro = Math.max(0.03, 0.75 * rCore);
          Translation2d bestMicroP = null;
          Translation2d bestMicroTouch = null;
          Rotation2d bestMicroHeading = new Rotation2d();
          CollectEval bestMicroE = null;

          for (int ix = -1; ix <= 1; ix++) {
            for (int iy = -1; iy <= 1; iy++) {
              Translation2d p0 =
                  new Translation2d(anchor.getX() + ix * micro, anchor.getY() + iy * micro);
              if (inShootBand.test(p0)) continue;

              Translation2d fuelTouch =
                  enforceHardStopOnFuel(dyn, p0, rCore, rSnap, rCentroid, 0.08);
              if (fuelTouch == null || inShootBand.test(fuelTouch)) continue;

              int cCore = dyn.countResourcesWithin(fuelTouch, rCore);
              if (cCore < 1) continue;

              double fuelOn = dyn.valueInSquare(fuelTouch, onHalf);
              int cOn = dyn.countResourcesWithin(fuelTouch, onR);
              double evHard = dyn.evidenceMassWithin(fuelTouch, EVIDENCE_R);
              if (fuelOn < minHardUnits || cOn < 1 || evHard < minEv) continue;

              Rotation2d baseHeading = face(p0, fuelTouch, new Rotation2d());
              HeadingPick pick =
                  bestHeadingForFootprint(
                      dyn, p0, fuelTouch, baseHeading, rCore, footprintMinUnits);
              if (pick == null || inShootBand.test(pick.center)) continue;
              Rotation2d heading = pick.heading;
              Translation2d p = pick.center;

              CollectEval e =
                  evalCollectPoint(ourPos, cap, p, goal, cellM, dyn, enemyIntent, allyIntent);

              e.units = fuelOn;
              e.count = cOn;
              e.evidence = evHard;
              e.coreCount = cCore;
              Translation2d nn = dyn.nearestResourceTo(fuelTouch, rCore);
              e.coreDist = nn != null ? nn.getDistance(fuelTouch) : 1e9;

              e.regionUnits = regionUnitsNear;
              e.banditBonus = regionBanditBonus(dyn, fuelTouch, now0);
              e.score += e.banditBonus;
              e.score -= wallPenalty.applyAsDouble(fuelTouch);
              e.score -= wallPenalty.applyAsDouble(p) * 0.85;
              e.score -= depletedPenaltySoft(fuelTouch) * DEPLETED_PEN_W * 1.35;
              if (e.coreCount > 1) e.score += Math.min(0.6, 0.2 * (e.coreCount - 1));
              double coreDistNorm = Math.min(1.0, e.coreDist / Math.max(0.05, rCore));
              e.score -= 0.85 * coreDistNorm;
              FootprintEval fp = pick.eval;
              double fpBonus = Math.min(2.0, 0.50 * fp.maxCount + 0.85 * fp.sumUnits);
              e.score += fpBonus;
              if (fp.avgEvidence < minEv * 0.90) e.score -= 1.35;

              if (e.units < minUnits * 0.55 || e.count < Math.max(1, minCount - 1)) e.score -= 2.75;
              if (e.evidence < minEv) e.score -= 2.25;

              if (bestMicroE == null || e.score > bestMicroE.score + 1e-9) {
                bestMicroE = e;
                bestMicroP = p;
                bestMicroTouch = fuelTouch;
                bestMicroHeading = heading;
              }
            }
          }

          if (bestMicroP != null && bestMicroE != null && !inShootBand.test(bestMicroP)) {
            localBest = bestMicroP;
            localBestE = bestMicroE;
            localBestTouch = bestMicroTouch;
            localBestHeading = bestMicroHeading;
            localHasCore = true;
          }
        }
      }

      if (!localHasCore
          || localBest == null
          || localBestE == null
          || localBestTouch == null
          || inShootBand.test(localBest)
          || inShootBand.test(localBestTouch)) continue;

      if (bestE == null || localBestE.score > bestE.score + 1e-9) {
        bestE = localBestE;
        bestP = localBest;
        bestTouch = localBestTouch;
        bestHeading = localBestHeading;
      }

      if (topN < topScore.length) {
        topScore[topN] = localBestE.score;
        topEta[topN] = localBestE.eta;
        topUnits[topN] = localBestE.units;
        topRegion[topN] = localBestE.regionUnits;
        topDep[topN] = localBestE.depleted;
        topAvoid[topN] = localBestE.localAvoid;
        topEP[topN] = localBestE.enemyPressure;
        topAC[topN] = localBestE.allyCongestion;
        topAct[topN] = localBestE.activity;
        topEv[topN] = localBestE.evidence;
        topN++;
      } else {
        int worstI = 0;
        double worstS = topScore[0];
        for (int t = 1; t < topN; t++) {
          if (topScore[t] < worstS) {
            worstS = topScore[t];
            worstI = t;
          }
        }
        if (localBestE.score > worstS) {
          topScore[worstI] = localBestE.score;
          topEta[worstI] = localBestE.eta;
          topUnits[worstI] = localBestE.units;
          topRegion[worstI] = localBestE.regionUnits;
          topDep[worstI] = localBestE.depleted;
          topAvoid[worstI] = localBestE.localAvoid;
          topEP[worstI] = localBestE.enemyPressure;
          topAC[worstI] = localBestE.allyCongestion;
          topAct[worstI] = localBestE.activity;
          topEv[worstI] = localBestE.evidence;
        }
      }
    }

    if (bestP == null || bestE == null || bestTouch == null || inShootBand.test(bestP)) {
      // Logger.recordOutput("Repulsor/Collect/NoCandidate", 1.0);
      return null;
    }

    Translation2d chosenPt = bestP;
    Translation2d chosenTouch = bestTouch;
    Rotation2d chosenHeading = bestHeading;
    CollectEval chosenE = bestE;

    double now = Timer.getFPGATimestamp();

    if (currentCollectTarget != null && inShootBand.test(currentCollectTarget)) {
      addDepletedMark(currentCollectTarget, 0.65, 1.25, DEPLETED_TTL_S, false);
      addDepletedRing(currentCollectTarget, 0.35, 0.95, 0.80, DEPLETED_TTL_S);
      recordRegionAttempt(dyn, currentCollectTarget, now, false);
      currentCollectTarget = null;
      currentCollectTouch = null;
      collectArrivalTs = -1.0;
    }

    boolean escape =
        currentCollectTarget != null
            && (currentCollectTarget.getDistance(chosenPt) < 1e-6
                ? false
                : shouldEscapeCurrentCollect(ourPos, dyn, totalEv, minUnits, minCount, cellM));

    if (currentCollectTarget != null
        && !footprintOk(
            dyn, currentCollectTarget, currentCollectHeading, rCore, footprintMinUnits)) {
      addDepletedMark(currentCollectTarget, 0.70, 1.85, DEPLETED_TTL_S, false);
      addDepletedRing(currentCollectTarget, 0.35, 0.95, 0.80, DEPLETED_TTL_S);
      recordRegionAttempt(dyn, currentCollectTarget, now, false);
      currentCollectTarget = null;
      currentCollectTouch = null;
      collectArrivalTs = -1.0;
      escape = true;
      // Logger.recordOutput("Repulsor/Collect/ChosenOffFootprintImmediate", 1.0);
    }

    double commitS = collectCommitWindow(currentCollectEta);
    double switchMargin =
        (COLLECT_SWITCH_BASE + COLLECT_SWITCH_ETA_W * Math.max(0.0, currentCollectEta)) * 1.2
            + 0.05;

    if (!escape && currentCollectTarget != null) {
      double age = now - currentCollectChosenTs;

      Translation2d curTouch =
          currentCollectTouch != null
              ? currentCollectTouch
              : resolveCollectTouch(
                  dyn, currentCollectTarget, currentCollectHeading, rCore, rSnap, rCentroid);

      if (curTouch == null
          || inShootBand.test(curTouch)
          || inShootBand.test(currentCollectTarget)) {
        recordRegionAttempt(dyn, currentCollectTarget, now, false);
        currentCollectTarget = null;
        currentCollectTouch = null;
        collectArrivalTs = -1.0;
        // Logger.recordOutput("Repulsor/Collect/ChosenOffFuelCore", 1.0);
        return null;
      }

      CollectEval curE =
          evalCollectPoint(
              ourPos, cap, currentCollectTarget, goal, cellM, dyn, enemyIntent, allyIntent);
      curE.banditBonus = regionBanditBonus(dyn, curTouch, now);
      curE.score += curE.banditBonus;
      curE.score += COLLECT_KEEP_BONUS;

      double curOnUnits = dyn.valueInSquare(curTouch, onHalf);
      int curOnCount = dyn.countResourcesWithin(curTouch, onR);
      double curEv = dyn.evidenceMassWithin(curTouch, EVIDENCE_R);
      int curCore = dyn.countResourcesWithin(curTouch, rCore);
      Translation2d curNN = dyn.nearestResourceTo(curTouch, rCore);

      curE.units = curOnUnits;
      curE.count = curOnCount;
      curE.evidence = curEv;
      curE.coreCount = curCore;
      curE.coreDist = curNN != null ? curNN.getDistance(curTouch) : 1e9;

      currentCollectScore = curE.score;
      currentCollectUnits = curE.units;
      currentCollectEta = curE.eta;

      if (age >= 0.0 && age <= commitS) {
        if (chosenE.score <= currentCollectScore + switchMargin) {
          chosenPt = currentCollectTarget;
          chosenE = curE;
          chosenTouch = curTouch;
          chosenHeading = currentCollectHeading;
        } else {
          if (inShootBand.test(chosenPt) || inShootBand.test(chosenTouch)) {
            chosenPt = currentCollectTarget;
            chosenE = curE;
            chosenTouch = curTouch;
            chosenHeading = currentCollectHeading;
          } else {
            recordRegionAttempt(dyn, currentCollectTarget, now, false);
            currentCollectTarget = chosenPt;
            currentCollectTouch = chosenTouch;
            currentCollectHeading = chosenHeading;
            currentCollectChosenTs = now;
            currentCollectScore = chosenE.score;
            currentCollectUnits = chosenE.units;
            currentCollectEta = chosenE.eta;
            recordRegionAttempt(dyn, currentCollectTarget, now, false);
          }
        }
      } else {
        if (chosenE.score > currentCollectScore + switchMargin
            && !inShootBand.test(chosenPt)
            && !inShootBand.test(chosenTouch)) {
          recordRegionAttempt(dyn, currentCollectTarget, now, false);
          currentCollectTarget = chosenPt;
          currentCollectTouch = chosenTouch;
          currentCollectHeading = chosenHeading;
          currentCollectChosenTs = now;
          currentCollectScore = chosenE.score;
          currentCollectUnits = chosenE.units;
          currentCollectEta = chosenE.eta;
          recordRegionAttempt(dyn, currentCollectTarget, now, false);
        } else {
          chosenPt = currentCollectTarget;
          chosenE = curE;
          chosenTouch = curTouch;
          chosenHeading = currentCollectHeading;
        }
      }
    } else {
      if (inShootBand.test(chosenPt) || inShootBand.test(chosenTouch)) {
        // Logger.recordOutput("Repulsor/Collect/NoCandidate", 1.0);
        return null;
      }

      if (currentCollectTarget == null
          || chosenPt.getDistance(currentCollectTarget) > 0.05
          || escape) {
        if (escape && currentCollectTarget != null) {
          recordRegionAttempt(dyn, currentCollectTarget, now, false);
        }
        currentCollectTarget = chosenPt;
        currentCollectTouch = chosenTouch;
        currentCollectHeading = chosenHeading;
        currentCollectChosenTs = now;
        currentCollectScore = chosenE.score;
        currentCollectUnits = chosenE.units;
        currentCollectEta = chosenE.eta;
        recordRegionAttempt(dyn, currentCollectTarget, now, false);
      } else {
        currentCollectTarget = chosenPt;
        currentCollectTouch = chosenTouch;
        currentCollectHeading = chosenHeading;
        currentCollectScore = chosenE.score;
        currentCollectUnits = chosenE.units;
        currentCollectEta = chosenE.eta;
      }
    }

    if (currentCollectTarget != null) {
      if (inShootBand.test(currentCollectTarget)) {
        addDepletedMark(currentCollectTarget, 0.65, 1.25, DEPLETED_TTL_S, false);
        addDepletedRing(currentCollectTarget, 0.35, 0.95, 0.80, DEPLETED_TTL_S);
        recordRegionAttempt(dyn, currentCollectTarget, now, false);
        currentCollectTarget = null;
        currentCollectTouch = null;
        collectArrivalTs = -1.0;
        // Logger.recordOutput("Repulsor/Collect/ChosenInShootBand", 1.0);
        return null;
      }

      if (currentCollectTouch == null) {
        currentCollectTouch =
            resolveCollectTouch(
                dyn, currentCollectTarget, currentCollectHeading, rCore, rSnap, rCentroid);
      }

      if (!footprintOk(
          dyn, currentCollectTarget, currentCollectHeading, rCore, footprintMinUnits)) {
        addDepletedMark(currentCollectTarget, 0.70, 1.85, DEPLETED_TTL_S, false);
        addDepletedRing(currentCollectTarget, 0.35, 0.95, 0.80, DEPLETED_TTL_S);
        recordRegionAttempt(dyn, currentCollectTarget, now, false);
        currentCollectTarget = null;
        currentCollectTouch = null;
        collectArrivalTs = -1.0;
        // Logger.recordOutput("Repulsor/Collect/ChosenOffFootprint", 1.0);
        return null;
      }

      Translation2d anchoredTouch =
          currentCollectTouch != null
              ? enforceHardStopOnFuel(dyn, currentCollectTouch, rCore, rSnap, rCentroid, 0.10)
              : null;

      if (anchoredTouch == null || inShootBand.test(anchoredTouch)) {
        addDepletedMark(currentCollectTarget, 0.65, 1.25, DEPLETED_TTL_S, false);
        addDepletedRing(currentCollectTarget, 0.35, 0.95, 0.80, DEPLETED_TTL_S);
        recordRegionAttempt(dyn, currentCollectTarget, now, false);
        currentCollectTarget = null;
        currentCollectTouch = null;
        collectArrivalTs = -1.0;
        // Logger.recordOutput("Repulsor/Collect/ChosenOffFuelCore", 1.0);
        return null;
      }

      currentCollectTouch = anchoredTouch;
      currentCollectHeading =
          face(currentCollectTarget, currentCollectTouch, currentCollectHeading);
      currentCollectTarget =
          COLLECT_INTAKE.snapCenterSoFootprintTouchesPoint(
              currentCollectTarget, currentCollectHeading, currentCollectTouch);

      if (inShootBand.test(currentCollectTarget)) {
        addDepletedMark(currentCollectTarget, 0.65, 1.25, DEPLETED_TTL_S, false);
        addDepletedRing(currentCollectTarget, 0.35, 0.95, 0.80, DEPLETED_TTL_S);
        recordRegionAttempt(dyn, currentCollectTarget, now, false);
        currentCollectTarget = null;
        currentCollectTouch = null;
        collectArrivalTs = -1.0;
        // Logger.recordOutput("Repulsor/Collect/ChosenInShootBandPostSnap", 1.0);
        return null;
      }
    }

    boolean arrived =
        currentCollectTarget != null
            && ourPos.getDistance(currentCollectTarget) <= COLLECT_ARRIVE_R;

    if (arrived) {
      if (collectArrivalTs < 0.0) collectArrivalTs = now;
      double held = now - collectArrivalTs;
      if (held >= COLLECT_ARRIVE_VERIFY_S) {
        Translation2d touch = currentCollectTouch;
        if (touch == null) {
          touch =
              resolveCollectTouch(
                  dyn, currentCollectTarget, currentCollectHeading, rCore, rSnap, rCentroid);
          currentCollectTouch = touch;
        }

        Translation2d nn = touch != null ? dyn.nearestResourceTo(touch, rCore) : null;
        int cc = touch != null ? dyn.countResourcesWithin(touch, rCore) : 0;

        if (touch == null
            || nn == null
            || cc < 1
            || inShootBand.test(touch)
            || inShootBand.test(currentCollectTarget)) {
          addDepletedMark(currentCollectTarget, 0.70, 1.85, DEPLETED_TTL_S, false);
          addDepletedMark(currentCollectTarget, 0.95, 1.10, COLLECT_FAIL_COOLDOWN_S, false);
          addDepletedRing(currentCollectTarget, 0.35, 1.05, 0.95, COLLECT_FAIL_COOLDOWN_S);
          recordRegionAttempt(dyn, currentCollectTarget, now, false);
          currentCollectTarget = null;
          currentCollectTouch = null;
          currentCollectScore = -1e18;
          currentCollectUnits = 0.0;
          currentCollectEta = 0.0;
          collectArrivalTs = -1.0;
          lastReturnedCollect = null;
          lastReturnedCollectTs = 0.0;
          // Logger.recordOutput("Repulsor/Collect/ArrivalFailCore", 1.0);
          return null;
        }

        CollectProbe pr = probeCollect(touch, 0.65);
        double needU = Math.max(0.02, minUnits * 0.80);
        int needC = Math.max(1, minCount);
        if (pr.units < needU || pr.count < needC) {
          addDepletedMark(currentCollectTarget, 0.70, 1.85, DEPLETED_TTL_S, false);
          addDepletedMark(currentCollectTarget, 0.95, 1.10, COLLECT_FAIL_COOLDOWN_S, false);
          addDepletedRing(currentCollectTarget, 0.35, 1.05, 0.95, COLLECT_FAIL_COOLDOWN_S);
          recordRegionAttempt(dyn, currentCollectTarget, now, false);
          currentCollectTarget = null;
          currentCollectTouch = null;
          currentCollectScore = -1e18;
          currentCollectUnits = 0.0;
          currentCollectEta = 0.0;
          collectArrivalTs = -1.0;
          lastReturnedCollect = null;
          lastReturnedCollectTs = 0.0;
          // Logger.recordOutput("Repulsor/Collect/ArrivalFail", 1.0);
          return null;
        }
      }
    } else {
      collectArrivalTs = -1.0;
    }

    if (currentCollectTarget == null || inShootBand.test(currentCollectTarget)) return null;

    if (currentCollectTouch == null) {
      currentCollectTouch =
          resolveCollectTouch(
              dyn, currentCollectTarget, currentCollectHeading, rCore, rSnap, rCentroid);
    }
    if (currentCollectTouch == null || inShootBand.test(currentCollectTouch)) {
      addDepletedMark(currentCollectTarget, 0.65, 1.25, DEPLETED_TTL_S, false);
      addDepletedRing(currentCollectTarget, 0.35, 0.95, 0.80, DEPLETED_TTL_S);
      recordRegionAttempt(dyn, currentCollectTarget, now, false);
      currentCollectTarget = null;
      collectArrivalTs = -1.0;
      // Logger.recordOutput("Repulsor/Collect/ChosenOffFuel", 1.0);
      return null;
    }

    double finalOnUnits = dyn.valueInSquare(currentCollectTouch, onHalf);
    int finalOnCount = dyn.countResourcesWithin(currentCollectTouch, onR);
    double finalEv = dyn.evidenceMassWithin(currentCollectTouch, EVIDENCE_R);

    if (finalOnUnits < minHardUnits || finalOnCount < 1 || finalEv < minEv) {
      Translation2d anchoredTouch =
          enforceHardStopOnFuel(dyn, currentCollectTouch, rCore, rSnap, rCentroid, 0.10);
      if (anchoredTouch != null && !inShootBand.test(anchoredTouch)) {
        currentCollectTouch = anchoredTouch;
        currentCollectHeading =
            face(currentCollectTarget, currentCollectTouch, currentCollectHeading);
        currentCollectTarget =
            COLLECT_INTAKE.snapCenterSoFootprintTouchesPoint(
                currentCollectTarget, currentCollectHeading, currentCollectTouch);
        if (inShootBand.test(currentCollectTarget)) {
          addDepletedMark(currentCollectTarget, 0.65, 1.25, DEPLETED_TTL_S, false);
          addDepletedRing(currentCollectTarget, 0.35, 0.95, 0.80, DEPLETED_TTL_S);
          recordRegionAttempt(dyn, currentCollectTarget, now, false);
          currentCollectTarget = null;
          currentCollectTouch = null;
          collectArrivalTs = -1.0;
          // Logger.recordOutput("Repulsor/Collect/ChosenInShootBandPostReanchor", 1.0);
          return null;
        }
        finalOnUnits = dyn.valueInSquare(currentCollectTouch, onHalf);
        finalOnCount = dyn.countResourcesWithin(currentCollectTouch, onR);
        finalEv = dyn.evidenceMassWithin(currentCollectTouch, EVIDENCE_R);
      } else {
        addDepletedMark(currentCollectTarget, 0.65, 1.25, DEPLETED_TTL_S, false);
        addDepletedRing(currentCollectTarget, 0.35, 0.95, 0.80, DEPLETED_TTL_S);
        recordRegionAttempt(dyn, currentCollectTarget, now, false);
        currentCollectTarget = null;
        currentCollectTouch = null;
        collectArrivalTs = -1.0;
        // Logger.recordOutput("Repulsor/Collect/ChosenOffFuel", 1.0);
        return null;
      }
    }

    Translation2d nnCore = dyn.nearestResourceTo(currentCollectTouch, rCore);
    int cCoreFinal = dyn.countResourcesWithin(currentCollectTouch, rCore);
    if (nnCore == null || cCoreFinal < 1) {
      addDepletedMark(currentCollectTarget, 0.65, 1.25, DEPLETED_TTL_S, false);
      addDepletedRing(currentCollectTarget, 0.35, 0.95, 0.80, DEPLETED_TTL_S);
      recordRegionAttempt(dyn, currentCollectTarget, now, false);
      currentCollectTarget = null;
      currentCollectTouch = null;
      collectArrivalTs = -1.0;
      // Logger.recordOutput("Repulsor/Collect/ChosenOffFuelCoreFinal", 1.0);
      return null;
    }

    CollectEval finalE =
        evalCollectPoint(
            ourPos, cap, currentCollectTarget, goal, cellM, dyn, enemyIntent, allyIntent);
    finalE.banditBonus = regionBanditBonus(dyn, currentCollectTouch, now);
    finalE.score += finalE.banditBonus;

    finalE.units = finalOnUnits;
    finalE.count = finalOnCount;
    finalE.evidence = finalEv;
    finalE.coreCount = cCoreFinal;
    finalE.coreDist = nnCore.getDistance(currentCollectTouch);

    if (finalE.units < minUnits * 0.65
        || finalE.count < Math.max(1, minCount - 1)
        || finalE.evidence < minEv
        || finalE.coreCount < 1
        || inShootBand.test(currentCollectTarget)
        || inShootBand.test(currentCollectTouch)) {
      addDepletedMark(currentCollectTarget, 0.65, 1.25, DEPLETED_TTL_S, false);
      addDepletedRing(currentCollectTarget, 0.35, 0.95, 0.80, DEPLETED_TTL_S);
      recordRegionAttempt(dyn, currentCollectTarget, now, false);
      currentCollectTarget = null;
      currentCollectTouch = null;
      collectArrivalTs = -1.0;
      // Logger.recordOutput("Repulsor/Collect/ChosenInvalid", 1.0);
      return null;
    }

    lastReturnedCollect = currentCollectTarget;
    lastReturnedCollectTs = now;

    Logger.recordOutput(
        "Repulsor/ChosenCollect", new Pose2d(currentCollectTarget, currentCollectHeading));
    // Logger.recordOutput("Repulsor/Collect/ChosenFuelUnits", finalE.units);
    // Logger.recordOutput("Repulsor/Collect/ChosenCount", finalE.count);
    // Logger.recordOutput("Repulsor/Collect/ChosenScore", finalE.score);
    // Logger.recordOutput("Repulsor/Collect/ChosenEta", finalE.eta);
    // Logger.recordOutput("Repulsor/Collect/ChosenEvidence", finalE.evidence);
    // Logger.recordOutput("Repulsor/Collect/ChosenDep", finalE.depleted);
    // Logger.recordOutput("Repulsor/Collect/ChosenAvoid", finalE.localAvoid);
    // Logger.recordOutput("Repulsor/Collect/ChosenEnemyP", finalE.enemyPressure);
    // Logger.recordOutput("Repulsor/Collect/ChosenAllyC", finalE.allyCongestion);
    // Logger.recordOutput("Repulsor/Collect/ChosenActivity", finalE.activity);

    if (topN > 0) {
      // Logger.recordOutput("Repulsor/Collect/Top/Score", topScore);
      // Logger.recordOutput("Repulsor/Collect/Top/Eta", topEta);
      // Logger.recordOutput("Repulsor/Collect/Top/Units", topUnits);
      // Logger.recordOutput("Repulsor/Collect/Top/Region", topRegion);
      // Logger.recordOutput("Repulsor/Collect/Top/Dep", topDep);
      // Logger.recordOutput("Repulsor/Collect/Top/Avoid", topAvoid);
      // Logger.recordOutput("Repulsor/Collect/Top/EnemyP", topEP);
      // Logger.recordOutput("Repulsor/Collect/Top/AllyC", topAC);
      // Logger.recordOutput("Repulsor/Collect/Top/Activity", topAct);
      // Logger.recordOutput("Repulsor/Collect/Top/Evidence", topEv);
    }

    return new PointCandidate(
        currentCollectTarget,
        currentCollectHeading,
        finalE.eta,
        finalE.value,
        finalE.enemyPressure,
        finalE.allyCongestion,
        finalE.enemyIntent,
        finalE.allyIntent,
        finalE.score);
  }

  public PointCandidate rankCollectHierarchical(
      Translation2d ourPos,
      double ourSpeedCap,
      Translation2d[] points,
      double cellM,
      int goalUnits,
      int coarseTopK,
      int refineGrid) {

    if (ourPos == null) return null;

    SpatialDyn dyn = cachedDyn();
    if (dyn == null || dyn.resources.isEmpty()) return null;

    lastOurPosForCollect = ourPos;
    lastOurCapForCollect = ourSpeedCap > 0.0 ? ourSpeedCap : DEFAULT_OUR_SPEED;
    lastGoalUnitsForCollect = Math.max(1, goalUnits);
    lastCellMForCollect = Math.max(0.10, cellM);

    double cap = lastOurCapForCollect;
    int goal = lastGoalUnitsForCollect;

    sweepDepletedMarks();

    Translation2d[] targets = buildCollectCandidates(points, dyn);
    if (targets.length == 0) return null;

    double totalEv = dyn.totalEvidence();
    double minUnits = dynamicMinUnits(totalEv);
    int minCount = dynamicMinCount(totalEv);

    FuelRegions enemyRegions = buildFuelRegions(dyn, ENEMY_REGIONS_MAX);
    IntentAggCont enemyIntent = enemyIntentToRegions(enemyMap, enemyRegions);
    IntentAggCont allyIntent = allyIntentToRegions(allyMap, enemyRegions);

    int topK = Math.max(1, Math.min(coarseTopK, Math.max(2, targets.length)));
    int rg = Math.max(2, refineGrid);

    double half = Math.max(0.05, cellM * 0.5);
    double step = (2.0 * half) / (rg - 1);

    int[] bestIdx = new int[topK];
    double[] bestScore = new double[topK];
    for (int i = 0; i < topK; i++) {
      bestIdx[i] = -1;
      bestScore[i] = -1e18;
    }

    int fallbackIdx = -1;
    double fallbackUnits = -1e18;

    double minEv = minEvidence(totalEv);

    for (int i = 0; i < targets.length; i++) {
      Translation2d cpt = targets[i];
      if (cpt == null) continue;

      double regionUnitsAny = dyn.valueInSquare(cpt, half);
      if (regionUnitsAny > fallbackUnits) {
        fallbackUnits = regionUnitsAny;
        fallbackIdx = i;
      }

      double ev = dyn.evidenceMassWithin(cpt, EVIDENCE_R);
      if (ev < minEv && regionUnitsAny < Math.max(0.03, minUnits * 0.70)) continue;

      double eta = estimateTravelTime(ourPos, cpt, cap);
      double near = Math.exp(-COLLECT_NEAR_DECAY * eta);

      if (regionUnitsAny < Math.min(COLLECT_COARSE_MIN_REGION_UNITS, minUnits * 0.90)) continue;

      double sat = 1.0 - Math.exp(-COLLECT_VALUE_SAT_K * Math.max(0.0, regionUnitsAny));
      double value = sat * goal * COLLECT_VALUE_GAIN;

      CollectEval e = evalCollectPoint(ourPos, cap, cpt, goal, cellM, dyn, enemyIntent, allyIntent);
      double activity =
          Math.min(
              ACTIVITY_CAP,
              COLLECT_ACTIVITY_ALLY_W * radialDensity(allyMap, cpt, COLLECT_ACTIVITY_SIGMA)
                  + COLLECT_ACTIVITY_ENEMY_W * radialDensity(enemyMap, cpt, COLLECT_ACTIVITY_SIGMA)
                  + COLLECT_ACTIVITY_DYN_W * dyn.otherDensity(cpt, COLLECT_ACTIVITY_SIGMA));

      double score =
          value * COLLECT_REGION_SAMPLES_W
              - eta * COLLECT_ETA_COST
              + near * COLLECT_NEAR_BONUS
              - e.enemyPressure * COLLECT_ENEMY_PRESS_COST
              - e.allyCongestion * COLLECT_ALLY_CONGEST_COST
              - e.enemyIntent * COLLECT_ENEMY_INTENT_COST
              - e.allyIntent * COLLECT_ALLY_INTENT_COST
              - activity
              - e.depleted * DEPLETED_PEN_W
              + regionBanditBonus(dyn, cpt, Timer.getFPGATimestamp());

      if (ev < minEv) score -= 2.00;

      int insertAt = -1;
      double worst = 1e18;
      int worstK = -1;
      for (int k = 0; k < topK; k++) {
        if (bestIdx[k] < 0) {
          insertAt = k;
          break;
        }
        if (bestScore[k] < worst) {
          worst = bestScore[k];
          worstK = k;
        }
      }
      if (insertAt < 0) {
        if (score > worst && worstK >= 0) insertAt = worstK;
      }
      if (insertAt >= 0) {
        bestIdx[insertAt] = i;
        bestScore[insertAt] = score;
      }
    }

    boolean anyCoarse = false;
    for (int k = 0; k < topK; k++) if (bestIdx[k] >= 0) anyCoarse = true;

    if (!anyCoarse && fallbackIdx >= 0) {
      bestIdx[0] = fallbackIdx;
      bestScore[0] = -1e12;
    }

    final int MAX_TRIES = 10;

    double[] candScore = new double[MAX_TRIES];
    Translation2d[] candPt = new Translation2d[MAX_TRIES];
    CollectEval[] candEval = new CollectEval[MAX_TRIES];
    int candN = 0;

    for (int k = 0; k < topK; k++) {
      int idx = bestIdx[k];
      if (idx < 0) continue;

      Translation2d center = targets[idx];
      if (center == null) continue;

      for (int ix = 0; ix < rg; ix++) {
        double ox = -half + ix * step;
        for (int iy = 0; iy < rg; iy++) {
          double oy = -half + iy * step;
          Translation2d p = new Translation2d(center.getX() + ox, center.getY() + oy);

          CollectEval e =
              evalCollectPoint(ourPos, cap, p, goal, cellM, dyn, enemyIntent, allyIntent);
          e.banditBonus = regionBanditBonus(dyn, p, Timer.getFPGATimestamp());
          e.score += e.banditBonus;

          if (e.units < minUnits * 0.55 || e.count < Math.max(1, minCount - 1)) e.score -= 2.75;
          if (e.evidence < minEv) e.score -= 2.25;

          int insert = -1;
          if (candN < MAX_TRIES) {
            insert = candN++;
          } else {
            int worstI = 0;
            double worstS = candScore[0];
            for (int t = 1; t < candN; t++) {
              if (candScore[t] < worstS) {
                worstS = candScore[t];
                worstI = t;
              }
            }
            if (e.score > worstS) insert = worstI;
          }

          if (insert >= 0) {
            candScore[insert] = e.score;
            candPt[insert] = p;
            candEval[insert] = e;
          }
        }
      }
    }

    if (candN == 0) return null;

    int[] order = new int[candN];
    for (int i = 0; i < candN; i++) order[i] = i;

    for (int i = 0; i < candN - 1; i++) {
      int best = i;
      double bestS = candScore[order[i]];
      for (int j = i + 1; j < candN; j++) {
        double s = candScore[order[j]];
        if (s > bestS) {
          best = j;
          bestS = s;
        }
      }
      int tmp = order[i];
      order[i] = order[best];
      order[best] = tmp;
    }

    double now = Timer.getFPGATimestamp();

    for (int oi = 0; oi < candN; oi++) {
      int ci = order[oi];
      Translation2d p = candPt[ci];
      CollectEval e0 = candEval[ci];
      if (p == null || e0 == null) continue;

      Translation2d snapped = dyn.centroidResourcesWithin(p, 0.85, 0.20);
      Translation2d use = p;
      CollectEval eUse = e0;

      if (snapped != null) {
        CollectEval es =
            evalCollectPoint(ourPos, cap, snapped, goal, cellM, dyn, enemyIntent, allyIntent);
        es.banditBonus = regionBanditBonus(dyn, snapped, now);
        es.score += es.banditBonus;

        if (es.units < minUnits * 0.55 || es.count < Math.max(1, minCount - 1)) es.score -= 2.75;
        if (es.evidence < minEv) es.score -= 2.25;

        if (es.score > e0.score + 1e-9) {
          use = snapped;
          eUse = es;
        }
      }

      if (eUse.units >= Math.max(0.02, minUnits * 0.70)
          && eUse.count >= Math.max(1, minCount - 1)) {
        lastReturnedCollect = use;
        lastReturnedCollectTs = now;

        Logger.recordOutput("Repulsor/ChosenCollect", new Pose2d(use, new Rotation2d()));
        // Logger.recordOutput("Repulsor/Collect/ChosenFuelUnits", eUse.units);
        // Logger.recordOutput("Repulsor/Collect/ChosenCount", eUse.count);
        // Logger.recordOutput("Repulsor/Collect/ChosenScore", eUse.score);
        // Logger.recordOutput("Repulsor/Collect/ChosenEta", eUse.eta);
        // Logger.recordOutput("Repulsor/Collect/ChosenEvidence", eUse.evidence);

        return new PointCandidate(
            use,
            new Rotation2d(),
            eUse.eta,
            eUse.value,
            eUse.enemyPressure,
            eUse.allyCongestion,
            eUse.enemyIntent,
            eUse.allyIntent,
            eUse.score);
      }

      addDepletedMark(use, 0.70, 1.20, DEPLETED_TTL_S, false);
      addDepletedRing(use, 0.35, 0.95, 0.75, DEPLETED_TTL_S);
      recordRegionAttempt(dyn, use, now, false);
    }

    return null;
  }

  public Translation2d bestCollectHotspot(Translation2d[] points, double cellM) {
    if (points == null || points.length == 0) return null;
    SpatialDyn dyn = cachedDyn();
    if (dyn == null || dyn.resources.isEmpty()) return null;

    double half = Math.max(0.05, cellM * 0.5);
    Translation2d best = null;
    double bestU = 0.0;

    for (Translation2d p : points) {
      if (p == null) continue;
      double u = dyn.valueInSquare(p, half);
      if (u > bestU) {
        bestU = u;
        best = p;
      }
    }

    double min = Math.max(0.02, COLLECT_FINE_MIN_UNITS * 0.5);
    return bestU >= min ? best : null;
  }

  public PointCandidate rankCollectPoints(
      Translation2d ourPos, double ourSpeedCap, Translation2d[] points, int goalUnits, int limit) {

    if (ourPos == null) return null;

    SpatialDyn dyn = cachedDyn();
    if (dyn == null || dyn.resources.isEmpty()) return null;

    lastOurPosForCollect = ourPos;
    lastOurCapForCollect = ourSpeedCap > 0.0 ? ourSpeedCap : DEFAULT_OUR_SPEED;
    lastGoalUnitsForCollect = Math.max(1, goalUnits);
    lastCellMForCollect = COLLECT_CELL_M;

    sweepDepletedMarks();

    Translation2d[] targets = buildCollectCandidates(points, dyn);
    if (targets == null || targets.length == 0) return null;

    double totalEv = dyn.totalEvidence();
    double minUnits = dynamicMinUnits(totalEv);
    int minCount = dynamicMinCount(totalEv);

    FuelRegions enemyRegions = buildFuelRegions(dyn, ENEMY_REGIONS_MAX);
    IntentAggCont enemyIntent = enemyIntentToRegions(enemyMap, enemyRegions);
    IntentAggCont allyIntent = allyIntentToRegions(allyMap, enemyRegions);

    double cap = lastOurCapForCollect;
    int goal = lastGoalUnitsForCollect;

    int maxCheck = limit > 0 ? Math.min(Math.max(1, limit), targets.length) : targets.length;

    double[] etas = new double[targets.length];
    int[] order = new int[targets.length];
    for (int i = 0; i < targets.length; i++) {
      order[i] = i;
      Translation2d t = targets[i];
      etas[i] = t != null ? estimateTravelTime(ourPos, t, cap) : Double.POSITIVE_INFINITY;
    }

    sortIdxByKey(etas, order);

    Translation2d bestP = null;
    CollectEval bestE = null;

    double minEv = minEvidence(totalEv);

    for (int oi = 0; oi < order.length && oi < maxCheck; oi++) {
      int i = order[oi];
      Translation2d p = targets[i];
      if (p == null) continue;

      CollectEval e =
          evalCollectPoint(ourPos, cap, p, goal, COLLECT_CELL_M, dyn, enemyIntent, allyIntent);
      e.banditBonus = regionBanditBonus(dyn, p, Timer.getFPGATimestamp());
      e.score += e.banditBonus;

      if (e.units < minUnits * 0.55 || e.count < Math.max(1, minCount - 1)) e.score -= 2.75;
      if (e.evidence < minEv) e.score -= 2.25;

      if (bestE == null || e.score > bestE.score + 1e-9) {
        bestE = e;
        bestP = p;
      }
    }

    if (bestP == null || bestE == null) return null;

    if (bestE.units < Math.max(0.02, minUnits * 0.70) || bestE.count < Math.max(1, minCount - 1)) {
      addDepletedMark(bestP, 0.70, 1.15, DEPLETED_TTL_S, false);
      addDepletedRing(bestP, 0.35, 0.95, 0.75, DEPLETED_TTL_S);
      return null;
    }

    lastReturnedCollect = bestP;
    lastReturnedCollectTs = Timer.getFPGATimestamp();

    Logger.recordOutput("Repulsor/ChosenCollect", new Pose2d(bestP, new Rotation2d()));
    // Logger.recordOutput("Repulsor/Collect/ChosenFuelUnits", bestE.units);
    // Logger.recordOutput("Repulsor/Collect/ChosenCount", bestE.count);
    // Logger.recordOutput("Repulsor/Collect/ChosenScore", bestE.score);

    return new PointCandidate(
        bestP,
        new Rotation2d(),
        bestE.eta,
        bestE.value,
        bestE.enemyPressure,
        bestE.allyCongestion,
        bestE.enemyIntent,
        bestE.allyIntent,
        bestE.score);
  }

  private Translation2d[] buildCollectCandidates(Translation2d[] gridPoints, SpatialDyn dyn) {
    Translation2d[] clusters = buildResourceClustersMulti(dyn, COLLECT_CLUSTER_MAX);

    ArrayList<Translation2d> peaks = new ArrayList<>(64);
    if (dyn != null && dyn.resources != null && !dyn.resources.isEmpty() && gridPoints != null) {
      Translation2d[] peakPts = peakFinder(gridPoints, dyn, PEAK_FINDER_TOPN);
      for (Translation2d p : peakPts) if (p != null) peaks.add(p);
    }

    ArrayList<Translation2d> out = new ArrayList<>(256);
    for (Translation2d c : clusters) if (c != null) out.add(c);
    for (Translation2d p : peaks) if (p != null) out.add(p);

    if (dyn != null
        && dyn.resources != null
        && !dyn.resources.isEmpty()
        && dyn.specs != null
        && !dyn.specs.isEmpty()) {
      Translation2d[] bestRes = new Translation2d[Math.max(1, COLLECT_RESOURCE_SEEDS_MAX)];
      double[] bestW = new double[bestRes.length];
      for (int i = 0; i < bestW.length; i++) bestW[i] = -1e18;

      for (DynamicObject o : dyn.resources) {
        if (o == null || o.pos == null || o.type == null) continue;
        ResourceSpec s = dyn.specs.get(o.type.toLowerCase());
        if (s == null) continue;

        double ageW = error() ? 1.0 : Math.exp(-COLLECT_AGE_DECAY * Math.max(0.0, o.ageS));
        double w = Math.max(0.0, s.unitValue) * ageW;

        int worstI = 0;
        double worst = bestW[0];
        for (int k = 1; k < bestW.length; k++) {
          if (bestW[k] < worst) {
            worst = bestW[k];
            worstI = k;
          }
        }
        if (w > worst) {
          bestW[worstI] = w;
          bestRes[worstI] = o.pos;
        }
      }

      for (Translation2d p : bestRes) if (p != null) out.add(p);
    }

    if (gridPoints == null || gridPoints.length == 0) {
      return spreadCollectPoints(dedupPoints(out, adaptiveDupSkip(dyn)), dyn)
          .toArray(new Translation2d[0]);
    }

    int nGrid = 0;
    for (Translation2d p : gridPoints) if (p != null) nGrid++;
    if (nGrid == 0) {
      return spreadCollectPoints(dedupPoints(out, adaptiveDupSkip(dyn)), dyn)
          .toArray(new Translation2d[0]);
    }

    int K = Math.min(COLLECT_GRID_TOPK, Math.min(nGrid, COLLECT_GRID_FALLBACK_MAX));
    Translation2d[] bestP = new Translation2d[K];
    Translation2d[] bestN = new Translation2d[K];
    double[] bestS = new double[K];
    for (int i = 0; i < K; i++) bestS[i] = -1e18;

    double gateR = COLLECT_CAND_GATE_R;

    for (int i = 0; i < gridPoints.length; i++) {
      Translation2d p = gridPoints[i];
      if (p == null) continue;
      if (dyn == null) continue;

      Translation2d near = dyn.nearestResourceTo(p, gateR);
      if (near == null) continue;

      double s = dyn.evidenceMassWithin(p, 0.85);
      if (s <= 1e-9) continue;

      int worstI = 0;
      double worst = bestS[0];
      for (int k = 1; k < K; k++) {
        if (bestS[k] < worst) {
          worst = bestS[k];
          worstI = k;
        }
      }
      if (s > worst) {
        bestS[worstI] = s;
        bestP[worstI] = p;
        bestN[worstI] = near;
      }
    }

    for (int i = 0; i < K; i++) {
      Translation2d p = bestP[i];
      if (p != null) out.add(p);
      Translation2d q = bestN[i];
      if (q != null) out.add(q);
    }

    return spreadCollectPoints(dedupPoints(out, adaptiveDupSkip(dyn)), dyn)
        .toArray(new Translation2d[0]);
  }

  private ArrayList<Translation2d> dedupPoints(ArrayList<Translation2d> in, double dupSkipM) {
    if (in == null || in.isEmpty()) return new ArrayList<>();
    double d2 = dupSkipM * dupSkipM;
    ArrayList<Translation2d> out = new ArrayList<>(in.size());
    for (int i = 0; i < in.size(); i++) {
      Translation2d p = in.get(i);
      if (p == null) continue;
      boolean dup = false;
      for (int j = 0; j < out.size(); j++) {
        Translation2d q = out.get(j);
        double dx = q.getX() - p.getX();
        double dy = q.getY() - p.getY();
        if (dx * dx + dy * dy <= d2) {
          dup = true;
          break;
        }
      }
      if (!dup) out.add(p);
    }
    return out;
  }

  private ArrayList<Translation2d> spreadCollectPoints(
      ArrayList<Translation2d> in, SpatialDyn dyn) {
    if (in == null || in.isEmpty()) return new ArrayList<>();
    double sep = adaptiveCollectSeparation(dyn);
    double d2 = sep * sep;
    int n = in.size();
    double[] score = new double[n];
    for (int i = 0; i < n; i++) {
      Translation2d p = in.get(i);
      score[i] =
          (dyn != null && p != null) ? dyn.evidenceMassWithin(p, COLLECT_SPREAD_SCORE_R) : 0.0;
    }

    boolean[] blocked = new boolean[n];
    ArrayList<Translation2d> out = new ArrayList<>(n);

    for (; ; ) {
      int bestI = -1;
      double best = -1e18;
      for (int i = 0; i < n; i++) {
        if (blocked[i]) continue;
        if (score[i] > best) {
          best = score[i];
          bestI = i;
        }
      }
      if (bestI < 0) break;
      Translation2d p = in.get(bestI);
      if (p != null) out.add(p);
      blocked[bestI] = true;
      if (p == null) continue;
      for (int j = 0; j < n; j++) {
        if (blocked[j]) continue;
        Translation2d q = in.get(j);
        if (q == null) {
          blocked[j] = true;
          continue;
        }
        double dx = q.getX() - p.getX();
        double dy = q.getY() - p.getY();
        if (dx * dx + dy * dy <= d2) blocked[j] = true;
      }
    }
    return out;
  }

  private double adaptiveDupSkip(SpatialDyn dyn) {
    double total = dyn != null ? dyn.totalEvidence() : 0.0;
    double x = clamp01(total / 6.0);
    return lerp(0.18, 0.42, x);
  }

  private double adaptiveCollectSeparation(SpatialDyn dyn) {
    double total = dyn != null ? dyn.totalEvidence() : 0.0;
    double x = clamp01(total / 6.0);
    return lerp(COLLECT_SPREAD_MIN, COLLECT_SPREAD_MAX, x);
  }

  private static final class ClusterAcc {
    double sx;
    double sy;
    double sw;
    int n;
  }

  private Translation2d[] buildResourceClustersMulti(SpatialDyn dyn, int maxClusters) {
    if (dyn == null || dyn.resources.isEmpty() || dyn.specs.isEmpty()) return new Translation2d[0];

    double coarseBin = Math.max(0.25, Math.min(0.35, 0.30));
    double inv = 1.0 / Math.max(1e-6, coarseBin);

    HashMap<Long, ClusterAcc> coarse = new HashMap<>(256);

    for (DynamicObject o : dyn.resources) {
      if (o == null || o.pos == null || o.type == null) continue;
      ResourceSpec s = dyn.specs.get(o.type.toLowerCase());
      if (s == null) continue;

      int cx = (int) Math.floor(o.pos.getX() * inv);
      int cy = (int) Math.floor(o.pos.getY() * inv);
      long k = SpatialDyn.key(cx, cy);

      ClusterAcc a = coarse.get(k);
      if (a == null) {
        a = new ClusterAcc();
        coarse.put(k, a);
      }

      double ageW = error() ? 1.0 : Math.exp(-COLLECT_AGE_DECAY * Math.max(0.0, o.ageS));
      double w = Math.max(0.0, s.unitValue) * ageW;

      a.sx += o.pos.getX() * w;
      a.sy += o.pos.getY() * w;
      a.sw += w;
      a.n++;
    }

    if (coarse.isEmpty()) return new Translation2d[0];

    ArrayList<long[]> scored = new ArrayList<>(coarse.size());
    for (var e : coarse.entrySet()) {
      ClusterAcc a = e.getValue();
      if (a == null || a.sw <= 1e-9) continue;
      scored.add(new long[] {e.getKey(), Double.doubleToLongBits(a.sw)});
    }

    if (scored.isEmpty()) return new Translation2d[0];

    scored.sort(
        (u, v) -> {
          double a = Double.longBitsToDouble(u[1]);
          double b = Double.longBitsToDouble(v[1]);
          return Double.compare(b, a);
        });

    int m = Math.min(maxClusters, scored.size());
    ArrayList<Translation2d> out = new ArrayList<>(m);

    for (int i = 0; i < m; i++) {
      long key = scored.get(i)[0];
      ClusterAcc a = coarse.get(key);
      if (a == null || a.sw <= 1e-9) continue;

      Translation2d c = new Translation2d(a.sx / a.sw, a.sy / a.sw);

      Translation2d refined = meanShiftRefine(dyn, c, 0.75, 3);
      if (refined != null) out.add(refined);
    }

    out = dedupPoints(out, 0.28);
    return out.toArray(new Translation2d[0]);
  }

  private Translation2d meanShiftRefine(SpatialDyn dyn, Translation2d seed, double r, int iters) {
    if (dyn == null || seed == null) return null;
    Translation2d c = seed;
    double rr2 = r * r;

    for (int it = 0; it < Math.max(1, iters); it++) {
      double sx = 0.0;
      double sy = 0.0;
      double sw = 0.0;

      for (DynamicObject o : dyn.resources) {
        if (o == null || o.pos == null || o.type == null) continue;
        ResourceSpec s = dyn.specs.get(o.type.toLowerCase());
        if (s == null) continue;

        double dx = o.pos.getX() - c.getX();
        double dy = o.pos.getY() - c.getY();
        double d2 = dx * dx + dy * dy;
        if (d2 > rr2) continue;

        double ageW = error() ? 1.0 : Math.exp(-COLLECT_AGE_DECAY * Math.max(0.0, o.ageS));
        double w = Math.max(0.0, s.unitValue) * ageW;

        sx += o.pos.getX() * w;
        sy += o.pos.getY() * w;
        sw += w;
      }

      if (sw <= 1e-9) break;

      Translation2d nc = new Translation2d(sx / sw, sy / sw);
      if (nc.getDistance(c) <= 0.02) {
        c = nc;
        break;
      }
      c = nc;
    }

    return c;
  }

  private void sweepDepletedMarks() {
    if (error()) return;
    if (depletedMarks.isEmpty()) return;
    double now = Timer.getFPGATimestamp();
    depletedMarks.removeIf(m -> now - m.t > m.ttl);
    if (depletedMarks.size() > DEPLETED_MARKS_MAX) {
      while (depletedMarks.size() > DEPLETED_MARKS_MAX) depletedMarks.remove(0);
    }
  }

  private void addDepletedMark(
      Translation2d p, double radiusM, double strength, double ttlS, boolean merge) {
    if (error()) return;
    if (p == null) return;

    double now = Timer.getFPGATimestamp();
    double r = Math.max(0.10, radiusM);
    double s = Math.max(0.0, strength);
    double ttl = Math.max(0.1, ttlS);

    if (merge) {
      for (int i = depletedMarks.size() - 1; i >= 0; i--) {
        DepletedMark m = depletedMarks.get(i);
        if (m == null || m.ring) continue;
        if (m.p.getDistance(p) <= 0.30) {
          m.t = now;
          m.s = Math.max(m.s, s);
          m.r = Math.max(m.r, r);
          m.ttl = Math.max(m.ttl, ttl);
          return;
        }
      }
    }

    depletedMarks.add(new DepletedMark(p, now, s, r, ttl));
    if (depletedMarks.size() > DEPLETED_MARKS_MAX) depletedMarks.remove(0);
  }

  private void addDepletedRing(
      Translation2d p, double r0, double r1, double strength, double ttlS) {
    if (error()) return;
    if (p == null) return;
    double now = Timer.getFPGATimestamp();
    depletedMarks.add(
        new DepletedMark(p, now, Math.max(0.0, strength), r0, r1, Math.max(0.1, ttlS)));
    if (depletedMarks.size() > DEPLETED_MARKS_MAX) depletedMarks.remove(0);
  }

  private double depletedPenaltySoft(Translation2d p) {
    if (error()) return 0.0;
    if (p == null || depletedMarks.isEmpty()) return 0.0;

    double now = Timer.getFPGATimestamp();
    double sum = 0.0;

    for (int i = 0; i < depletedMarks.size(); i++) {
      DepletedMark m = depletedMarks.get(i);
      if (m == null) continue;
      double age = Math.max(0.0, now - m.t);
      if (age > m.ttl) continue;

      double w = m.s * Math.exp(-DEPLETED_DECAY * age);

      double d = m.p.getDistance(p);

      if (!m.ring) {
        double sig2 = Math.max(1e-6, m.r * m.r);
        sum += w * Math.exp(-0.5 * (d * d) / sig2);
      } else {
        double mid = 0.5 * (m.ringR0 + m.ringR1);
        double width = Math.max(0.08, 0.33 * (m.ringR1 - m.ringR0));
        double x = d - mid;
        double sig2 = Math.max(1e-6, width * width);
        sum += w * Math.exp(-0.5 * (x * x) / sig2);
      }
    }

    return Math.min(2.25, sum);
  }

  private static Translation2d lerpVec(Translation2d a, Translation2d b, double alpha) {
    double t = Math.max(0.0, Math.min(1.0, alpha));
    return new Translation2d(
        a.getX() + (b.getX() - a.getX()) * t, a.getY() + (b.getY() - a.getY()) * t);
  }

  private static double emaAlpha(double baseAlpha, double dt) {
    double a = Math.max(0.0, Math.min(1.0, baseAlpha));
    double x = Math.max(0.0, dt / MIN_DT);
    return 1.0 - Math.pow(1.0 - a, x);
  }

  private static Translation2d clampDeltaV(
      Translation2d oldV, Translation2d newV, double aMax, double dt) {
    double lim = Math.max(0.0, aMax) * Math.max(0.0, dt);
    Translation2d dv = newV.minus(oldV);
    double n = dv.getNorm();
    if (n <= lim || n <= 1e-9) return newV;
    Translation2d dvClamped = dv.div(n).times(lim);
    return oldV.plus(dvClamped);
  }

  private static double etaPath(Translation2d a, Translation2d b, double speed) {
    double d = a.getDistance(b);
    return Math.max(ETA_FLOOR, d / Math.max(0.1, speed));
  }

  private double estimateTravelTime(Translation2d a, Translation2d b, double speed) {
    double base = etaPath(a, b, speed);
    if (base <= ETA_FLOOR + 1e-9) return base;

    SpatialDyn dyn = cachedDyn();
    if (dyn == null) return base;

    double seg = a.getDistance(b);
    if (seg < 1e-3) return base;

    double inv = 1.0 / Math.max(1.0, (double) (PATH_SAMPLES - 1));
    double cost = 0.0;

    for (int i = 0; i < PATH_SAMPLES; i++) {
      double t = i * inv;
      Translation2d p = new Translation2d(lerp(a.getX(), b.getX(), t), lerp(a.getY(), b.getY(), t));

      double c =
          0.85 * radialDensity(allyMap, p, 1.00)
              + 0.95 * radialDensity(enemyMap, p, 1.00)
              + 0.70 * dyn.otherDensity(p, 1.00);

      cost += c;
    }

    cost /= Math.max(1.0, PATH_SAMPLES);
    double mult = 1.0 + PATH_COST_W * Math.min(1.35, cost);
    return Math.max(ETA_FLOOR, base * mult);
  }

  private static double minEtaToTarget(HashMap<Integer, Track> map, Translation2d t) {
    double best = Double.POSITIVE_INFINITY;
    for (Track r : map.values()) {
      double s = Math.max(0.1, r.speedCap);
      double e = etaPath(r.pos, t, s);
      if (e < best) best = e;
    }
    return best;
  }

  private static double dotNorm(Translation2d a, Translation2d b) {
    double an = a.getNorm();
    double bn = b.getNorm();
    if (an < 1e-6 || bn < 1e-6) return 0.0;
    return (a.getX() * b.getX() + a.getY() * b.getY()) / (an * bn);
  }

  private double radialKernel(double dist) {
    double s2 = KERNEL_SIGMA * KERNEL_SIGMA;
    return Math.exp(-0.5 * (dist * dist) / Math.max(1e-6, s2));
  }

  private static double densityKernel(double d, double sigma) {
    double s2 = sigma * sigma;
    return Math.exp(-0.5 * (d * d) / Math.max(1e-6, s2));
  }

  private static double radialDensity(
      HashMap<Integer, Track> map, Translation2d target, double sigma) {
    if (map.isEmpty()) return 0.0;
    double agg = 0.0;
    for (Track r : map.values()) {
      double d = r.pos.getDistance(target);
      agg += densityKernel(d, sigma);
    }
    return agg / Math.max(1.0, map.size());
  }

  private static Translation2d predictAt(Track r, double horizonS) {
    if (r == null) return new Translation2d();
    double h = Math.max(0.0, horizonS);
    if (r.vel == null || r.vel.getNorm() < 1e-9) return r.pos;
    return r.pos.plus(r.vel.times(h));
  }

  private double radialPressure(
      HashMap<Integer, Track> enemies,
      Translation2d target,
      double ourEtaS,
      double intentMass,
      int count) {
    if (enemies.isEmpty()) return 0.0;
    double agg = 0.0;
    for (Track r : enemies.values()) {
      Translation2d p = predictAt(r, ourEtaS);
      double d = p.getDistance(target);
      double k = radialKernel(Math.max(0.0, d - RESERVATION_RADIUS));
      agg += k;
    }
    double base = agg / Math.max(1.0, enemies.size());
    double intent = count > 0 ? (intentMass / Math.max(1.0, count)) : 0.0;
    return base * (1.0 + 0.85 * intent);
  }

  private double radialCongestion(
      HashMap<Integer, Track> allies,
      Translation2d target,
      double ourEtaS,
      double intentMass,
      int count) {
    if (allies.isEmpty()) return 0.0;
    double agg = 0.0;
    for (Track r : allies.values()) {
      Translation2d p = predictAt(r, ourEtaS);
      double d = p.getDistance(target);
      double k = radialKernel(Math.max(0.0, d - RESERVATION_RADIUS));
      agg += k;
    }
    double base = agg / Math.max(1.0, allies.size());
    double intent = count > 0 ? (intentMass / Math.max(1.0, count)) : 0.0;
    return base * (1.0 + 0.75 * intent);
  }

  private double headingAffinity(
      Translation2d ourPos,
      Translation2d target,
      HashMap<Integer, Track> allies,
      HashMap<Integer, Track> enemies) {
    Translation2d toTarget = target.minus(ourPos);
    if (toTarget.getNorm() < 1e-6) return 0.0;
    double allyAlign = 0.0;
    int na = 0;
    for (Track a : allies.values()) {
      Translation2d to = target.minus(a.pos);
      if (to.getNorm() < 1e-6 || a.vel.getNorm() < 1e-6) continue;
      double c = dotNorm(to, a.vel);
      allyAlign += Math.max(0.0, c);
      na++;
    }
    if (na > 0) allyAlign /= na;
    double enemyMis = 0.0;
    int ne = 0;
    for (Track e : enemies.values()) {
      Translation2d to = target.minus(e.pos);
      if (to.getNorm() < 1e-6 || e.vel.getNorm() < 1e-6) continue;
      double c = dotNorm(to, e.vel);
      enemyMis += Math.max(0.0, 1.0 - c);
      ne++;
    }
    if (ne > 0) enemyMis /= ne;
    return 0.6 * allyAlign + 0.4 * enemyMis;
  }

  private static final class IntentAgg {
    final double[] intent;
    final int count;

    IntentAgg(double[] intent, int count) {
      this.intent = intent;
      this.count = count;
    }
  }

  private IntentAgg softIntentAgg(HashMap<Integer, Track> map, List<Translation2d> targets) {
    int m = targets.size();
    double[] accum = new double[m];
    if (map.isEmpty() || m == 0) return new IntentAgg(accum, 0);

    ensureScratch(m);
    double[] logits = scratchLogits;
    double[] probs = scratchLogits2;

    int n = 0;
    double invT = 1.0 / Math.max(0.2, SOFTMAX_TEMP);

    for (Track r : map.values()) {
      n++;
      double maxLogit = -1e18;

      for (int i = 0; i < m; i++) {
        Translation2d t = targets.get(i);
        double d = r.pos.getDistance(t);
        double eta = d / Math.max(0.1, r.speedCap);

        Translation2d dir = t.minus(r.pos);
        double align = 0.0;
        if (dir.getNorm() > 1e-6 && r.vel.getNorm() > 1e-6) {
          align = Math.max(0.0, dotNorm(dir, r.vel));
        }

        double logit = -(eta * invT) + 0.35 * align;
        logits[i] = logit;
        if (logit > maxLogit) maxLogit = logit;
      }

      double sum = 0.0;
      for (int i = 0; i < m; i++) {
        double e = Math.exp(logits[i] - maxLogit);
        probs[i] = e;
        sum += e;
      }
      double inv = 1.0 / Math.max(1e-9, sum);

      for (int i = 0; i < m; i++) {
        double w = probs[i] * inv;
        accum[i] += w;
      }
    }

    return new IntentAgg(accum, n);
  }

  private static final class FuelRegions {
    final Translation2d[] centers;
    final double[] mass;

    FuelRegions(Translation2d[] centers, double[] mass) {
      this.centers = centers != null ? centers : new Translation2d[0];
      this.mass = mass != null ? mass : new double[this.centers.length];
    }
  }

  private static final class IntentAggCont {
    final Translation2d[] regions;
    final double[] intentMass;
    final int count;

    IntentAggCont(Translation2d[] regions, double[] intentMass, int count) {
      this.regions = regions != null ? regions : new Translation2d[0];
      this.intentMass = intentMass != null ? intentMass : new double[this.regions.length];
      this.count = count;
    }

    double intentAt(Translation2d p) {
      if (p == null || regions.length == 0) return 0.0;
      double sum = 0.0;
      double s2 = ENEMY_REGION_SIGMA * ENEMY_REGION_SIGMA;
      for (int i = 0; i < regions.length; i++) {
        Translation2d c = regions[i];
        if (c == null) continue;
        double d = c.getDistance(p);
        double k = Math.exp(-0.5 * (d * d) / Math.max(1e-6, s2));
        sum += intentMass[i] * k;
      }
      return sum;
    }
  }

  private FuelRegions buildFuelRegions(SpatialDyn dyn, int maxRegions) {
    if (dyn == null) return new FuelRegions(new Translation2d[0], new double[0]);
    Translation2d[] c = buildResourceClustersMulti(dyn, Math.max(4, maxRegions));
    double[] m = new double[c.length];
    for (int i = 0; i < c.length; i++) {
      Translation2d p = c[i];
      m[i] = p != null ? dyn.evidenceMassWithin(p, 0.85) : 0.0;
    }
    return new FuelRegions(c, m);
  }

  private IntentAggCont enemyIntentToRegions(HashMap<Integer, Track> map, FuelRegions regs) {
    int m = regs != null && regs.centers != null ? regs.centers.length : 0;
    double[] accum = new double[m];
    if (map.isEmpty() || m == 0) return new IntentAggCont(regs.centers, accum, 0);

    ensureScratch(m);
    double[] logits = scratchLogits;
    double[] probs = scratchLogits2;

    int n = 0;
    double invT = 1.0 / Math.max(0.2, SOFTMAX_TEMP);

    for (Track r : map.values()) {
      n++;
      double maxLogit = -1e18;

      for (int i = 0; i < m; i++) {
        Translation2d t = regs.centers[i];
        if (t == null) {
          logits[i] = -1e18;
          continue;
        }

        double d = r.pos.getDistance(t);
        double eta = d / Math.max(0.1, r.speedCap);

        Translation2d dir = t.minus(r.pos);
        double align = 0.0;
        if (dir.getNorm() > 1e-6 && r.vel.getNorm() > 1e-6) {
          align = Math.max(0.0, dotNorm(dir, r.vel));
        }

        double mass = regs.mass != null && i < regs.mass.length ? regs.mass[i] : 0.0;
        double massTerm = Math.log(1.0 + Math.max(0.0, mass));

        double logit = -(eta * invT) + 0.35 * align + 0.22 * massTerm;
        logits[i] = logit;
        if (logit > maxLogit) maxLogit = logit;
      }

      double sum = 0.0;
      for (int i = 0; i < m; i++) {
        double li = logits[i];
        if (li <= -1e17) {
          probs[i] = 0.0;
          continue;
        }
        double e = Math.exp(li - maxLogit);
        probs[i] = e;
        sum += e;
      }
      double inv = 1.0 / Math.max(1e-9, sum);

      for (int i = 0; i < m; i++) {
        accum[i] += probs[i] * inv;
      }
    }

    return new IntentAggCont(regs.centers, accum, n);
  }

  private IntentAggCont allyIntentToRegions(HashMap<Integer, Track> map, FuelRegions regs) {
    int m = regs != null && regs.centers != null ? regs.centers.length : 0;
    double[] accum = new double[m];
    if (map.isEmpty() || m == 0) return new IntentAggCont(regs.centers, accum, 0);

    ensureScratch(m);
    double[] logits = scratchLogits;
    double[] probs = scratchLogits2;

    int n = 0;
    double invT = 1.0 / Math.max(0.2, SOFTMAX_TEMP);

    for (Track r : map.values()) {
      n++;
      double maxLogit = -1e18;

      for (int i = 0; i < m; i++) {
        Translation2d t = regs.centers[i];
        if (t == null) {
          logits[i] = -1e18;
          continue;
        }

        double d = r.pos.getDistance(t);
        double eta = d / Math.max(0.1, r.speedCap);

        Translation2d dir = t.minus(r.pos);
        double align = 0.0;
        if (dir.getNorm() > 1e-6 && r.vel.getNorm() > 1e-6) {
          align = Math.max(0.0, dotNorm(dir, r.vel));
        }

        double mass = regs.mass != null && i < regs.mass.length ? regs.mass[i] : 0.0;
        double massTerm = Math.log(1.0 + Math.max(0.0, mass));

        double logit = -(eta * invT) + 0.35 * align + 0.18 * massTerm;
        logits[i] = logit;
        if (logit > maxLogit) maxLogit = logit;
      }

      double sum = 0.0;
      for (int i = 0; i < m; i++) {
        double li = logits[i];
        if (li <= -1e17) {
          probs[i] = 0.0;
          continue;
        }
        double e = Math.exp(li - maxLogit);
        probs[i] = e;
        sum += e;
      }
      double inv = 1.0 / Math.max(1e-9, sum);

      for (int i = 0; i < m; i++) {
        accum[i] += probs[i] * inv;
      }
    }

    return new IntentAggCont(regs.centers, accum, n);
  }

  private static double clamp(double x, double lo, double hi) {
    return Math.max(lo, Math.min(hi, x));
  }

  private static double coreRadiusFor(double cellM) {
    double r = 0.55 * Math.max(0.10, cellM);
    return clamp(r, COLLECT_CORE_R_MIN, COLLECT_CORE_R_MAX);
  }

  private static double snapRadiusFor(double cellM) {
    double r = 4.5 * coreRadiusFor(cellM);
    return clamp(r, COLLECT_SNAP_R_MIN, COLLECT_SNAP_R_MAX);
  }

  private static double microCentroidRadiusFor(double cellM) {
    double r = 2.6 * coreRadiusFor(cellM);
    return clamp(r, COLLECT_MICRO_CENTROID_R_MIN, COLLECT_MICRO_CENTROID_R_MAX);
  }

  private static double jitterRadiusFor(double cellM) {
    double r = 1.15 * coreRadiusFor(cellM);
    return clamp(r, COLLECT_JITTER_R_MIN, COLLECT_JITTER_R_MAX);
  }

  private Translation2d snapToNearestThenMicroCentroid(
      Translation2d p, SpatialDyn dyn, double rSnap, double rCentroid, double minMass) {
    if (p == null || dyn == null) return null;

    Translation2d nearest = dyn.nearestResourceTo(p, Math.max(0.01, rSnap));
    if (nearest == null) return null;

    Translation2d q = nearest;

    Translation2d c1 =
        dyn.centroidResourcesWithin(q, Math.max(0.05, rCentroid), Math.max(0.0, minMass));
    if (c1 != null) q = c1;

    Translation2d c2 =
        dyn.centroidResourcesWithin(
            q, Math.max(0.05, 0.60 * rCentroid), Math.max(0.0, minMass * 0.65));
    if (c2 != null) q = c2;

    return q;
  }

  private Translation2d enforceHardStopOnFuel(
      SpatialDyn dyn,
      Translation2d p,
      double rCore,
      double rSnap,
      double rCentroid,
      double minMass) {
    if (p == null || dyn == null) return null;

    Translation2d nearestCore = dyn.nearestResourceTo(p, Math.max(0.01, rCore));
    if (nearestCore != null) {
      Translation2d q = nearestCore;
      Translation2d c =
          dyn.centroidResourcesWithin(
              q, Math.max(0.05, Math.min(rCentroid, 0.22)), Math.max(0.0, minMass));
      if (c != null) q = c;
      if (dyn.nearestResourceTo(q, Math.max(0.01, rCore)) != null) return q;
      return nearestCore;
    }

    Translation2d snapped = snapToNearestThenMicroCentroid(p, dyn, rSnap, rCentroid, minMass);
    if (snapped == null) return null;

    Translation2d nearest2 = dyn.nearestResourceTo(snapped, Math.max(0.01, rCore));
    if (nearest2 == null) return null;

    Translation2d q = nearest2;
    Translation2d c =
        dyn.centroidResourcesWithin(
            q, Math.max(0.05, Math.min(rCentroid, 0.22)), Math.max(0.0, minMass));
    if (c != null) q = c;

    return dyn.nearestResourceTo(q, Math.max(0.01, rCore)) != null ? q : nearest2;
  }

  private double pickupRobustPenalty(
      SpatialDyn dyn, Translation2d p, double rCore, double jitterR) {
    if (dyn == null || p == null) return COLLECT_NOFUEL_PENALTY;

    int c0 = dyn.countResourcesWithin(p, Math.max(0.01, rCore));
    Translation2d nn0 = dyn.nearestResourceTo(p, Math.max(0.01, rCore));
    double d0 = nn0 != null ? nn0.getDistance(p) : Double.POSITIVE_INFINITY;

    double jr = Math.max(0.0, jitterR);
    if (jr <= 1e-6) return (c0 >= 1 || d0 <= rCore) ? 0.0 : COLLECT_NOFUEL_PENALTY;

    Translation2d[] samp =
        new Translation2d[] {
          p,
          new Translation2d(p.getX() + jr, p.getY()),
          new Translation2d(p.getX() - jr, p.getY()),
          new Translation2d(p.getX(), p.getY() + jr),
          new Translation2d(p.getX(), p.getY() - jr),
          new Translation2d(p.getX() + 0.7071 * jr, p.getY() + 0.7071 * jr),
          new Translation2d(p.getX() + 0.7071 * jr, p.getY() - 0.7071 * jr),
          new Translation2d(p.getX() - 0.7071 * jr, p.getY() + 0.7071 * jr),
          new Translation2d(p.getX() - 0.7071 * jr, p.getY() - 0.7071 * jr)
        };

    int minC = Integer.MAX_VALUE;
    int maxC = 0;

    for (Translation2d s : samp) {
      int c = dyn.countResourcesWithin(s, Math.max(0.01, rCore));
      minC = Math.min(minC, c);
      maxC = Math.max(maxC, c);
    }

    boolean centerOk = (c0 >= 1) || (d0 <= rCore);

    if (centerOk) {
      if (minC >= 1) return 0.0;
      return COLLECT_EDGE_PENALTY;
    }

    if (maxC >= 1) return COLLECT_HOLE_PENALTY;
    return COLLECT_NOFUEL_PENALTY;
  }

  private static final class CollectEval {
    Translation2d p;
    double eta;
    double units;
    int count;
    double evidence;
    double value;
    double enemyPressure;
    double allyCongestion;
    double enemyIntent;
    double allyIntent;
    double localAvoid;
    double activity;
    double depleted;
    double overlap;
    int coreCount;
    double coreDist;
    double robustPenalty;
    double score;
    double regionUnits;
    double banditBonus;
  }

  private CollectEval evalCollectPoint(
      Translation2d ourPos,
      double cap,
      Translation2d p,
      int goal,
      double cellM,
      SpatialDyn dyn,
      IntentAggCont enemyIntent,
      IntentAggCont allyIntent) {

    CollectEval e = new CollectEval();
    e.p = p != null ? p : new Translation2d();
    if (p == null || dyn == null) {
      e.score = -1e18;
      return e;
    }

    double rCore = coreRadiusFor(cellM);
    double rJitter = jitterRadiusFor(cellM);

    e.eta = estimateTravelTime(ourPos, p, cap);

    e.units = dyn.valueAt(p);
    e.count = dyn.countResourcesWithin(p, 0.70);
    e.evidence = dyn.evidenceMassWithin(p, EVIDENCE_R);

    e.depleted = depletedPenaltySoft(p);
    e.localAvoid = dyn.localAvoidPenalty(p, COLLECT_LOCAL_AVOID_R);

    e.activity =
        Math.min(
            ACTIVITY_CAP,
            COLLECT_ACTIVITY_ALLY_W * radialDensity(allyMap, p, COLLECT_ACTIVITY_SIGMA)
                + COLLECT_ACTIVITY_ENEMY_W * radialDensity(enemyMap, p, COLLECT_ACTIVITY_SIGMA)
                + COLLECT_ACTIVITY_DYN_W * dyn.otherDensity(p, COLLECT_ACTIVITY_SIGMA));

    double ei = enemyIntent != null ? enemyIntent.intentAt(p) : 0.0;
    double ai = allyIntent != null ? allyIntent.intentAt(p) : 0.0;
    e.enemyIntent = ei;
    e.allyIntent = ai;

    e.enemyPressure = radialPressure(enemyMap, p, e.eta, 0.0, 0);
    e.allyCongestion = radialCongestion(allyMap, p, e.eta, 0.0, 0);

    e.overlap = reservationOverlapPenalty(allyMap, p, e.eta);

    Translation2d nn = dyn.nearestResourceTo(p, rCore);
    e.coreDist = nn != null ? nn.getDistance(p) : Double.POSITIVE_INFINITY;
    e.coreCount = dyn.countResourcesWithin(p, rCore);

    e.robustPenalty = pickupRobustPenalty(dyn, p, rCore, rJitter);

    double sat = 1.0 - Math.exp(-COLLECT_VALUE_SAT_K * Math.max(0.0, e.units));
    e.value = sat * Math.max(1, goal) * COLLECT_VALUE_GAIN;

    double near = Math.exp(-COLLECT_NEAR_DECAY * e.eta);

    double valueNorm = normalizeValue(dyn, e.value);

    e.score =
        valueNorm
            - e.eta * COLLECT_ETA_COST
            + near * COLLECT_NEAR_BONUS
            - e.enemyPressure * COLLECT_ENEMY_PRESS_COST
            - e.allyCongestion * COLLECT_ALLY_CONGEST_COST
            - e.enemyIntent * COLLECT_ENEMY_INTENT_COST
            - e.allyIntent * COLLECT_ALLY_INTENT_COST
            - e.localAvoid
            - e.activity
            - e.depleted * DEPLETED_PEN_W
            - e.overlap
            - e.robustPenalty;

    return e;
  }

  private double normalizeValue(SpatialDyn dyn, double value) {
    double tot = dyn != null ? dyn.totalEvidence() : 0.0;
    double denom = 0.65 + 0.35 * Math.log(1.0 + Math.max(0.0, tot)) + 0.20 * Math.max(0.0, tot);
    return value / Math.max(0.65, denom);
  }

  private double reservationOverlapPenalty(
      HashMap<Integer, Track> allies, Translation2d p, double ourEtaS) {
    if (allies.isEmpty() || p == null) return 0.0;
    double best = Double.POSITIVE_INFINITY;
    for (Track r : allies.values()) {
      Translation2d pred = predictAt(r, ourEtaS);
      double d = pred.getDistance(p);
      if (d < best) best = d;
    }
    if (best <= 1e-6) return RES_OVERLAP_GAIN * 1.25;
    if (best >= RES_OVERLAP_R) return 0.0;
    double x = (RES_OVERLAP_R - best) / Math.max(1e-6, RES_OVERLAP_R);
    return RES_OVERLAP_GAIN * (0.35 + 0.95 * x * x);
  }

  private boolean shouldEscapeCurrentCollect(
      Translation2d ourPos,
      SpatialDyn dyn,
      double totalEv,
      double minUnits,
      int minCount,
      double cellM) {

    if (currentCollectTarget == null || ourPos == null || dyn == null) return true;

    double rCore = coreRadiusFor(cellM);
    double footprintMinUnits = Math.max(0.025, Math.min(COLLECT_FINE_MIN_UNITS, minUnits * 0.80));
    Rotation2d heading = currentCollectHeading != null ? currentCollectHeading : new Rotation2d();
    if (!footprintOk(dyn, currentCollectTarget, heading, rCore, footprintMinUnits)) return true;

    CollectEval cur =
        evalCollectPoint(
            ourPos,
            lastOurCapForCollect > 0.0 ? lastOurCapForCollect : DEFAULT_OUR_SPEED,
            currentCollectTarget,
            Math.max(1, lastGoalUnitsForCollect),
            cellM,
            dyn,
            null,
            null);

    if (cur.units < Math.max(0.02, minUnits * 0.70)) return true;
    if (cur.count < Math.max(1, minCount - 1)) return true;
    if (cur.depleted > 0.90) return true;
    if (cur.evidence < minEvidence(totalEv) * 0.75) return true;

    double now = Timer.getFPGATimestamp();
    double dist = ourPos.getDistance(currentCollectTarget);

    if (!Double.isFinite(collectProgressLastDist)) {
      collectProgressLastDist = dist;
      collectProgressLastTs = now;
      return false;
    }

    if (dist + 1e-9 < collectProgressLastDist - COLLECT_PROGRESS_MIN_DROP_M) {
      collectProgressLastDist = dist;
      collectProgressLastTs = now;
      return false;
    }

    if (now - collectProgressLastTs > COLLECT_PROGRESS_WINDOW_S && dist > COLLECT_ARRIVE_R + 0.20) {
      return true;
    }

    return false;
  }

  private double collectCommitWindow(double etaCurrent) {
    double x = clamp01((etaCurrent - 0.20) / 1.25);
    return lerp(COLLECT_COMMIT_MIN_S, COLLECT_COMMIT_MAX_S, x);
  }

  private double minEvidence(double totalEvidence) {
    double x = clamp01(totalEvidence / 6.0);
    return lerp(EVIDENCE_MIN_BASE, EVIDENCE_MIN_MAX, x);
  }

  private double dynamicMinUnits(double totalEvidence) {
    double x = clamp01(totalEvidence / 7.5);
    return lerp(0.05, 0.15, x);
  }

  private int dynamicMinCount(double totalEvidence) {
    double x = clamp01(totalEvidence / 7.5);
    return (int) Math.round(lerp(1.0, 3.0, x));
  }

  private void recordRegionAttempt(SpatialDyn dyn, Translation2d p, double now, boolean success) {
    if (dyn == null || p == null) return;
    long k = regionKey(p, 0.30);
    RegionStat st = regionStats.get(k);
    if (st == null) {
      if (regionStats.size() > REGION_STATS_MAX) regionStats.clear();
      st = new RegionStat();
      regionStats.put(k, st);
    }
    st.attempts++;
    if (success) st.successes++;
    st.lastAttemptTs = now;
  }

  private double regionBanditBonus(SpatialDyn dyn, Translation2d p, double now) {
    if (dyn == null || p == null) return 0.0;
    if (regionStats.isEmpty()) return 0.0;

    long k = regionKey(p, 0.30);
    RegionStat st = regionStats.get(k);
    if (st == null) return 0.0;

    int a = Math.max(1, st.attempts);
    double mean = (double) st.successes / a;

    double totalA = 0.0;
    for (RegionStat rs : regionStats.values()) totalA += Math.max(1, rs.attempts);
    totalA = Math.max(1.0, totalA);

    double ucb = mean + 0.65 * Math.sqrt(Math.log(totalA + 1.0) / a);

    double rec = now - st.lastAttemptTs;
    double recW = rec >= 0.0 ? Math.exp(-0.25 * rec) : 1.0;

    return 0.22 * ucb * recW;
  }

  private static long regionKey(Translation2d p, double binM) {
    double inv = 1.0 / Math.max(1e-6, binM);
    int cx = (int) Math.floor(p.getX() * inv);
    int cy = (int) Math.floor(p.getY() * inv);
    return SpatialDyn.key(cx, cy);
  }

  private static double lerp(double a, double b, double t) {
    double x = Math.max(0.0, Math.min(1.0, t));
    return a + (b - a) * x;
  }

  private static double clamp01(double x) {
    return Math.max(0.0, Math.min(1.0, x));
  }

  private static double wallDistance(Translation2d p) {
    if (p == null) return 0.0;
    double dx = Math.min(p.getX(), Constants.FIELD_LENGTH - p.getX());
    double dy = Math.min(p.getY(), Constants.FIELD_WIDTH - p.getY());
    return Math.min(dx, dy);
  }

  private static boolean isInvalidFuelBand(Translation2d p) {
    if (p == null) return false;
    double x = p.getX();
    return X_LEFT_BAND.within(x) || X_RIGHT_BAND.within(x);
  }

  private static final class SpatialDyn {
    static long key(int cx, int cy) {
      return (((long) cx) << 32) ^ (cy & 0xffffffffL);
    }

    final List<DynamicObject> all;
    final List<DynamicObject> resources;
    final List<DynamicObject> others;
    final HashMap<String, ResourceSpec> specs;
    final HashMap<String, Double> otherWeights;

    final double cellM;
    final HashMap<Long, ArrayList<DynamicObject>> resCells;
    final HashMap<Long, ArrayList<DynamicObject>> othCells;

    SpatialDyn(
        List<DynamicObject> dyn,
        HashMap<String, ResourceSpec> specsIn,
        HashMap<String, Double> otherWeightsIn) {
      this.all = dyn != null ? dyn : List.of();
      this.specs = new HashMap<>();
      if (specsIn != null) {
        for (var e : specsIn.entrySet()) {
          if (e.getKey() == null || e.getValue() == null) continue;
          this.specs.put(e.getKey().toLowerCase(), e.getValue());
        }
      }
      this.otherWeights = new HashMap<>();
      if (otherWeightsIn != null) {
        for (var e : otherWeightsIn.entrySet()) {
          if (e.getKey() == null || e.getKey().isEmpty()) continue;
          this.otherWeights.put(e.getKey().toLowerCase(), Math.max(0.0, e.getValue()));
        }
      }

      this.cellM = 0.50;
      this.resCells = new HashMap<>(128);
      this.othCells = new HashMap<>(128);

      ArrayList<DynamicObject> res = new ArrayList<>();
      ArrayList<DynamicObject> oth = new ArrayList<>();

      for (DynamicObject o : this.all) {
        if (o == null || o.pos == null) continue;
        if (!error() && o.ageS > RESOURCE_HARD_MAX_AGE_S) continue;

        String ty = o.type != null ? o.type.toLowerCase() : "unknown";
        if ("fuel".equals(ty) && isInvalidFuelBand(o.pos)) continue;
        if (this.specs.containsKey(ty)) {
          res.add(o);
          addTo(resCells, o.pos, o);
        } else {
          oth.add(o);
          addTo(othCells, o.pos, o);
        }
      }

      this.resources = res;
      this.others = oth;
    }

    private void addTo(
        HashMap<Long, ArrayList<DynamicObject>> map, Translation2d p, DynamicObject o) {
      int cx = (int) Math.floor(p.getX() / Math.max(1e-6, cellM));
      int cy = (int) Math.floor(p.getY() / Math.max(1e-6, cellM));
      long k = key(cx, cy);
      map.computeIfAbsent(k, kk -> new ArrayList<>()).add(o);
    }

    private Iterable<ArrayList<DynamicObject>> cellsInRadius(
        Translation2d p, double r, HashMap<Long, ArrayList<DynamicObject>> map) {
      int cx0 = (int) Math.floor(p.getX() / Math.max(1e-6, cellM));
      int cy0 = (int) Math.floor(p.getY() / Math.max(1e-6, cellM));
      int rad = (int) Math.ceil(r / Math.max(1e-6, cellM));
      ArrayList<ArrayList<DynamicObject>> out = new ArrayList<>((2 * rad + 1) * (2 * rad + 1));
      for (int dx = -rad; dx <= rad; dx++) {
        for (int dy = -rad; dy <= rad; dy++) {
          ArrayList<DynamicObject> c = map.get(key(cx0 + dx, cy0 + dy));
          if (c != null) out.add(c);
        }
      }
      return out;
    }

    int countResourcesWithin(Translation2d p, double r) {
      if (p == null) return 0;
      double rr2 = r * r;
      int c = 0;
      for (ArrayList<DynamicObject> cell : cellsInRadius(p, r, resCells)) {
        for (int i = 0; i < cell.size(); i++) {
          DynamicObject o = cell.get(i);
          if (o == null || o.pos == null) continue;
          double dx = o.pos.getX() - p.getX();
          double dy = o.pos.getY() - p.getY();
          if (dx * dx + dy * dy <= rr2) c++;
        }
      }
      return c;
    }

    double totalEvidence() {
      if (resources.isEmpty() || specs.isEmpty()) return 0.0;
      double sum = 0.0;
      for (DynamicObject o : resources) {
        if (o == null || o.pos == null || o.type == null) continue;
        ResourceSpec s = specs.get(o.type.toLowerCase());
        if (s == null) continue;
        double ageW = error() ? 1.0 : Math.exp(-COLLECT_AGE_DECAY * Math.max(0.0, o.ageS));
        sum += Math.max(0.0, s.unitValue) * ageW;
      }
      return Math.max(0.0, sum);
    }

    double evidenceMassWithin(Translation2d p, double r) {
      if (p == null || resources.isEmpty() || specs.isEmpty()) return 0.0;
      double rr2 = r * r;
      double sum = 0.0;
      for (ArrayList<DynamicObject> cell : cellsInRadius(p, r, resCells)) {
        for (int i = 0; i < cell.size(); i++) {
          DynamicObject o = cell.get(i);
          if (o == null || o.pos == null || o.type == null) continue;
          double dx = o.pos.getX() - p.getX();
          double dy = o.pos.getY() - p.getY();
          double d2 = dx * dx + dy * dy;
          if (d2 > rr2) continue;

          ResourceSpec s = specs.get(o.type.toLowerCase());
          if (s == null) continue;

          double ageW = error() ? 1.0 : Math.exp(-COLLECT_AGE_DECAY * Math.max(0.0, o.ageS));
          sum += Math.max(0.0, s.unitValue) * ageW;
        }
      }
      return Math.max(0.0, sum);
    }

    double valueAt(Translation2d p) {
      if (p == null || resources.isEmpty() || specs.isEmpty()) return 0.0;

      double r = 2.0;
      double sum = 0.0;

      for (ArrayList<DynamicObject> cell : cellsInRadius(p, r, resCells)) {
        for (int i = 0; i < cell.size(); i++) {
          DynamicObject o = cell.get(i);
          if (o == null || o.pos == null || o.type == null) continue;

          ResourceSpec s = specs.get(o.type.toLowerCase());
          if (s == null) continue;

          double dx = o.pos.getX() - p.getX();
          double dy = o.pos.getY() - p.getY();
          double d2 = dx * dx + dy * dy;

          double age = Math.max(0.0, o.ageS);
          double ageW = error() ? 1.0 : Math.exp(-COLLECT_AGE_DECAY * age);

          double sigmaBase =
              Math.max(
                  RESOURCE_SIGMA_MIN,
                  Math.min(
                      RESOURCE_SIGMA_ABS_MAX, Math.min(s.sigmaM * RESOURCE_SIGMA_REL_MAX, 2.0)));

          double sigma = sigmaBase * (0.70 + 0.35 * Math.exp(-0.85 * age));
          sigma = Math.max(RESOURCE_SIGMA_MIN, Math.min(RESOURCE_SIGMA_ABS_MAX, sigma));

          double sig2 = sigma * sigma;

          double w = Math.max(0.0, s.unitValue) * ageW;
          double k = Math.exp(-0.5 * d2 / Math.max(1e-6, sig2));

          sum += w * k;
        }
      }

      return Math.max(0.0, sum);
    }

    double valueInSquare(Translation2d center, double half) {
      if (center == null || resources.isEmpty() || specs.isEmpty()) return 0.0;
      double x0 = center.getX() - half;
      double x1 = center.getX() + half;
      double y0 = center.getY() - half;
      double y1 = center.getY() + half;

      double r = Math.sqrt(2.0) * half + 0.20;
      double sum = 0.0;

      for (ArrayList<DynamicObject> cell : cellsInRadius(center, r, resCells)) {
        for (int i = 0; i < cell.size(); i++) {
          DynamicObject o = cell.get(i);
          if (o == null || o.pos == null || o.type == null) continue;
          double x = o.pos.getX();
          double y = o.pos.getY();
          if (x < x0 || x > x1 || y < y0 || y > y1) continue;

          ResourceSpec s = specs.get(o.type.toLowerCase());
          if (s == null) continue;

          double ageW = error() ? 1.0 : Math.exp(-COLLECT_AGE_DECAY * Math.max(0.0, o.ageS));
          sum += Math.max(0.0, s.unitValue) * ageW;
        }
      }

      return Math.max(0.0, sum);
    }

    Translation2d centroidResourcesWithin(Translation2d seed, double r, double minMass) {
      if (seed == null || resources.isEmpty() || specs.isEmpty()) return null;

      double rr2 = r * r;
      double sx = 0.0;
      double sy = 0.0;
      double sw = 0.0;

      for (ArrayList<DynamicObject> cell : cellsInRadius(seed, r, resCells)) {
        for (int i = 0; i < cell.size(); i++) {
          DynamicObject o = cell.get(i);
          if (o == null || o.pos == null || o.type == null) continue;

          ResourceSpec s = specs.get(o.type.toLowerCase());
          if (s == null) continue;

          double dx = o.pos.getX() - seed.getX();
          double dy = o.pos.getY() - seed.getY();
          double d2 = dx * dx + dy * dy;
          if (d2 > rr2) continue;

          double age = Math.max(0.0, o.ageS);
          double ageW = error() ? 1.0 : Math.exp(-COLLECT_AGE_DECAY * age);

          double sigmaBase =
              Math.max(RESOURCE_SIGMA_MIN, Math.min(RESOURCE_SIGMA_ABS_MAX, s.sigmaM));
          double sigma = sigmaBase * (0.70 + 0.35 * Math.exp(-0.85 * age));
          sigma = Math.max(RESOURCE_SIGMA_MIN, Math.min(RESOURCE_SIGMA_ABS_MAX, sigma));

          double sig2 = sigma * sigma;

          double w = Math.max(0.0, s.unitValue) * ageW * Math.exp(-0.5 * d2 / Math.max(1e-6, sig2));

          sx += o.pos.getX() * w;
          sy += o.pos.getY() * w;
          sw += w;
        }
      }

      if (sw < Math.max(0.0, minMass)) return null;
      return new Translation2d(sx / sw, sy / sw);
    }

    Translation2d nearestResourceTo(Translation2d p, double maxDist) {
      if (p == null || resources.isEmpty()) return null;
      double bestD2 = maxDist * maxDist;
      Translation2d best = null;

      for (ArrayList<DynamicObject> cell : cellsInRadius(p, maxDist, resCells)) {
        for (int i = 0; i < cell.size(); i++) {
          DynamicObject o = cell.get(i);
          if (o == null || o.pos == null) continue;
          double dx = o.pos.getX() - p.getX();
          double dy = o.pos.getY() - p.getY();
          double d2 = dx * dx + dy * dy;
          if (d2 <= bestD2) {
            bestD2 = d2;
            best = o.pos;
          }
        }
      }

      return best;
    }

    double otherDensity(Translation2d p, double sigma) {
      if (p == null || others.isEmpty()) return 0.0;
      double s = Math.max(0.05, sigma);
      double r = 3.0 * s;
      double agg = 0.0;
      double wSum = 0.0;

      for (ArrayList<DynamicObject> cell : cellsInRadius(p, r, othCells)) {
        for (int i = 0; i < cell.size(); i++) {
          DynamicObject o = cell.get(i);
          if (o == null || o.pos == null) continue;
          double d = o.pos.getDistance(p);

          double w = 1.0;
          if (o.type != null) {
            Double ow = otherWeights.get(o.type.toLowerCase());
            if (ow != null) w = Math.max(0.0, ow);
          }

          agg += w * densityKernel(d, s);
          wSum += Math.max(0.05, w);
        }
      }

      if (wSum <= 1e-9) return 0.0;
      return Math.max(0.0, agg / wSum);
    }

    double localAvoidPenalty(Translation2d p, double r) {
      if (p == null || others.isEmpty()) return 0.0;
      double rr = Math.max(0.05, r);
      double rr2 = rr * rr;

      double pen = 0.0;
      double wSum = 0.0;

      for (ArrayList<DynamicObject> cell : cellsInRadius(p, rr, othCells)) {
        for (int i = 0; i < cell.size(); i++) {
          DynamicObject o = cell.get(i);
          if (o == null || o.pos == null) continue;
          double dx = o.pos.getX() - p.getX();
          double dy = o.pos.getY() - p.getY();
          double d2 = dx * dx + dy * dy;
          if (d2 > rr2) continue;
          double d = Math.sqrt(Math.max(0.0, d2));

          double w = 1.0;
          if (o.type != null) {
            Double ow = otherWeights.get(o.type.toLowerCase());
            if (ow != null) w = Math.max(0.0, ow);
          }

          double k = Math.exp(-0.5 * (d * d) / Math.max(1e-6, (0.45 * 0.45)));
          pen += w * k;
          wSum += Math.max(0.05, w);
        }
      }

      if (wSum <= 1e-9) return 0.0;
      return Math.min(1.25, pen / wSum);
    }
  }
}
