// File: src/main/java/org/curtinfrc/frc2026/util/Repulsor/PredictiveFieldState.java
package org.curtinfrc.frc2026.util.Repulsor;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
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

  public void markCollectDepleted(Translation2d p, double cellM, double strength) {
    markDepleted(p, cellM, strength);
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

    public PointCandidate(
        Translation2d point,
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

  private static final double COLLECT_VALUE_GAIN = 2.10;
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
  private static final double COLLECT_REGION_SAMPLES_W = 0.70;
  private static final double COLLECT_CELL_M = 0.20;
  private static final double COLLECT_NEAR_BONUS = 0.55;
  private static final double COLLECT_NEAR_DECAY = 1.35;

  private static final double COLLECT_COARSE_MIN_REGION_UNITS = 0.12;
  private static final double COLLECT_FINE_MIN_UNITS = 0.08;

  private static final double RESOURCE_SIGMA_ABS_MAX = 0.45;
  private static final double RESOURCE_SIGMA_REL_MAX = 1.25;
  private static final double RESOURCE_SIGMA_MIN = 0.06;

  private static final int COLLECT_CLUSTER_MAX = 72;
  private static final double COLLECT_CLUSTER_BIN_M = 0.14;
  private static final int COLLECT_GRID_FALLBACK_MAX = 1400;
  private static final double COLLECT_DUP_SKIP_M = 0.30;

  private static final double DEPLETED_TTL_S = 3.25;
  private static final double DEPLETED_DECAY = 1.15;
  private static final double DEPLETED_PEN_W = 1.75;
  private static final double DEPLETED_MARK_NEAR_M = 0.65;
  private static final double DEPLETED_MARK_EMPTY_UNITS = 0.07;

  private static final double RESOURCE_HARD_MAX_AGE_S = 0.95;

  private double[] scratchLogits = new double[0];
  private double[] scratchLogits2 = new double[0];

  private volatile List<DynamicObject> lastDynRef = null;
  private volatile SpatialDyn lastDyn = null;
  private volatile int specsVersion = 0;
  private volatile int lastDynSpecsVersion = -1;

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
    SpatialDyn nd = new SpatialDyn(ref, resourceSpecs);
    lastDynRef = ref;
    lastDyn = nd;
    lastDynSpecsVersion = sv;
    return nd;
  }

  private final HashMap<Integer, Track> allyMap = new HashMap<>();
  private final HashMap<Integer, Track> enemyMap = new HashMap<>();

  private List<GameElement> worldElements = List.of();
  private Alliance ourAlliance = Alliance.kBlue;

  private RepulsorSetpoint lastChosen = null;
  private double lastChosenTs = 0.0;

  private volatile List<DynamicObject> dynamicObjects = List.of();
  private final HashMap<String, ResourceSpec> resourceSpecs = new HashMap<>();

  private static final class Depleted {
    double t;
    double s;

    Depleted(double t, double s) {
      this.t = t;
      this.s = s;
    }
  }

  private final HashMap<Long, Depleted> depleted = new HashMap<>(256);
  private Translation2d lastReturnedCollect = null;
  private double lastReturnedCollectTs = 0.0;

  public void registerResourceSpec(String type, ResourceSpec spec) {
    if (type == null || type.isEmpty() || spec == null) return;
    resourceSpecs.put(type.toLowerCase(), spec);
    specsVersion++;
  }

  public void setDynamicObjects(List<DynamicObject> objs) {
    dynamicObjects = objs != null ? objs : List.of();
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

      double ourEta = etaPath(ourPos, t, cap);
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

    double seedU = dyn.valueAt(seed);
    double centU = dyn.valueAt(c);

    int seedC = dyn.countResourcesWithin(seed, Math.max(0.05, r));
    int centC = dyn.countResourcesWithin(c, Math.max(0.05, r));

    if (centU + 1e-9 < seedU - 0.05) return seed;
    if (centC + 1 < seedC) return seed;
    return c;
  }

  public Translation2d nearestCollectResource(Translation2d p, double maxDist) {
    if (p == null) return null;
    SpatialDyn d = cachedDyn();
    if (d == null) return null;
    return d.nearestResourceTo(p, Math.max(0.01, maxDist));
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
    Translation2d[] seeds = buildCollectCandidates(points, dyn);
    if (seeds.length == 0) return null;

    sweepDepleted();

    double cap = ourSpeedCap > 0.0 ? ourSpeedCap : DEFAULT_OUR_SPEED;

    double hardCountR = 0.60;
    int hardMinCount = 2;

    double snapR = 1.10;
    double snapMinMass = 0.18;

    Translation2d[] targets = new Translation2d[seeds.length];
    for (int i = 0; i < seeds.length; i++) {
      Translation2d s = seeds[i];
      Translation2d c = dyn != null ? dyn.centroidResourcesWithin(s, snapR, snapMinMass) : null;
      targets[i] = c != null ? c : s;
    }

    int bestI = -1;
    double bestEta = Double.POSITIVE_INFINITY;
    double bestUnits = 0.0;

    int maxCheck = limit > 0 ? Math.min(limit, targets.length) : targets.length;

    double[] etas = new double[targets.length];
    int[] order = new int[targets.length];
    for (int i = 0; i < targets.length; i++) {
      order[i] = i;
      etas[i] = etaPath(ourPos, targets[i], cap);
    }

    sortIdxByKey(etas, order);

    int checked = 0;
    for (int oi = 0; oi < order.length && checked < maxCheck; oi++) {
      int i = order[oi];
      Translation2d t = targets[i];
      checked++;

      if (dyn.countResourcesWithin(t, hardCountR) < hardMinCount) continue;

      double units = dyn.valueAt(t);
      if (units < COLLECT_FINE_MIN_UNITS) continue;

      double dep = depletedPenalty(t, cellM);
      if (dep > 0.45) continue;

      double eta = etas[i];

      if (eta < bestEta - 1e-9) {
        bestEta = eta;
        bestI = i;
        bestUnits = units;
      } else if (Math.abs(eta - bestEta) <= 0.04 && units > bestUnits + 0.08) {
        bestEta = eta;
        bestI = i;
        bestUnits = units;
      }
    }

    if (bestI < 0) return null;

    Translation2d chosen = targets[bestI];
    int count = dyn.countResourcesWithin(chosen, hardCountR);
    double units = dyn.valueAt(chosen);

    if (count < 1 || units < Math.max(0.02, COLLECT_FINE_MIN_UNITS * 0.75)) {
      markDepleted(chosen, cellM, 1.0);
      return null;
    }

    double sat = 1.0 - Math.exp(-COLLECT_VALUE_SAT_K * Math.max(0.0, units));
    double value = sat * Math.max(1, goalUnits) * COLLECT_VALUE_GAIN;

    lastReturnedCollect = chosen;
    lastReturnedCollectTs = Timer.getFPGATimestamp();

    Logger.recordOutput("Repulsor/ChosenCollect", new Pose2d(chosen, new Rotation2d()));
    Logger.recordOutput("Repulsor/Collect/ChosenFuelUnits", units);
    Logger.recordOutput("Repulsor/Collect/ChosenCount", count);

    return new PointCandidate(chosen, bestEta, value, 0.0, 0.0, 0.0, 0.0, -bestEta);
  }

  public PointCandidate rankCollectHierarchical(
      Translation2d ourPos,
      double ourSpeedCap,
      Translation2d[] points,
      double cellM,
      int goalUnits,
      int coarseTopK,
      int refineGrid) {

    double hardCountR = 0.75;
    int hardMinCountCoarse = 1;
    int hardMinCountFine = 1;
    double snapR = 0.85;
    double snapMinMass = 0.20;

    if (ourPos == null) return null;

    SpatialDyn dyn = cachedDyn();

    Translation2d[] targets = buildCollectCandidates(points, dyn);
    if (targets.length == 0) return null;

    IntentAgg allyAgg = softIntentAggArr(allyMap, targets);
    IntentAgg enemyAgg = softIntentAggArr(enemyMap, targets);

    double cap = ourSpeedCap > 0.0 ? ourSpeedCap : DEFAULT_OUR_SPEED;
    int goal = Math.max(1, goalUnits);
    int topK = Math.max(1, Math.min(coarseTopK, Math.max(2, targets.length)));
    int rg = Math.max(2, refineGrid);

    double half = Math.max(0.05, cellM * 0.5);

    sweepDepleted();

    if (lastReturnedCollect != null) {
      double now = Timer.getFPGATimestamp();
      double age = now - lastReturnedCollectTs;
      if (age >= 0.0
          && age <= 0.55
          && ourPos.getDistance(lastReturnedCollect) <= DEPLETED_MARK_NEAR_M) {
        double u = dyn.valueAt(lastReturnedCollect);
        if (u < DEPLETED_MARK_EMPTY_UNITS) markDepleted(lastReturnedCollect, cellM, 1.0);
      }
    }

    int[] bestIdx = new int[topK];
    double[] bestScore = new double[topK];
    for (int i = 0; i < topK; i++) {
      bestIdx[i] = -1;
      bestScore[i] = -1e18;
    }

    int fallbackIdx = -1;
    double fallbackUnits = -1e18;

    for (int i = 0; i < targets.length; i++) {
      Translation2d cpt = targets[i];

      double regionUnitsAny = dyn.valueInSquare(cpt, half);
      if (regionUnitsAny > fallbackUnits) {
        fallbackUnits = regionUnitsAny;
        fallbackIdx = i;
      }

      if (dyn.countResourcesWithin(cpt, hardCountR) < hardMinCountCoarse) continue;

      double eta = etaPath(ourPos, cpt, cap);
      double near = Math.exp(-COLLECT_NEAR_DECAY * eta);

      double regionUnits = regionUnitsAny;
      if (regionUnits < COLLECT_COARSE_MIN_REGION_UNITS) continue;

      double sat = 1.0 - Math.exp(-COLLECT_VALUE_SAT_K * Math.max(0.0, regionUnits));
      double value = sat * goal * COLLECT_VALUE_GAIN;

      double enemyP = radialPressure(enemyMap, cpt, eta, enemyAgg.intent[i], enemyAgg.count);
      double allyC = radialCongestion(allyMap, cpt, eta, allyAgg.intent[i], allyAgg.count);

      double ei = enemyAgg.count > 0 ? enemyAgg.intent[i] / Math.max(1.0, enemyAgg.count) : 0.0;
      double ai = allyAgg.count > 0 ? allyAgg.intent[i] / Math.max(1.0, allyAgg.count) : 0.0;

      double activity =
          COLLECT_ACTIVITY_ALLY_W * radialDensity(allyMap, cpt, COLLECT_ACTIVITY_SIGMA)
              + COLLECT_ACTIVITY_ENEMY_W * radialDensity(enemyMap, cpt, COLLECT_ACTIVITY_SIGMA)
              + COLLECT_ACTIVITY_DYN_W * dyn.otherDensity(cpt, COLLECT_ACTIVITY_SIGMA);

      double dep = depletedPenalty(cpt, cellM);

      double score =
          value * COLLECT_REGION_SAMPLES_W
              - eta * COLLECT_ETA_COST
              + near * COLLECT_NEAR_BONUS
              - enemyP * COLLECT_ENEMY_PRESS_COST
              - allyC * COLLECT_ALLY_CONGEST_COST
              - ei * COLLECT_ENEMY_INTENT_COST
              - ai * COLLECT_ALLY_INTENT_COST
              - activity
              - dep * DEPLETED_PEN_W;

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

    double step = (2.0 * half) / (rg - 1);

    double[] candScore = new double[MAX_TRIES];
    Translation2d[] candPt = new Translation2d[MAX_TRIES];
    double[] candEta = new double[MAX_TRIES];
    double[] candVal = new double[MAX_TRIES];
    double[] candEP = new double[MAX_TRIES];
    double[] candAC = new double[MAX_TRIES];
    double[] candEI = new double[MAX_TRIES];
    double[] candAI = new double[MAX_TRIES];
    int candN = 0;

    for (int k = 0; k < topK; k++) {
      int idx = bestIdx[k];
      if (idx < 0) continue;

      Translation2d center = targets[idx];

      double eiC = enemyAgg.count > 0 ? enemyAgg.intent[idx] / Math.max(1.0, enemyAgg.count) : 0.0;
      double aiC = allyAgg.count > 0 ? allyAgg.intent[idx] / Math.max(1.0, allyAgg.count) : 0.0;

      for (int ix = 0; ix < rg; ix++) {
        double ox = -half + ix * step;
        for (int iy = 0; iy < rg; iy++) {
          double oy = -half + iy * step;

          Translation2d p = new Translation2d(center.getX() + ox, center.getY() + oy);

          if (dyn.countResourcesWithin(p, hardCountR) < hardMinCountFine) continue;

          double eta = etaPath(ourPos, p, cap);
          double near = Math.exp(-COLLECT_NEAR_DECAY * eta);

          double rawUnits = dyn.valueAt(p);
          if (rawUnits < COLLECT_FINE_MIN_UNITS) continue;

          double sat = 1.0 - Math.exp(-COLLECT_VALUE_SAT_K * Math.max(0.0, rawUnits));
          double value = sat * goal * COLLECT_VALUE_GAIN;

          double enemyP = radialPressure(enemyMap, p, eta, enemyAgg.intent[idx], enemyAgg.count);
          double allyC = radialCongestion(allyMap, p, eta, allyAgg.intent[idx], allyAgg.count);

          double localAvoid = dyn.localAvoidPenalty(p, COLLECT_LOCAL_AVOID_R);

          double activity =
              COLLECT_ACTIVITY_ALLY_W * radialDensity(allyMap, p, COLLECT_ACTIVITY_SIGMA)
                  + COLLECT_ACTIVITY_ENEMY_W * radialDensity(enemyMap, p, COLLECT_ACTIVITY_SIGMA)
                  + COLLECT_ACTIVITY_DYN_W * dyn.otherDensity(p, COLLECT_ACTIVITY_SIGMA);

          double dep = depletedPenalty(p, cellM);

          double score =
              value
                  - eta * COLLECT_ETA_COST
                  + near * COLLECT_NEAR_BONUS
                  - enemyP * COLLECT_ENEMY_PRESS_COST
                  - allyC * COLLECT_ALLY_CONGEST_COST
                  - eiC * COLLECT_ENEMY_INTENT_COST
                  - aiC * COLLECT_ALLY_INTENT_COST
                  - localAvoid
                  - activity
                  - dep * DEPLETED_PEN_W;

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
            if (score > worstS) insert = worstI;
          }

          if (insert >= 0) {
            candScore[insert] = score;
            candPt[insert] = p;
            candEta[insert] = eta;
            candVal[insert] = value;
            candEP[insert] = enemyP;
            candAC[insert] = allyC;
            candEI[insert] = eiC;
            candAI[insert] = aiC;
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

    for (int oi = 0; oi < candN; oi++) {
      int ci = order[oi];
      Translation2d p = candPt[ci];
      if (p == null) continue;

      Translation2d snapped = dyn.centroidResourcesWithin(p, snapR, snapMinMass);
      Translation2d use = snapped != null ? snapped : p;

      double pU = dyn.valueAt(p);
      double uU = dyn.valueAt(use);
      int pC = dyn.countResourcesWithin(p, hardCountR);
      int uC = dyn.countResourcesWithin(use, hardCountR);

      if (snapped != null) {
        if (uU + 1e-9 < pU - 0.05) use = p;
        else if (uC + 1 < pC) use = p;
      }

      int finalCount = dyn.countResourcesWithin(use, hardCountR);
      double finalUnits = dyn.valueAt(use);

      Logger.recordOutput("Repulsor/Collect/ChosenFuelUnits", finalUnits);
      Logger.recordOutput("Repulsor/Collect/ChosenScore", candScore[ci]);
      Logger.recordOutput("Repulsor/Collect/ChosenCount", finalCount);

      if (finalCount >= 1 && finalUnits >= Math.max(0.02, COLLECT_FINE_MIN_UNITS * 0.75)) {
        lastReturnedCollect = use;
        lastReturnedCollectTs = Timer.getFPGATimestamp();

        if (ourPos.getDistance(use) <= DEPLETED_MARK_NEAR_M) {
          double u = dyn.valueAt(use);
          if (u < DEPLETED_MARK_EMPTY_UNITS) markDepleted(use, cellM, 1.0);
        }

        Logger.recordOutput("Repulsor/ChosenCollect", new Pose2d(use, new Rotation2d()));

        return new PointCandidate(
            use,
            candEta[ci],
            candVal[ci],
            candEP[ci],
            candAC[ci],
            candEI[ci],
            candAI[ci],
            candScore[ci]);
      }

      markDepleted(use, cellM, 1.0);
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
    Translation2d ourPos,
    double ourSpeedCap,
    Translation2d[] points,
    int goalUnits,
    int limit) {

  if (ourPos == null) return null;

  SpatialDyn dyn = cachedDyn();
  Translation2d[] targets = buildCollectCandidates(points, dyn);
  if (targets == null || targets.length == 0) return null;

  sweepDepleted();

  IntentAgg allyAgg = softIntentAggArr(allyMap, targets);
  IntentAgg enemyAgg = softIntentAggArr(enemyMap, targets);

  double cap = ourSpeedCap > 0.0 ? ourSpeedCap : DEFAULT_OUR_SPEED;
  int goal = Math.max(1, goalUnits);

  double hardCountR = 0.70;
  int hardMinCount = 1;

  int maxCheck = limit > 0 ? Math.min(Math.max(1, limit), targets.length) : targets.length;

  double[] etas = new double[targets.length];
  int[] order = new int[targets.length];
  for (int i = 0; i < targets.length; i++) {
    order[i] = i;
    Translation2d t = targets[i];
    etas[i] = t != null ? etaPath(ourPos, t, cap) : Double.POSITIVE_INFINITY;
  }

  sortIdxByKey(etas, order);

  int bestI = -1;
  double bestScore = -1e18;

  for (int oi = 0; oi < order.length && oi < maxCheck; oi++) {
    int i = order[oi];
    Translation2d p = targets[i];
    if (p == null) continue;

    if (dyn == null) continue;

    int count = dyn.countResourcesWithin(p, hardCountR);
    if (count < hardMinCount) continue;

    double units = dyn.valueAt(p);
    if (units < COLLECT_FINE_MIN_UNITS) continue;

    double dep = depletedPenalty(p, COLLECT_CELL_M);
    if (dep > 0.55) continue;

    double eta = etas[i];
    double near = Math.exp(-COLLECT_NEAR_DECAY * eta);

    double sat = 1.0 - Math.exp(-COLLECT_VALUE_SAT_K * Math.max(0.0, units));
    double value = sat * goal * COLLECT_VALUE_GAIN;

    double enemyP = radialPressure(enemyMap, p, eta, enemyAgg.intent[i], enemyAgg.count);
    double allyC = radialCongestion(allyMap, p, eta, allyAgg.intent[i], allyAgg.count);

    double ei = enemyAgg.count > 0 ? enemyAgg.intent[i] / Math.max(1.0, enemyAgg.count) : 0.0;
    double ai = allyAgg.count > 0 ? allyAgg.intent[i] / Math.max(1.0, allyAgg.count) : 0.0;

    double activity =
        COLLECT_ACTIVITY_ALLY_W * radialDensity(allyMap, p, COLLECT_ACTIVITY_SIGMA)
            + COLLECT_ACTIVITY_ENEMY_W * radialDensity(enemyMap, p, COLLECT_ACTIVITY_SIGMA)
            + COLLECT_ACTIVITY_DYN_W * dyn.otherDensity(p, COLLECT_ACTIVITY_SIGMA);

    double score =
        value
            - eta * COLLECT_ETA_COST
            + near * COLLECT_NEAR_BONUS
            - enemyP * COLLECT_ENEMY_PRESS_COST
            - allyC * COLLECT_ALLY_CONGEST_COST
            - ei * COLLECT_ENEMY_INTENT_COST
            - ai * COLLECT_ALLY_INTENT_COST
            - activity
            - dep * DEPLETED_PEN_W;

    if (score > bestScore) {
      bestScore = score;
      bestI = i;
    }
  }

  if (bestI < 0) return null;

  Translation2d chosen = targets[bestI];
  int count = dyn.countResourcesWithin(chosen, hardCountR);
  double units = dyn.valueAt(chosen);

  if (count < 1 || units < Math.max(0.02, COLLECT_FINE_MIN_UNITS * 0.75)) {
    markDepleted(chosen, COLLECT_CELL_M, 1.0);
    return null;
  }

  double eta = etaPath(ourPos, chosen, cap);
  double sat = 1.0 - Math.exp(-COLLECT_VALUE_SAT_K * Math.max(0.0, units));
  double value = sat * goal * COLLECT_VALUE_GAIN;

  lastReturnedCollect = chosen;
  lastReturnedCollectTs = Timer.getFPGATimestamp();

  Logger.recordOutput("Repulsor/ChosenCollect", new Pose2d(chosen, new Rotation2d()));
  Logger.recordOutput("Repulsor/Collect/ChosenFuelUnits", units);
  Logger.recordOutput("Repulsor/Collect/ChosenCount", count);
  Logger.recordOutput("Repulsor/Collect/ChosenScore", bestScore);

  return new PointCandidate(chosen, eta, value, 0.0, 0.0, 0.0, 0.0, bestScore);
}

  private Translation2d[] buildCollectCandidates(Translation2d[] gridPoints, SpatialDyn dyn) {
    Translation2d[] clusters =
        buildResourceClusters(dyn, COLLECT_CLUSTER_MAX, COLLECT_CLUSTER_BIN_M);

    if (gridPoints == null || gridPoints.length == 0) return clusters;

    int nGrid = 0;
    for (Translation2d p : gridPoints) if (p != null) nGrid++;
    if (nGrid == 0) return clusters;

    int take = Math.min(nGrid, COLLECT_GRID_FALLBACK_MAX);
    int stride = (int) Math.max(1.0, Math.floor((double) nGrid / Math.max(1, take)));

    ArrayList<Translation2d> out = new ArrayList<>(clusters.length + take);
    for (Translation2d c : clusters) if (c != null) out.add(c);

    double dup2 = COLLECT_DUP_SKIP_M * COLLECT_DUP_SKIP_M;

    double minKeepUnits = Math.max(COLLECT_FINE_MIN_UNITS * 0.5, 0.04);
    double countR = 0.55;

    int added = 0;
    int seen = 0;
    for (int i = 0; i < gridPoints.length && added < take; i++) {
      Translation2d p = gridPoints[i];
      if (p == null) continue;
      if ((seen++ % stride) != 0) continue;

      if (dyn.valueAt(p) < minKeepUnits) continue;
      if (dyn.countResourcesWithin(p, countR) < 1) continue;

      boolean dup = false;
      for (int j = 0; j < out.size(); j++) {
        Translation2d c = out.get(j);
        if (c == null) continue;
        double dx = c.getX() - p.getX();
        double dy = c.getY() - p.getY();
        if (dx * dx + dy * dy <= dup2) {
          dup = true;
          break;
        }
      }
      if (!dup) {
        out.add(p);
        added++;
      }
    }

    return out.toArray(new Translation2d[0]);
  }

  private static final class ClusterAcc {
    double sx;
    double sy;
    double sw;
    int n;
  }

  private Translation2d[] buildResourceClusters(SpatialDyn dyn, int maxClusters, double binM) {
    if (dyn == null || dyn.resources.isEmpty() || dyn.specs.isEmpty()) return new Translation2d[0];

    double inv = 1.0 / Math.max(1e-6, binM);
    HashMap<Long, ClusterAcc> acc = new HashMap<>(128);

    for (DynamicObject o : dyn.resources) {
      if (o == null || o.pos == null || o.type == null) continue;
      ResourceSpec s = dyn.specs.get(o.type.toLowerCase());
      if (s == null) continue;

      int cx = (int) Math.floor(o.pos.getX() * inv);
      int cy = (int) Math.floor(o.pos.getY() * inv);
      long k = SpatialDyn.key(cx, cy);

      ClusterAcc a = acc.get(k);
      if (a == null) {
        a = new ClusterAcc();
        acc.put(k, a);
      }

      double ageW =
          RobotBase.isSimulation() ? 1.0 : Math.exp(-COLLECT_AGE_DECAY * Math.max(0.0, o.ageS));
      double w = Math.max(0.0, s.unitValue) * ageW;

      a.sx += o.pos.getX() * w;
      a.sy += o.pos.getY() * w;
      a.sw += w;
      a.n++;
    }

    if (acc.isEmpty()) return new Translation2d[0];

    ArrayList<long[]> scored = new ArrayList<>(acc.size());
    for (var e : acc.entrySet()) {
      ClusterAcc a = e.getValue();
      if (a == null) continue;
      if (a.sw <= 1e-9) continue;
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
    Translation2d[] out = new Translation2d[m];

    for (int i = 0; i < m; i++) {
      long key = scored.get(i)[0];
      ClusterAcc a = acc.get(key);
      if (a == null || a.sw <= 1e-9) continue;
      out[i] = new Translation2d(a.sx / a.sw, a.sy / a.sw);
    }

    int nn = 0;
    for (Translation2d t : out) if (t != null) nn++;
    if (nn == out.length) return out;

    Translation2d[] compact = new Translation2d[nn];
    int ti = 0;
    for (Translation2d t : out) if (t != null) compact[ti++] = t;
    return compact;
  }

  private void sweepDepleted() {
    if (RobotBase.isSimulation()) return;

    if (depleted.isEmpty()) return;
    double now = Timer.getFPGATimestamp();
    depleted.entrySet().removeIf(e -> now - e.getValue().t > DEPLETED_TTL_S);
  }

  private void markDepleted(Translation2d p, double cellM, double strength) {
    if (RobotBase.isSimulation()) return;

    if (p == null) return;
    double now = Timer.getFPGATimestamp();
    long k = depletedKey(p, cellM);
    Depleted d = depleted.get(k);
    if (d == null) {
      depleted.put(k, new Depleted(now, Math.max(0.0, strength)));
    } else {
      d.t = now;
      d.s = Math.max(d.s, Math.max(0.0, strength));
    }
  }

  private double depletedPenalty(Translation2d p, double cellM) {
    if (RobotBase.isSimulation()) return 0.0;

    if (p == null || depleted.isEmpty()) return 0.0;
    long k = depletedKey(p, cellM);
    Depleted d = depleted.get(k);
    if (d == null) return 0.0;
    double now = Timer.getFPGATimestamp();
    double age = Math.max(0.0, now - d.t);
    if (age > DEPLETED_TTL_S) return 0.0;
    return d.s * Math.exp(-DEPLETED_DECAY * age);
  }

  private static long depletedKey(Translation2d p, double cellM) {
    double inv = 1.0 / Math.max(1e-6, cellM);
    int cx = (int) Math.floor(p.getX() * inv);
    int cy = (int) Math.floor(p.getY() * inv);
    return SpatialDyn.key(cx, cy);
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

  private IntentAgg softIntentAggArr(HashMap<Integer, Track> map, Translation2d[] targets) {
    int m = targets != null ? targets.length : 0;
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
        Translation2d t = targets[i];
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

        double logit = -(eta * invT) + 0.35 * align;
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

    return new IntentAgg(accum, n);
  }

  private static final class SpatialDyn {
    static long key(int cx, int cy) {
      return (((long) cx) << 32) ^ (cy & 0xffffffffL);
    }

    final List<DynamicObject> all;
    final List<DynamicObject> resources;
    final List<DynamicObject> others;
    final HashMap<String, ResourceSpec> specs;

    final double cellM;
    final HashMap<Long, ArrayList<DynamicObject>> resCells;
    final HashMap<Long, ArrayList<DynamicObject>> othCells;

    SpatialDyn(List<DynamicObject> dyn, HashMap<String, ResourceSpec> specsIn) {
      this.all = dyn != null ? dyn : List.of();
      this.specs = new HashMap<>();
      if (specsIn != null) {
        for (var e : specsIn.entrySet()) {
          if (e.getKey() == null || e.getValue() == null) continue;
          this.specs.put(e.getKey().toLowerCase(), e.getValue());
        }
      }

      this.cellM = 0.50;
      this.resCells = new HashMap<>(128);
      this.othCells = new HashMap<>(128);

      ArrayList<DynamicObject> res = new ArrayList<>();
      ArrayList<DynamicObject> oth = new ArrayList<>();

      for (DynamicObject o : this.all) {
        if (o == null || o.pos == null) continue;
        if (!RobotBase.isSimulation() && o.ageS > RESOURCE_HARD_MAX_AGE_S) continue;

        String ty = o.type != null ? o.type.toLowerCase() : "unknown";
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

          double sigma =
              Math.max(
                  RESOURCE_SIGMA_MIN,
                  Math.min(
                      RESOURCE_SIGMA_ABS_MAX, Math.min(s.sigmaM * RESOURCE_SIGMA_REL_MAX, 2.0)));
          double sig2 = sigma * sigma;

          double ageW =
              RobotBase.isSimulation() ? 1.0 : Math.exp(-COLLECT_AGE_DECAY * Math.max(0.0, o.ageS));

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

          double ageW =
              RobotBase.isSimulation() ? 1.0 : Math.exp(-COLLECT_AGE_DECAY * Math.max(0.0, o.ageS));
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

          double sigma = Math.max(RESOURCE_SIGMA_MIN, Math.min(RESOURCE_SIGMA_ABS_MAX, s.sigmaM));
          double sig2 = sigma * sigma;

          double ageW =
              RobotBase.isSimulation() ? 1.0 : Math.exp(-COLLECT_AGE_DECAY * Math.max(0.0, o.ageS));
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
      int n = 0;

      for (ArrayList<DynamicObject> cell : cellsInRadius(p, r, othCells)) {
        for (int i = 0; i < cell.size(); i++) {
          DynamicObject o = cell.get(i);
          if (o == null || o.pos == null) continue;
          double d = o.pos.getDistance(p);
          agg += densityKernel(d, s);
          n++;
        }
      }

      if (n == 0) return 0.0;
      return agg / Math.max(1.0, n);
    }

    double localAvoidPenalty(Translation2d p, double r) {
      if (p == null || others.isEmpty()) return 0.0;
      double rr = Math.max(0.05, r);
      double rr2 = rr * rr;

      double pen = 0.0;
      int n = 0;

      for (ArrayList<DynamicObject> cell : cellsInRadius(p, rr, othCells)) {
        for (int i = 0; i < cell.size(); i++) {
          DynamicObject o = cell.get(i);
          if (o == null || o.pos == null) continue;
          double dx = o.pos.getX() - p.getX();
          double dy = o.pos.getY() - p.getY();
          double d2 = dx * dx + dy * dy;
          if (d2 > rr2) continue;
          double d = Math.sqrt(Math.max(0.0, d2));
          pen += Math.exp(-0.5 * (d * d) / Math.max(1e-6, (0.45 * 0.45)));
          n++;
        }
      }

      if (n == 0) return 0.0;
      return Math.min(1.25, pen / Math.max(1.0, n));
    }
  }
}
