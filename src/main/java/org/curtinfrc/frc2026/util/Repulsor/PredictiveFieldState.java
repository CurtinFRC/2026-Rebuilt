// File: src/main/java/org/curtinfrc/frc2026/util/Repulsor/PredictiveFieldState.java
package org.curtinfrc.frc2026.util.Repulsor;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
      pos = p;
      vel = v;
      speedCap = cap;
      lastTs = t;
    }
  }

  private static final double MIN_DT = 0.02;
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

  private static final double COLLECT_COARSE_MIN_REGION_UNITS = 0.10;
  private static final double COLLECT_FINE_MIN_UNITS = 0.06;

  private static final double RESOURCE_SIGMA_ABS_MAX = 0.45;
  private static final double RESOURCE_SIGMA_REL_MAX = 1.25;
  private static final double RESOURCE_SIGMA_MIN = 0.06;

  private static final int COLLECT_CLUSTER_MAX = 72;
  private static final double COLLECT_CLUSTER_BIN_M = 0.75;
  private static final int COLLECT_GRID_FALLBACK_MAX = 1400;
  private static final double COLLECT_DUP_SKIP_M = 0.30;

  private static final double DEPLETED_TTL_S = 3.25;
  private static final double DEPLETED_DECAY = 1.15;
  private static final double DEPLETED_PEN_W = 1.75;
  private static final double DEPLETED_MARK_NEAR_M = 0.65;
  private static final double DEPLETED_MARK_EMPTY_UNITS = 0.05;

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
    double now = Timer.getFPGATimestamp();
    Track t =
        allyMap.getOrDefault(
            id,
            new Track(
                pos, new Translation2d(), speedCap != null ? speedCap : DEFAULT_ALLY_SPEED, now));
    double dt = Math.max(MIN_DT, now - t.lastTs);
    Translation2d vMeas = velHint != null ? velHint : pos.minus(t.pos).div(dt);
    double vMag =
        Math.min(vMeas.getNorm(), speedCap != null ? Math.max(0.1, speedCap) : DEFAULT_ALLY_SPEED);
    Translation2d vClamped =
        vMeas.getNorm() > 1e-6 ? vMeas.div(vMeas.getNorm()).times(vMag) : new Translation2d();
    t.vel = lerpVec(t.vel, vClamped, VEL_EMA);
    t.vel = clampAccel(t.vel, ACC_LIMIT, dt);
    t.pos = lerpVec(t.pos, pos, POS_EMA);
    t.speedCap = speedCap != null ? speedCap : DEFAULT_ALLY_SPEED;
    t.lastTs = now;
    allyMap.put(id, t);
  }

  public void updateEnemy(int id, Translation2d pos, Translation2d velHint, Double speedCap) {
    double now = Timer.getFPGATimestamp();
    Track t =
        enemyMap.getOrDefault(
            id,
            new Track(
                pos, new Translation2d(), speedCap != null ? speedCap : DEFAULT_ENEMY_SPEED, now));
    double dt = Math.max(MIN_DT, now - t.lastTs);
    Translation2d vMeas = velHint != null ? velHint : pos.minus(t.pos).div(dt);
    double vMag =
        Math.min(vMeas.getNorm(), speedCap != null ? Math.max(0.1, speedCap) : DEFAULT_ENEMY_SPEED);
    Translation2d vClamped =
        vMeas.getNorm() > 1e-6 ? vMeas.div(vMeas.getNorm()).times(vMag) : new Translation2d();
    t.vel = lerpVec(t.vel, vClamped, VEL_EMA);
    t.vel = clampAccel(t.vel, ACC_LIMIT, dt);
    t.pos = lerpVec(t.pos, pos, POS_EMA);
    t.speedCap = speedCap != null ? speedCap : DEFAULT_ENEMY_SPEED;
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
    for (int i = 0; i < targets.size(); i++) {
      Translation2d t = targets.get(i);
      RepulsorSetpoint sp = sps.get(i);
      double ourEta = etaPath(ourPos, t, ourSpeedCap > 0 ? ourSpeedCap : DEFAULT_OUR_SPEED);
      double enemyEta = minEtaToTarget(enemyMap, t);
      double allyEta = minEtaToTarget(allyMap, t);

      double pressure = radialPressure(enemyMap, t, enemyAgg.intent[i], enemyAgg.count);
      double congestion = radialCongestion(allyMap, t, allyAgg.intent[i], allyAgg.count);
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

  public List<RepulsorSetpoint> rankSetpoints(
      Translation2d ourPos, double ourSpeedCap, CategorySpec cat, int limit) {
    List<Candidate> c = rank(ourPos, ourSpeedCap, cat, limit);
    List<RepulsorSetpoint> out = new ArrayList<>();
    for (Candidate k : c) out.add(k.setpoint);
    return out;
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
    int hardMinCountCoarse = 2;
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

    for (int i = 0; i < targets.length; i++) {
      Translation2d cpt = targets[i];
      if (dyn.countResourcesWithin(cpt, hardCountR) < hardMinCountCoarse) continue;

      double eta = etaPath(ourPos, cpt, cap);

      double regionUnits = dyn.valueInSquare(cpt, half);
      if (regionUnits < COLLECT_COARSE_MIN_REGION_UNITS) continue;

      double sat = 1.0 - Math.exp(-COLLECT_VALUE_SAT_K * Math.max(0.0, regionUnits));
      double value = sat * goal * COLLECT_VALUE_GAIN;

      double enemyP = radialPressure(enemyMap, cpt, enemyAgg.intent[i], enemyAgg.count);
      double allyC = radialCongestion(allyMap, cpt, allyAgg.intent[i], allyAgg.count);

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

    int bestI = -1;
    double bestFineScore = -1e18;
    double bestFineEta = 0.0;
    double bestFineVal = 0.0;
    double bestFineEP = 0.0;
    double bestFineAC = 0.0;
    double bestFineEI = 0.0;
    double bestFineAI = 0.0;
    Translation2d bestPt = null;

    double step = (2.0 * half) / (rg - 1);

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

          double rawUnits = dyn.valueAt(p);
          if (rawUnits < COLLECT_FINE_MIN_UNITS) continue;

          double sat = 1.0 - Math.exp(-COLLECT_VALUE_SAT_K * Math.max(0.0, rawUnits));
          double value = sat * goal * COLLECT_VALUE_GAIN;

          double enemyP = radialPressure(enemyMap, p, enemyAgg.intent[idx], enemyAgg.count);
          double allyC = radialCongestion(allyMap, p, allyAgg.intent[idx], allyAgg.count);

          double localAvoid = dyn.localAvoidPenalty(p, COLLECT_LOCAL_AVOID_R);

          double activity =
              COLLECT_ACTIVITY_ALLY_W * radialDensity(allyMap, p, COLLECT_ACTIVITY_SIGMA)
                  + COLLECT_ACTIVITY_ENEMY_W * radialDensity(enemyMap, p, COLLECT_ACTIVITY_SIGMA)
                  + COLLECT_ACTIVITY_DYN_W * dyn.otherDensity(p, COLLECT_ACTIVITY_SIGMA);

          double dep = depletedPenalty(p, cellM);

          double score =
              value
                  - eta * COLLECT_ETA_COST
                  - enemyP * COLLECT_ENEMY_PRESS_COST
                  - allyC * COLLECT_ALLY_CONGEST_COST
                  - eiC * COLLECT_ENEMY_INTENT_COST
                  - aiC * COLLECT_ALLY_INTENT_COST
                  - localAvoid
                  - activity
                  - dep * DEPLETED_PEN_W;

          if (score > bestFineScore) {
            bestFineScore = score;
            bestI = idx;
            bestPt = p;
            bestFineEta = eta;
            bestFineVal = value;
            bestFineEP = enemyP;
            bestFineAC = allyC;
            bestFineEI = eiC;
            bestFineAI = aiC;
          }
        }
      }
    }

    if (bestPt == null || bestI < 0) return null;

    Translation2d snapped = dyn.centroidResourcesWithin(bestPt, snapR, snapMinMass);
    if (snapped != null) bestPt = snapped;

    int finalCount = dyn.countResourcesWithin(bestPt, hardCountR);
    double finalUnits = dyn.valueAt(bestPt);

    Logger.recordOutput("Repulsor/Collect/ChosenFuelUnits", finalUnits);
    Logger.recordOutput("Repulsor/Collect/ChosenScore", bestFineScore);
    Logger.recordOutput("Repulsor/Collect/ChosenCount", finalCount);

    if (finalCount < 1 || finalUnits < COLLECT_FINE_MIN_UNITS) {
      markDepleted(bestPt, cellM, 1.0);
      return null;
    }

    lastReturnedCollect = bestPt;
    lastReturnedCollectTs = Timer.getFPGATimestamp();

    if (ourPos.getDistance(bestPt) <= DEPLETED_MARK_NEAR_M) {
      double u = dyn.valueAt(bestPt);
      if (u < DEPLETED_MARK_EMPTY_UNITS) markDepleted(bestPt, cellM, 1.0);
    }

    Logger.recordOutput("Repulsor/ChosenCollect", new Pose2d(bestPt, new Rotation2d()));

    return new PointCandidate(
        bestPt,
        bestFineEta,
        bestFineVal,
        bestFineEP,
        bestFineAC,
        bestFineEI,
        bestFineAI,
        bestFineScore);
  }

  public PointCandidate rankCollectPoints(
      Translation2d ourPos, double ourSpeedCap, Translation2d[] points, int goalUnits, int limit) {
    if (ourPos == null) return null;

    SpatialDyn dyn = cachedDyn();

    Translation2d[] targets = buildCollectCandidates(points, dyn);
    if (targets.length == 0) return null;

    IntentAgg allyAgg = softIntentAggArr(allyMap, targets);
    IntentAgg enemyAgg = softIntentAggArr(enemyMap, targets);

    sweepDepleted();

    double cap = ourSpeedCap > 0.0 ? ourSpeedCap : DEFAULT_OUR_SPEED;

    int goal = Math.max(1, goalUnits);
    int bestI = -1;
    double bestScore = -1e18;
    double bestEta = 0.0;
    double bestVal = 0.0;
    double bestEP = 0.0;
    double bestAC = 0.0;
    double bestEI = 0.0;
    double bestAI = 0.0;

    for (int i = 0; i < targets.length; i++) {
      Translation2d t = targets[i];
      double eta = etaPath(ourPos, t, cap);

      double rawUnits = dyn.valueAt(t);
      if (rawUnits < COLLECT_FINE_MIN_UNITS) continue;

      double sat = 1.0 - Math.exp(-COLLECT_VALUE_SAT_K * Math.max(0.0, rawUnits));
      double value = sat * goal * COLLECT_VALUE_GAIN;

      double enemyP = radialPressure(enemyMap, t, enemyAgg.intent[i], enemyAgg.count);
      double allyC = radialCongestion(allyMap, t, allyAgg.intent[i], allyAgg.count);

      double ei = enemyAgg.count > 0 ? enemyAgg.intent[i] / Math.max(1.0, enemyAgg.count) : 0.0;
      double ai = allyAgg.count > 0 ? allyAgg.intent[i] / Math.max(1.0, allyAgg.count) : 0.0;

      double localAvoid = dyn.localAvoidPenalty(t, COLLECT_LOCAL_AVOID_R);

      double activity =
          COLLECT_ACTIVITY_ALLY_W * radialDensity(allyMap, t, COLLECT_ACTIVITY_SIGMA)
              + COLLECT_ACTIVITY_ENEMY_W * radialDensity(enemyMap, t, COLLECT_ACTIVITY_SIGMA)
              + COLLECT_ACTIVITY_DYN_W * dyn.otherDensity(t, COLLECT_ACTIVITY_SIGMA);

      double dep = depletedPenalty(t, 0.45);

      double score =
          value
              - eta * COLLECT_ETA_COST
              - enemyP * COLLECT_ENEMY_PRESS_COST
              - allyC * COLLECT_ALLY_CONGEST_COST
              - ei * COLLECT_ENEMY_INTENT_COST
              - ai * COLLECT_ALLY_INTENT_COST
              - localAvoid
              - activity
              - dep * DEPLETED_PEN_W;

      if (score > bestScore) {
        bestScore = score;
        bestI = i;
        bestEta = eta;
        bestVal = value;
        bestEP = enemyP;
        bestAC = allyC;
        bestEI = ei;
        bestAI = ai;
      }
    }

    if (bestI < 0) return null;
    Translation2d chosen = targets[bestI];

    int count = dyn.countResourcesWithin(chosen, 0.75);
    double units = dyn.valueAt(chosen);

    Logger.recordOutput("Repulsor/Collect/ChosenFuelUnits", units);
    Logger.recordOutput("Repulsor/Collect/ChosenScore", bestScore);
    Logger.recordOutput("Repulsor/Collect/ChosenCount", count);

    if (count < 1 || units < COLLECT_FINE_MIN_UNITS) {
      markDepleted(chosen, 0.45, 1.0);
      return null;
    }

    lastReturnedCollect = chosen;
    lastReturnedCollectTs = Timer.getFPGATimestamp();

    if (ourPos.getDistance(chosen) <= DEPLETED_MARK_NEAR_M) {
      double u = dyn.valueAt(chosen);
      if (u < DEPLETED_MARK_EMPTY_UNITS) markDepleted(chosen, 0.45, 1.0);
    }

    Logger.recordOutput("Repulsor/ChosenCollect", new Pose2d(chosen, new Rotation2d()));

    return new PointCandidate(chosen, bestEta, bestVal, bestEP, bestAC, bestEI, bestAI, bestScore);
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

    double minKeepUnits = Math.max(COLLECT_FINE_MIN_UNITS, 0.10);
    double countR = 0.65;

    int added = 0;
    int seen = 0;
    for (int i = 0; i < gridPoints.length && added < take; i++) {
      Translation2d p = gridPoints[i];
      if (p == null) continue;
      if ((seen++ % stride) != 0) continue;

      if (dyn.valueAt(p) < minKeepUnits) continue;
      if (dyn.countResourcesWithin(p, countR) < 1) continue;

      boolean dup = false;
      for (int j = 0; j < clusters.length; j++) {
        Translation2d c = clusters[j];
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

      double ageW = Math.exp(-COLLECT_AGE_DECAY * Math.max(0.0, o.ageS));
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
    if (depleted.isEmpty()) return;
    double now = Timer.getFPGATimestamp();
    depleted.entrySet().removeIf(e -> now - e.getValue().t > DEPLETED_TTL_S);
  }

  private void markDepleted(Translation2d p, double cellM, double strength) {
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
    return new Translation2d(
        a.getX() + (b.getX() - a.getX()) * alpha, a.getY() + (b.getY() - a.getY()) * alpha);
  }

  private static Translation2d clampAccel(Translation2d v, double aMax, double dt) {
    double lim = Math.max(0.1, aMax * dt);
    double n = v.getNorm();
    if (n <= lim) return v;
    return v.div(n).times(lim);
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

  private double radialPressure(
      HashMap<Integer, Track> enemies, Translation2d target, double intentMass, int count) {
    if (enemies.isEmpty()) return 0.0;
    double agg = 0.0;
    for (Track r : enemies.values()) {
      double d = r.pos.getDistance(target);
      double k = radialKernel(Math.max(0.0, d - RESERVATION_RADIUS));
      agg += k;
    }
    double base = agg / Math.max(1.0, enemies.size());
    double intent = count > 0 ? (intentMass / Math.max(1.0, count)) : 0.0;
    return base * (1.0 + 0.85 * intent);
  }

  private double radialCongestion(
      HashMap<Integer, Track> allies, Translation2d target, double intentMass, int count) {
    if (allies.isEmpty()) return 0.0;
    double agg = 0.0;
    for (Track r : allies.values()) {
      double d = r.pos.getDistance(target);
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
          double cos = dotNorm(dir, r.vel);
          align = (cos + 1.0) * 0.5;
        }

        double logit = -(eta) + 0.65 * align;
        logits[i] = logit;
        if (logit > maxLogit) maxLogit = logit;
      }

      double denom = 0.0;
      for (int i = 0; i < m; i++) denom += Math.exp((logits[i] - maxLogit) * invT);
      double invDen = 1.0 / Math.max(1e-9, denom);

      for (int i = 0; i < m; i++) {
        double p = Math.exp((logits[i] - maxLogit) * invT) * invDen;
        accum[i] += p;
      }
    }

    return new IntentAgg(accum, n);
  }

  private IntentAgg softIntentAggArr(HashMap<Integer, Track> map, Translation2d[] targets) {
    int m = targets.length;
    double[] accum = new double[m];
    if (map.isEmpty() || m == 0) return new IntentAgg(accum, 0);

    ensureScratch(m);
    double[] logits = scratchLogits;

    int n = 0;
    double invT = 1.0 / Math.max(0.2, SOFTMAX_TEMP);

    for (Track r : map.values()) {
      n++;
      double maxLogit = -1e18;

      for (int i = 0; i < m; i++) {
        Translation2d t = targets[i];
        double d = r.pos.getDistance(t);
        double eta = d / Math.max(0.1, r.speedCap);

        Translation2d dir = t.minus(r.pos);
        double align = 0.0;
        if (dir.getNorm() > 1e-6 && r.vel.getNorm() > 1e-6) {
          double cos = dotNorm(dir, r.vel);
          align = (cos + 1.0) * 0.5;
        }

        double logit = -(eta) + 0.65 * align;
        logits[i] = logit;
        if (logit > maxLogit) maxLogit = logit;
      }

      double denom = 0.0;
      for (int i = 0; i < m; i++) denom += Math.exp((logits[i] - maxLogit) * invT);
      double invDen = 1.0 / Math.max(1e-9, denom);

      for (int i = 0; i < m; i++) {
        double p = Math.exp((logits[i] - maxLogit) * invT) * invDen;
        accum[i] += p;
      }
    }

    return new IntentAgg(accum, n);
  }

  private static final class SpatialDyn {
    private final ArrayList<DynamicObject> resources = new ArrayList<>();
    private final ArrayList<DynamicObject> others = new ArrayList<>();
    private final HashMap<String, ResourceSpec> specs;
    private final HashMap<Long, IntList> grid = new HashMap<>();
    private final double cell;
    private final double invCell;

    SpatialDyn(List<DynamicObject> dyn, HashMap<String, ResourceSpec> specs) {
      this.specs = specs != null ? specs : new HashMap<>();
      double c = 0.45;
      for (ResourceSpec s : this.specs.values()) {
        c = Math.max(c, s.radiusM);
      }
      this.cell = c;
      this.invCell = 1.0 / Math.max(1e-6, c);

      if (dyn != null) {
        for (DynamicObject o : dyn) {
          if (o == null) continue;
          if (isResource(o)) {
            if (o.ageS <= RESOURCE_HARD_MAX_AGE_S) resources.add(o);
          } else {
            others.add(o);
          }
        }
      }

      for (int i = 0; i < resources.size(); i++) {
        DynamicObject o = resources.get(i);
        int cx = (int) Math.floor(o.pos.getX() * invCell);
        int cy = (int) Math.floor(o.pos.getY() * invCell);
        long k = key(cx, cy);
        IntList lst = grid.get(k);
        if (lst == null) {
          lst = new IntList();
          grid.put(k, lst);
        }
        lst.add(i);
      }
    }

    private boolean isResource(DynamicObject o) {
      if (o.type == null) return false;
      return specs.containsKey(o.type.toLowerCase());
    }

    private static long key(int x, int y) {
      return (((long) x) << 32) ^ (y & 0xffffffffL);
    }

    private double kernel(double d, double sigma) {
      double s2 = sigma * sigma;
      return Math.exp(-0.5 * (d * d) / Math.max(1e-6, s2));
    }

    private double effectiveSigma(ResourceSpec s) {
      double maxRel = Math.max(RESOURCE_SIGMA_MIN, s.radiusM * RESOURCE_SIGMA_REL_MAX);
      double cap = Math.min(RESOURCE_SIGMA_ABS_MAX, maxRel);
      return Math.max(RESOURCE_SIGMA_MIN, Math.min(s.sigmaM, cap));
    }

    double valueAt(Translation2d p) {
      if (resources.isEmpty() || specs.isEmpty()) return 0.0;
      int cx = (int) Math.floor(p.getX() * invCell);
      int cy = (int) Math.floor(p.getY() * invCell);
      double sum = 0.0;

      for (int dx = -1; dx <= 1; dx++) {
        for (int dy = -1; dy <= 1; dy++) {
          IntList lst = grid.get(key(cx + dx, cy + dy));
          if (lst == null) continue;
          int[] a = lst.a;
          int n = lst.n;
          for (int ii = 0; ii < n; ii++) {
            DynamicObject o = resources.get(a[ii]);
            ResourceSpec s = specs.get(o.type.toLowerCase());
            if (s == null) continue;
            double d = o.pos.getDistance(p);
            if (d <= s.radiusM * 2.2) {
              double ageW = Math.exp(-COLLECT_AGE_DECAY * Math.max(0.0, o.ageS));
              double sig = effectiveSigma(s);
              sum += s.unitValue * kernel(d, sig) * ageW;
            }
          }
        }
      }
      return sum;
    }

    double valueInSquare(Translation2d center, double half) {
      double s = 0.0;
      s += valueAt(center);
      s += valueAt(new Translation2d(center.getX() - half * 0.6, center.getY()));
      s += valueAt(new Translation2d(center.getX() + half * 0.6, center.getY()));
      s += valueAt(new Translation2d(center.getX(), center.getY() - half * 0.6));
      s += valueAt(new Translation2d(center.getX(), center.getY() + half * 0.6));
      return s * 0.2;
    }

    double otherDensity(Translation2d p, double sigma) {
      if (others.isEmpty()) return 0.0;
      double agg = 0.0;
      for (DynamicObject o : others) {
        double d = o.pos.getDistance(p);
        agg += densityKernel(d, sigma);
      }
      return agg / Math.max(1.0, others.size());
    }

    double localAvoidPenalty(Translation2d p, double r) {
      if (others.isEmpty()) return 0.0;
      double r2 = r * r;
      double agg = 0.0;
      for (DynamicObject o : others) {
        double dx = o.pos.getX() - p.getX();
        double dy = o.pos.getY() - p.getY();
        double d2 = dx * dx + dy * dy;
        if (d2 <= r2) {
          double d = Math.sqrt(Math.max(1e-9, d2));
          double w = 1.0 / (0.22 + d);
          agg += w;
        }
      }
      return agg * 0.22;
    }

    int countResourcesWithin(Translation2d p, double r) {
      if (resources.isEmpty() || specs.isEmpty()) return 0;
      double r2 = r * r;
      int cx = (int) Math.floor(p.getX() * invCell);
      int cy = (int) Math.floor(p.getY() * invCell);

      int cnt = 0;
      int cells = (int) Math.ceil(r * invCell);

      for (int dx = -cells; dx <= cells; dx++) {
        for (int dy = -cells; dy <= cells; dy++) {
          IntList lst = grid.get(key(cx + dx, cy + dy));
          if (lst == null) continue;
          int[] a = lst.a;
          int n = lst.n;
          for (int ii = 0; ii < n; ii++) {
            DynamicObject o = resources.get(a[ii]);
            ResourceSpec s = specs.get(o.type.toLowerCase());
            if (s == null) continue;
            double ox = o.pos.getX() - p.getX();
            double oy = o.pos.getY() - p.getY();
            if (ox * ox + oy * oy <= r2) cnt++;
          }
        }
      }
      return cnt;
    }

    Translation2d centroidResourcesWithin(Translation2d p, double r, double minMass) {
      if (resources.isEmpty() || specs.isEmpty()) return null;
      double r2 = r * r;
      int cx = (int) Math.floor(p.getX() * invCell);
      int cy = (int) Math.floor(p.getY() * invCell);

      double sx = 0.0, sy = 0.0, sw = 0.0;
      int cells = (int) Math.ceil(r * invCell);

      for (int dx = -cells; dx <= cells; dx++) {
        for (int dy = -cells; dy <= cells; dy++) {
          IntList lst = grid.get(key(cx + dx, cy + dy));
          if (lst == null) continue;
          int[] a = lst.a;
          int n = lst.n;
          for (int ii = 0; ii < n; ii++) {
            DynamicObject o = resources.get(a[ii]);
            ResourceSpec s = specs.get(o.type.toLowerCase());
            if (s == null) continue;

            double ox = o.pos.getX() - p.getX();
            double oy = o.pos.getY() - p.getY();
            double d2 = ox * ox + oy * oy;
            if (d2 > r2) continue;

            double ageW = Math.exp(-COLLECT_AGE_DECAY * Math.max(0.0, o.ageS));
            double w = Math.max(0.0, s.unitValue) * ageW;
            sx += o.pos.getX() * w;
            sy += o.pos.getY() * w;
            sw += w;
          }
        }
      }

      if (sw < minMass) return null;
      return new Translation2d(sx / sw, sy / sw);
    }
  }

  private static final class IntList {
    int[] a = new int[8];
    int n = 0;

    void add(int v) {
      if (n >= a.length) {
        int[] b = new int[a.length * 2];
        System.arraycopy(a, 0, b, 0, a.length);
        a = b;
      }
      a[n++] = v;
    }
  }
}
