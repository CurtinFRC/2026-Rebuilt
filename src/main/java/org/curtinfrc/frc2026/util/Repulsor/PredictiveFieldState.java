package org.curtinfrc.frc2026.util.Repulsor;

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
  private static final double PRESSURE_GAIN = 0.6;
  private static final double CONGEST_COST = 0.85;
  private static final double CAPACITY_GAIN = 0.35;
  private static final double HEADING_GAIN = 0.18;

  private final HashMap<Integer, Track> allyMap = new HashMap<>();
  private final HashMap<Integer, Track> enemyMap = new HashMap<>();

  private List<GameElement> worldElements = List.of();
  private Alliance ourAlliance = Alliance.kBlue;

  private RepulsorSetpoint lastChosen = null;
  private double lastChosenTs = 0.0;

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

    double[] allyIntent = softIntent(allyMap, targets);
    double[] enemyIntent = softIntent(enemyMap, targets);

    double now = Timer.getFPGATimestamp();
    List<Candidate> out = new ArrayList<>();
    for (int i = 0; i < targets.size(); i++) {
      Translation2d t = targets.get(i);
      RepulsorSetpoint sp = sps.get(i);
      double ourEta = etaPath(ourPos, t, ourSpeedCap > 0 ? ourSpeedCap : DEFAULT_OUR_SPEED);
      double enemyEta = minEtaToTarget(enemyMap, t);
      double allyEta = minEtaToTarget(allyMap, t);

      double pressure = radialPressure(enemyMap, t, enemyIntent);
      double congestion = radialCongestion(allyMap, t, allyIntent);
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

  private double[] softIntent(HashMap<Integer, Track> map, List<Translation2d> targets) {
    double[] accum = new double[targets.size()];
    if (map.isEmpty()) return accum;
    for (Track r : map.values()) {
      double[] logits = new double[targets.size()];
      double maxLogit = -1e9;
      for (int i = 0; i < targets.size(); i++) {
        Translation2d t = targets.get(i);
        double d = r.pos.getDistance(t);
        double eta = d / Math.max(0.1, r.speedCap);
        Translation2d dir = t.minus(r.pos);
        double align = 0.0;
        if (dir.getNorm() > 1e-6 && r.vel.getNorm() > 1e-6) {
          double cos = dotNorm(dir, r.vel);
          align = (cos + 1.0) * 0.5;
        }
        double logit = -(eta) + 0.6 * align;
        logits[i] = logit;
        if (logit > maxLogit) maxLogit = logit;
      }
      double denom = 0.0;
      for (int i = 0; i < logits.length; i++)
        denom += Math.exp((logits[i] - maxLogit) / Math.max(0.2, SOFTMAX_TEMP));
      for (int i = 0; i < logits.length; i++) {
        double p =
            Math.exp((logits[i] - maxLogit) / Math.max(0.2, SOFTMAX_TEMP)) / Math.max(1e-6, denom);
        accum[i] += p;
      }
    }
    return accum;
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

  private double radialPressure(
      HashMap<Integer, Track> enemies, Translation2d target, double[] enemyIntentOverTargets) {
    if (enemies.isEmpty()) return 0.0;
    double agg = 0.0;
    for (Track r : enemies.values()) {
      double d = r.pos.getDistance(target);
      double k = radialKernel(Math.max(0.0, d - RESERVATION_RADIUS));
      agg += k;
    }
    return agg / Math.max(1.0, enemies.size());
  }

  private double radialCongestion(
      HashMap<Integer, Track> allies, Translation2d target, double[] allyIntentOverTargets) {
    if (allies.isEmpty()) return 0.0;
    double agg = 0.0;
    for (Track r : allies.values()) {
      double d = r.pos.getDistance(target);
      double k = radialKernel(Math.max(0.0, d - RESERVATION_RADIUS));
      agg += k;
    }
    return agg / Math.max(1.0, allies.size());
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
}
