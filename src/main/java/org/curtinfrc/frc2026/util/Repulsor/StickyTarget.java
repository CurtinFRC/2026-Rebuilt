package org.curtinfrc.frc2026.util.Repulsor;

import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.function.Predicate;
import java.util.function.ToDoubleBiFunction;
import java.util.function.ToDoubleFunction;

public final class StickyTarget<T> {
  private final double candidateStableSec;
  private final double maxStaleSec;
  private final double switchBackCooldownSec;

  private static final double FORCE_SWITCH_STILL_SEC = 0.10;
  private static final double FORCE_SWITCH_FLICKER_SEC = 0.20;

  private static final double STICKY_INVALID_DEBOUNCE_SEC = 0.35;

  private static final double BASE_HARD_LOCK_SEC = 0.25;
  private static final double HARD_LOCK_PER_M_SEC = 0.18;
  private static final double HARD_LOCK_MAX_SEC = 1.20;

  private static final double EARLY_SWITCH_MULT = 1.25;
  private static final double SWITCH_BACK_EPS_MIN = 0.35;

  private double pingPeriodEmaSec = 0.0;

  private static final double PINGPONG_EMA_ALPHA = 0.35;

  private static final double PINGPONG_WINDOW_MIN_SEC = 0.50;
  private static final double PINGPONG_WINDOW_MAX_SEC = 2.10;

  private static final double PINGPONG_BLOCK_MIN_SEC = 0.60;
  private static final double PINGPONG_BLOCK_MAX_SEC = 2.20;

  private static final double PINGPONG_WINDOW_BASE_SEC = 0.60;
  private static final double PINGPONG_WINDOW_PER_M_SEC = 0.45;

  private static final double PINGPONG_BLOCK_BASE_SEC = 0.70;
  private static final double PINGPONG_BLOCK_PER_M_SEC = 0.45;

  private static final double MIN_SWITCH_STABLE_SEC = 0.05;
  private static final double FLICKER_DELTA_SEC = 0.10;
  private static final double FLICKER_RESET_STABLE_SEC = 0.18;

  private static final double BEST_VALID_DEBOUNCE_SEC = 0.06;
  private static final double BEST_MISSING_DEBOUNCE_SEC = 0.10;
  private static final double BEST_MISSING_DROP_SEC = 0.40;

  private static final double DEFAULT_SEEN_TIMEOUT_SEC = 0.45;

  private static final double CANON_EVICT_SEC = 5.0;
  private static final int CANON_MAX = 28;

  private static final double FLICKER_FREEZE_EXTEND_SEC = 0.30;
  private static final double HARD_LOCK_OVERRIDE_MULT = 2.00;

  private static final double CLOSE_PAIR_DIST_ADD = 0.15;
  private static final double CLOSE_PAIR_REQ_MULT = 1.15;

  private T lastOut;
  private double lastOutChangeSec = -1e9;

  private double stickyInvalidSinceSec = -1e9;

  private T bestValidKey;

  private T lastBestSeen;
  private double bestLastChangeSec = -1e9;
  private double flickerSinceSec;

  private T sticky;
  private double stickySinceSec;
  private double stickyLastBestSeenSec;

  private T candidate;
  private double candidateSinceSec;

  private T lastSticky;
  private double lastSwitchSec;

  private double lastUpdateSec;

  private double hardLockUntilSec = -1e9;

  private T pingA;
  private T pingB;
  private double pingSinceSec = -1e9;
  private double pingBlockUntilSec = -1e9;

  private static final class EmaEntry {
    double v;
    double t;

    EmaEntry(double v, double t) {
      this.v = v;
      this.t = t;
    }
  }

  private final Map<T, EmaEntry> emaScore = new HashMap<>(8);
  private final double emaTauSec = 0.26;
  private double emaEvictSec = 3.5;

  private final Map<T, Double> lastSeenSec = new HashMap<>(16);
  private double seenTimeoutSec;

  private double bestMissingSinceSec = -1e9;
  private double bestValidSinceSec = -1e9;

  private final ArrayList<T> canon = new ArrayList<>(CANON_MAX);
  private final Map<T, Double> canonLastUseSec = new HashMap<>(CANON_MAX);

  public StickyTarget(double candidateStableSec, double maxStaleSec, double switchBackCooldownSec) {
    this.candidateStableSec = Math.max(0.0, candidateStableSec);
    this.maxStaleSec = Math.max(0.0, maxStaleSec);
    this.switchBackCooldownSec = Math.max(0.0, switchBackCooldownSec);
    double base = this.maxStaleSec > 1e-6 ? this.maxStaleSec : DEFAULT_SEEN_TIMEOUT_SEC;
    this.seenTimeoutSec = Math.max(0.12, Math.min(2.5, base));
    clear();
  }

  public Optional<T> get() {
    return Optional.ofNullable(sticky);
  }

  public void setSeenTimeoutSec(double sec) {
    if (!Double.isFinite(sec)) return;
    seenTimeoutSec = Math.max(0.12, sec);
  }

  public void setEmaEvictSec(double sec) {
    if (!Double.isFinite(sec)) return;
    emaEvictSec = Math.max(0.25, sec);
  }

  public void noteSeen(T value) {
    if (value == null) return;
    double now = Timer.getFPGATimestamp();
    lastSeenSec.put(value, now);
  }

  public void noteSeen(Iterable<T> values) {
    if (values == null) return;
    double now = Timer.getFPGATimestamp();
    for (T v : values) {
      if (v != null) lastSeenSec.put(v, now);
    }
  }

  public void clear() {
    sticky = null;
    candidate = null;
    lastSticky = null;
    stickySinceSec = -1e9;
    stickyLastBestSeenSec = -1e9;
    candidateSinceSec = -1e9;
    lastSwitchSec = -1e9;
    lastUpdateSec = 0.0;
    lastBestSeen = null;
    bestLastChangeSec = -1e9;
    flickerSinceSec = -1e9;
    stickyInvalidSinceSec = -1e9;
    hardLockUntilSec = -1e9;
    lastOut = null;
    lastOutChangeSec = -1e9;
    pingA = null;
    pingB = null;
    pingSinceSec = -1e9;
    pingBlockUntilSec = -1e9;
    emaScore.clear();
    lastSeenSec.clear();
    bestMissingSinceSec = -1e9;
    bestValidSinceSec = -1e9;
    bestValidKey = null;
    canon.clear();
    canonLastUseSec.clear();
    pingPeriodEmaSec = 0.0;
  }

  public void force(T value) {
    double now = Timer.getFPGATimestamp();
    sticky = value;
    candidate = null;
    candidateSinceSec = -1e9;
    stickySinceSec = now;
    stickyLastBestSeenSec = now;
    lastSticky = null;
    lastSwitchSec = -1e9;
    lastBestSeen = value;
    bestLastChangeSec = now;
    flickerSinceSec = -1e9;
    stickyInvalidSinceSec = -1e9;
    hardLockUntilSec = -1e9;
    pingA = null;
    pingB = null;
    pingSinceSec = -1e9;
    pingBlockUntilSec = -1e9;
    bestMissingSinceSec = -1e9;
    bestValidSinceSec = now;
    bestValidKey = value;
    if (value != null) lastSeenSec.put(value, now);
    if (value != null) {
      canonAdd(now, value);
    }
  }

  public void forceInvalidate() {
    double now = Timer.getFPGATimestamp();
    sticky = null;
    candidate = null;
    lastSticky = null;
    lastSwitchSec = now;
    stickySinceSec = -1e9;
    stickyLastBestSeenSec = -1e9;
    candidateSinceSec = -1e9;
    bestMissingSinceSec = -1e9;
    bestValidSinceSec = -1e9;
    bestValidKey = null;
    lastOut = null;
    lastOutChangeSec = now;
    hardLockUntilSec = -1e9;
    pingA = null;
    pingB = null;
    pingSinceSec = -1e9;
    pingBlockUntilSec = -1e9;
    emaScore.clear();
    lastSeenSec.clear();
  }

  private double clampDt(double dt) {
    if (dt < 1e-3) return 1e-3;
    if (dt > 0.10) return 0.10;
    return dt;
  }

  private void evictEma(double now) {
    if (emaScore.isEmpty()) return;
    double ttl = Math.max(0.25, emaEvictSec);
    Iterator<Map.Entry<T, EmaEntry>> it = emaScore.entrySet().iterator();
    while (it.hasNext()) {
      Map.Entry<T, EmaEntry> e = it.next();
      EmaEntry v = e.getValue();
      if (v == null || (now - v.t) > ttl) it.remove();
    }
  }

  private double ema(double now, T key, double raw, double dt) {
    if (key == null) return raw;
    EmaEntry prevE = emaScore.get(key);
    double prev = prevE != null ? prevE.v : raw;
    double tau = Math.max(1e-3, emaTauSec);
    double a = 1.0 - Math.exp(-dt / tau);
    double v = prev + (raw - prev) * a;
    emaScore.put(key, new EmaEntry(v, now));
    return v;
  }

  private boolean eq(T a, T b, ToDoubleBiFunction<T, T> distanceFn, double sameEps) {
    if (a == null || b == null) return false;
    if (Objects.equals(a, b)) return true;
    if (distanceFn == null) return false;
    double d = distanceFn.applyAsDouble(a, b);
    if (!Double.isFinite(d) || d < 0.0) return false;
    return d <= Math.max(0.0, sameEps);
  }

  private boolean eqN(T a, T b, ToDoubleBiFunction<T, T> distanceFn, double sameEps) {
    if (a == null && b == null) return true;
    if (a == null || b == null) return false;
    return eq(a, b, distanceFn, sameEps);
  }

  private double dist(T a, T b, ToDoubleBiFunction<T, T> distanceFn) {
    if (a == null || b == null || distanceFn == null) return 0.0;
    double d = distanceFn.applyAsDouble(a, b);
    if (!Double.isFinite(d) || d < 0.0) return 0.0;
    return d;
  }

  private double seenAge(double now, T t) {
    if (t == null) return 1e9;
    Double s = lastSeenSec.get(t);
    if (s == null) return 1e9;
    return now - s;
  }

  private boolean seenRecently(double now, T t) {
    return seenAge(now, t) <= Math.max(0.12, seenTimeoutSec);
  }

  private void armHardLock(double now, double switchDistM) {
    double lock = BASE_HARD_LOCK_SEC + HARD_LOCK_PER_M_SEC * Math.max(0.0, switchDistM);
    if (lock > HARD_LOCK_MAX_SEC) lock = HARD_LOCK_MAX_SEC;
    if (lock < BASE_HARD_LOCK_SEC) lock = BASE_HARD_LOCK_SEC;
    hardLockUntilSec = now + lock;
  }

  private void notePingPong(
      double now,
      T from,
      T to,
      ToDoubleBiFunction<T, T> distanceFn,
      double eps,
      double switchDistM) {

    if (from == null || to == null) return;

    double d = Double.isFinite(switchDistM) ? Math.max(0.0, switchDistM) : 0.0;

    double windowDist = PINGPONG_WINDOW_BASE_SEC + PINGPONG_WINDOW_PER_M_SEC * d;
    if (windowDist > PINGPONG_WINDOW_MAX_SEC) windowDist = PINGPONG_WINDOW_MAX_SEC;
    if (windowDist < PINGPONG_WINDOW_BASE_SEC) windowDist = PINGPONG_WINDOW_BASE_SEC;

    double blockDist = PINGPONG_BLOCK_BASE_SEC + PINGPONG_BLOCK_PER_M_SEC * d;
    if (blockDist > PINGPONG_BLOCK_MAX_SEC) blockDist = PINGPONG_BLOCK_MAX_SEC;
    if (blockDist < PINGPONG_BLOCK_BASE_SEC) blockDist = PINGPONG_BLOCK_BASE_SEC;

    if (pingA == null || pingB == null) {
      pingA = from;
      pingB = to;
      pingSinceSec = now;
      return;
    }

    boolean sameAB = eq(from, pingA, distanceFn, eps) && eq(to, pingB, distanceFn, eps);
    boolean sameBA = eq(from, pingB, distanceFn, eps) && eq(to, pingA, distanceFn, eps);

    if (sameAB || sameBA) {
      double age = now - pingSinceSec;

      if (age > 0.0 && Double.isFinite(age)) {
        if (pingPeriodEmaSec <= 0.0) pingPeriodEmaSec = age;
        else pingPeriodEmaSec += (age - pingPeriodEmaSec) * PINGPONG_EMA_ALPHA;
      }

      double windowLearned = pingPeriodEmaSec > 0.0 ? (pingPeriodEmaSec * 1.12 + 0.18) : 0.0;
      double blockLearned = pingPeriodEmaSec > 0.0 ? (pingPeriodEmaSec * 0.85 + 0.65) : 0.0;

      if (windowLearned < PINGPONG_WINDOW_MIN_SEC) windowLearned = PINGPONG_WINDOW_MIN_SEC;
      if (windowLearned > PINGPONG_WINDOW_MAX_SEC) windowLearned = PINGPONG_WINDOW_MAX_SEC;

      if (blockLearned < PINGPONG_BLOCK_MIN_SEC) blockLearned = PINGPONG_BLOCK_MIN_SEC;
      if (blockLearned > PINGPONG_BLOCK_MAX_SEC) blockLearned = PINGPONG_BLOCK_MAX_SEC;

      double window = Math.max(windowDist, windowLearned);
      double block = Math.max(blockDist, blockLearned);

      if (age <= window) {
        pingBlockUntilSec = Math.max(pingBlockUntilSec, now + block);
      }

      pingSinceSec = now;
      return;
    }

    pingA = from;
    pingB = to;
    pingSinceSec = now;
  }

  private boolean pingBlocked(double now, T next, ToDoubleBiFunction<T, T> distanceFn, double eps) {
    if (now >= pingBlockUntilSec) return false;
    if (next == null) return false;
    if (pingA != null && eq(next, pingA, distanceFn, eps)) return true;
    if (pingB != null && eq(next, pingB, distanceFn, eps)) return true;
    return false;
  }

  private T commitSwitch(double now, T next, ToDoubleBiFunction<T, T> distanceFn, double eps) {
    T prev = sticky;
    lastSticky = prev;
    lastSwitchSec = now;
    sticky = next;
    stickySinceSec = now;
    stickyLastBestSeenSec = now;
    candidate = null;
    candidateSinceSec = -1e9;
    flickerSinceSec = -1e9;
    stickyInvalidSinceSec = -1e9;

    double d = dist(prev, sticky, distanceFn);
    armHardLock(now, d);
    notePingPong(now, prev, sticky, distanceFn, eps, d);

    if (sticky != null) lastSeenSec.put(sticky, now);

    return sticky;
  }

  private static double safeScore(double v) {
    if (!Double.isFinite(v)) return 0.0;
    if (v > 1e9) return 1e9;
    if (v < -1e9) return -1e9;
    return v;
  }

  private double adaptiveEps(double sameEps, double flickerBoost, double motionBoost) {
    double e = Math.max(0.0, sameEps);
    double baseFloor = 0.08;
    if (e < baseFloor) e = baseFloor;
    e *= (1.0 + 0.85 * flickerBoost);
    e *= (1.0 + 0.65 * motionBoost);
    if (e < baseFloor) e = baseFloor;
    return e;
  }

  private void canonEvict(double now) {
    if (canon.isEmpty()) return;
    double ttl = Math.max(0.75, CANON_EVICT_SEC);
    boolean any = false;
    for (int i = canon.size() - 1; i >= 0; i--) {
      T r = canon.get(i);
      Double t = canonLastUseSec.get(r);
      if (r == null || t == null || (now - t) > ttl) {
        canon.remove(i);
        canonLastUseSec.remove(r);
        any = true;
      }
    }
    if (!any && canon.size() > CANON_MAX) {
      while (canon.size() > CANON_MAX) {
        T r = canon.remove(0);
        canonLastUseSec.remove(r);
      }
    }
  }

  private void canonAdd(double now, T v) {
    if (v == null) return;
    if (canon.size() >= CANON_MAX) {
      canonEvict(now);
      if (canon.size() >= CANON_MAX) {
        T r = canon.remove(0);
        canonLastUseSec.remove(r);
      }
    }
    canon.add(v);
    canonLastUseSec.put(v, now);
  }

  private T canonicalize(double now, T v, ToDoubleBiFunction<T, T> distanceFn, double eps) {
    if (v == null) return null;
    if (distanceFn == null) {
      canonLastUseSec.put(v, now);
      if (!canonLastUseSec.containsKey(v)) canonAdd(now, v);
      return v;
    }
    canonEvict(now);
    for (int i = 0; i < canon.size(); i++) {
      T r = canon.get(i);
      if (r == null) continue;
      if (eq(v, r, distanceFn, eps)) {
        canonLastUseSec.put(r, now);
        return r;
      }
    }
    canonAdd(now, v);
    return v;
  }

  private T fallbackOut(
      double now, Predicate<T> valid, ToDoubleBiFunction<T, T> distanceFn, double sameEps) {
    if (sticky != null) return sticky;
    if (lastOut != null && valid.test(lastOut) && seenRecently(now, lastOut)) {
      sticky = lastOut;
      stickySinceSec = now;
      stickyLastBestSeenSec = now;
      candidate = null;
      candidateSinceSec = -1e9;
      lastSticky = null;
      lastSwitchSec = -1e9;
      hardLockUntilSec = -1e9;
      pingA = null;
      pingB = null;
      pingSinceSec = -1e9;
      pingBlockUntilSec = -1e9;
      lastSeenSec.put(lastOut, now);
      recordOut(now, sticky, distanceFn, sameEps);
      return sticky;
    }
    recordOut(now, null, distanceFn, sameEps);
    return null;
  }

  public T update(
      T best,
      double bestScore,
      ToDoubleFunction<T> scoreFn,
      Predicate<T> validFn,
      double minHoldSec,
      double keepMargin,
      double immediateDelta,
      ToDoubleBiFunction<T, T> transitionExtraFn,
      ToDoubleBiFunction<T, T> distanceFn,
      double sameEps,
      double stillSec,
      double robotFlickerSec) {

    final double now = Timer.getFPGATimestamp();
    final double dt = lastUpdateSec > 0.0 ? clampDt(now - lastUpdateSec) : 0.02;
    lastUpdateSec = now;

    evictEma(now);

    final Predicate<T> valid = validFn != null ? validFn : (x -> true);
    final ToDoubleFunction<T> score = scoreFn != null ? scoreFn : (x -> 0.0);

    double motionBoost = 0.0;
    if (Double.isFinite(robotFlickerSec) && robotFlickerSec > 0.0) {
      motionBoost = Math.max(motionBoost, Math.min(1.0, robotFlickerSec / 0.40));
    }

    boolean bestOkRaw0 = best != null && valid.test(best);
    boolean stickyOkRaw0 = sticky != null && valid.test(sticky);

    double baseEps = Math.max(0.0, sameEps);
    double eqEps = adaptiveEps(baseEps, 0.0, motionBoost);
    double idEps = Math.max(eqEps, 0.12);

    best = bestOkRaw0 ? canonicalize(now, best, distanceFn, idEps) : best;
    if (stickyOkRaw0) sticky = canonicalize(now, sticky, distanceFn, idEps);
    if (candidate != null) candidate = canonicalize(now, candidate, distanceFn, idEps);
    if (lastSticky != null) lastSticky = canonicalize(now, lastSticky, distanceFn, idEps);
    if (lastBestSeen != null) lastBestSeen = canonicalize(now, lastBestSeen, distanceFn, idEps);
    if (pingA != null) pingA = canonicalize(now, pingA, distanceFn, idEps);
    if (pingB != null) pingB = canonicalize(now, pingB, distanceFn, idEps);

    boolean bestOkRaw = best != null && valid.test(best);
    if (bestOkRaw) {
      lastSeenSec.put(best, now);
      boolean sameKey =
          bestValidKey != null
              && (Objects.equals(best, bestValidKey) || eq(best, bestValidKey, distanceFn, idEps));
      if (!sameKey) {
        bestValidKey = best;
        bestValidSinceSec = now;
      } else if (bestValidSinceSec < 0.0) {
        bestValidSinceSec = now;
      }
      bestMissingSinceSec = -1e9;
    } else {
      bestValidKey = null;
      bestValidSinceSec = -1e9;
      if (bestMissingSinceSec < 0.0) bestMissingSinceSec = now;
    }

    boolean bestOkDebounced = bestOkRaw && ((now - bestValidSinceSec) >= BEST_VALID_DEBOUNCE_SEC);
    boolean bestOk = bestOkDebounced || (sticky == null && bestOkRaw);

    boolean stickyGeomOk = sticky != null && valid.test(sticky);
    boolean stickySeenOk = sticky != null && seenRecently(now, sticky);
    boolean stickyOk = stickyGeomOk && stickySeenOk;

    // Logger.recordOutput("sticky_best_ok_raw", bestOkRaw);
    // Logger.recordOutput("sticky_best_ok_debounced", bestOkDebounced);
    // Logger.recordOutput("sticky_best_ok", bestOk);
    // Logger.recordOutput("sticky_seen_timeout_s", seenTimeoutSec);
    // Logger.recordOutput(
    // "sticky_best_missing_age_s",
    // bestMissingSinceSec > 0.0 ? (now - bestMissingSinceSec) : -1.0);
    // Logger.recordOutput("sticky_sticky_seen_age_s", sticky != null ? seenAge(now, sticky) :
    // -1.0);
    // Logger.recordOutput("sticky_best_seen_age_s", best != null ? seenAge(now, best) : -1.0);

    // Logger.recordOutput("bestOk", bestOk);
    // Logger.recordOutput("stickNull", sticky == null);
    // Logger.recordOutput("stickyOk", stickyOk);

    if (!bestOk) {
      boolean brieflyMissing =
          bestMissingSinceSec > 0.0 && (now - bestMissingSinceSec) < BEST_MISSING_DEBOUNCE_SEC;

      if (sticky != null) {
        if (!stickyGeomOk || !stickySeenOk) {
          if (stickyInvalidSinceSec < 0.0) stickyInvalidSinceSec = now;
        } else {
          stickyInvalidSinceSec = -1e9;
        }

        double invalidAge = stickyInvalidSinceSec > 0.0 ? (now - stickyInvalidSinceSec) : 0.0;

        // Logger.recordOutput(
        // "sticky_invalid_age_s", stickyInvalidSinceSec > 0.0 ? invalidAge : -1.0);

        if (stickyGeomOk && stickySeenOk) {
          recordOut(now, sticky, distanceFn, idEps);
          return sticky;
        }

        if (invalidAge < STICKY_INVALID_DEBOUNCE_SEC) {
          recordOut(now, sticky, distanceFn, idEps);
          return sticky;
        }

        double missingAge = bestMissingSinceSec > 0.0 ? (now - bestMissingSinceSec) : 0.0;
        boolean allowHold =
            brieflyMissing
                || missingAge < BEST_MISSING_DROP_SEC
                || (maxStaleSec > 1e-6 && (now - stickySinceSec) < Math.max(0.10, maxStaleSec));

        if (allowHold) {
          recordOut(now, sticky, distanceFn, idEps);
          return sticky;
        }
      }

      sticky = null;
      candidate = null;
      lastBestSeen = null;
      bestLastChangeSec = -1e9;
      flickerSinceSec = -1e9;
      hardLockUntilSec = -1e9;
      lastSticky = null;
      lastSwitchSec = -1e9;
      pingA = null;
      pingB = null;
      pingSinceSec = -1e9;
      pingBlockUntilSec = -1e9;
      bestMissingSinceSec = bestMissingSinceSec < 0.0 ? now : bestMissingSinceSec;
      return fallbackOut(now, valid, distanceFn, idEps);
    }

    boolean stickyPresenceBad = sticky != null && stickyGeomOk && !stickySeenOk;
    if (stickyPresenceBad && stickyInvalidSinceSec < 0.0) stickyInvalidSinceSec = now;

    if (sticky != null && (!stickyGeomOk || !stickySeenOk)) {
      if (stickyInvalidSinceSec < 0.0) stickyInvalidSinceSec = now;
      double invalidAge = now - stickyInvalidSinceSec;
      // Logger.recordOutput("sticky_invalid_age_s", invalidAge);

      if (invalidAge < STICKY_INVALID_DEBOUNCE_SEC) {
        recordOut(now, sticky, distanceFn, idEps);
        return sticky;
      }

      double switchBackEps = Math.max(SWITCH_BACK_EPS_MIN, Math.max(0.0, baseEps) * 3.0);
      boolean blocked =
          pingBlocked(now, best, distanceFn, switchBackEps)
              || (lastSticky != null
                  && eq(best, lastSticky, distanceFn, switchBackEps)
                  && (now - lastSwitchSec) < switchBackCooldownSec);

      if (blocked) {
        recordOut(now, sticky, distanceFn, idEps);
        return sticky;
      }

      T out = commitSwitch(now, best, distanceFn, switchBackEps);
      lastBestSeen = best;
      bestLastChangeSec = now;
      recordOut(now, out, distanceFn, idEps);
      return out;
    } else {
      stickyInvalidSinceSec = -1e9;
    }

    if (sticky == null) {
      sticky = best;
      stickySinceSec = now;
      stickyLastBestSeenSec = now;
      candidate = null;
      candidateSinceSec = -1e9;
      lastBestSeen = best;
      bestLastChangeSec = now;
      flickerSinceSec = -1e9;
      stickyInvalidSinceSec = -1e9;
      hardLockUntilSec = -1e9;
      pingA = null;
      pingB = null;
      pingSinceSec = -1e9;
      pingBlockUntilSec = -1e9;
      lastSeenSec.put(best, now);
      recordOut(now, sticky, distanceFn, idEps);
      return sticky;
    }

    boolean bestChanged;
    if (lastBestSeen == null) {
      bestChanged = true;
      lastBestSeen = best;
      bestLastChangeSec = now;
      flickerSinceSec = -1e9;
    } else {
      double changeEps = adaptiveEps(baseEps, 0.35, motionBoost);
      bestChanged = !eq(best, lastBestSeen, distanceFn, changeEps);
      if (bestChanged) {
        double delta = bestLastChangeSec > 0.0 ? (now - bestLastChangeSec) : 1e9;
        bestLastChangeSec = now;

        if (delta <= FLICKER_DELTA_SEC) {
          if (flickerSinceSec < 0.0) flickerSinceSec = now - delta;
        } else if (delta >= FLICKER_RESET_STABLE_SEC) {
          flickerSinceSec = -1e9;
        }
        lastBestSeen = best;
      } else {
        if (bestLastChangeSec > 0.0 && (now - bestLastChangeSec) >= FLICKER_RESET_STABLE_SEC) {
          flickerSinceSec = -1e9;
        }
      }
    }

    boolean bestIsSticky = eq(best, sticky, distanceFn, adaptiveEps(baseEps, 0.0, motionBoost));

    if (bestIsSticky) {
      stickyLastBestSeenSec = now;
      candidate = null;
      candidateSinceSec = -1e9;
      flickerSinceSec = -1e9;
      if (Objects.equals(best, sticky)) sticky = best;
      lastSeenSec.put(sticky, now);
      recordOut(now, sticky, distanceFn, idEps);
      return sticky;
    }

    double candIdEps = adaptiveEps(baseEps, 0.65, motionBoost);
    if (candidate == null || !eq(candidate, best, distanceFn, candIdEps)) {
      candidate = best;
      candidateSinceSec = now;
    }

    double bestRaw;
    if (scoreFn != null) bestRaw = score.applyAsDouble(best);
    else bestRaw = bestScore;

    double stickyRaw = score.applyAsDouble(sticky);

    bestRaw = safeScore(bestRaw);
    stickyRaw = safeScore(stickyRaw);

    double bestS = ema(now, best, bestRaw, dt);
    double stickyS = ema(now, sticky, stickyRaw, dt);

    double extra = 0.0;
    if (transitionExtraFn != null)
      extra = Math.max(0.0, safeScore(transitionExtraFn.applyAsDouble(sticky, best)));

    double advantage = (bestS - stickyS) - extra;

    double candAge = now - candidateSinceSec;
    double bestStableAge = bestLastChangeSec > 0.0 ? (now - bestLastChangeSec) : candAge;

    boolean candStable = candAge >= candidateStableSec;
    boolean candStableShort = candAge >= Math.min(candidateStableSec, 0.22);
    boolean bestStableEnough = bestStableAge >= MIN_SWITCH_STABLE_SEC;

    boolean holdPassed = (now - stickySinceSec) >= Math.max(0.0, minHoldSec);
    boolean stickyStale = (now - stickyLastBestSeenSec) >= maxStaleSec;
    boolean forcedRefresh = (now - stickySinceSec) >= maxStaleSec;

    boolean flickerLong =
        flickerSinceSec > 0.0 && (now - flickerSinceSec) >= FORCE_SWITCH_FLICKER_SEC;
    boolean flickeringNow =
        bestLastChangeSec > 0.0 && (now - bestLastChangeSec) <= FLICKER_DELTA_SEC;

    boolean stillLong = stillSec >= FORCE_SWITCH_STILL_SEC;
    boolean robotFlickerLong = robotFlickerSec >= FORCE_SWITCH_FLICKER_SEC;

    double switchBackEps = Math.max(SWITCH_BACK_EPS_MIN, Math.max(0.0, baseEps) * 3.0);
    boolean isSwitchBack = lastSticky != null && eq(best, lastSticky, distanceFn, switchBackEps);
    boolean switchBackBlocked = isSwitchBack && (now - lastSwitchSec) < switchBackCooldownSec;

    double pingEps = Math.max(switchBackEps, adaptiveEps(baseEps, 0.75, motionBoost));
    boolean pingBlock = pingBlocked(now, best, distanceFn, pingEps);

    boolean hardLocked = now < hardLockUntilSec;

    if ((flickerLong && flickeringNow) || robotFlickerLong) {
      hardLockUntilSec = Math.max(hardLockUntilSec, now + FLICKER_FREEZE_EXTEND_SEC);
      hardLocked = true;
    }

    double pairDist = dist(sticky, best, distanceFn);
    double closeThresh = Math.max(0.0, baseEps) * 2.0 + CLOSE_PAIR_DIST_ADD;
    boolean closePair = pairDist > 0.0 && pairDist <= closeThresh;

    double forceReq = Math.max(0.01, Math.min(keepMargin, 0.10) * 0.5);

    boolean freezeOnFlicker = (flickerLong && flickeringNow) || robotFlickerLong;
    boolean forceMoveOn =
        !freezeOnFlicker
            && bestStableEnough
            && candStableShort
            && stillLong
            && advantage >= forceReq
            && !switchBackBlocked
            && !pingBlock
            && !hardLocked;

    boolean shouldSwitch = false;

    if (forceMoveOn) {
      shouldSwitch = true;
    } else if (!freezeOnFlicker
        && bestStableEnough
        && candStableShort
        && advantage >= immediateDelta) {
      shouldSwitch = true;
    } else if (!switchBackBlocked && !freezeOnFlicker) {
      if (bestStableEnough && holdPassed && candStable && advantage >= keepMargin)
        shouldSwitch = true;
      else if (bestStableEnough
          && candStable
          && (stickyStale || forcedRefresh)
          && advantage >= Math.max(0.02, keepMargin * 0.25)) shouldSwitch = true;
    } else {
      if (bestStableEnough && candStable && advantage >= (immediateDelta * 1.65))
        shouldSwitch = true;
    }

    if (shouldSwitch) {
      double sinceSwitch = now - lastSwitchSec;
      double req = Math.max(immediateDelta, keepMargin) * EARLY_SWITCH_MULT;

      if (closePair) req *= CLOSE_PAIR_REQ_MULT;

      if (hardLocked) req *= 2.10;
      else if (lastSwitchSec > 0.0 && sinceSwitch < BASE_HARD_LOCK_SEC) req *= 1.45;

      if (isSwitchBack) {
        req *= 2.15;
        if (!candStable) req *= 1.35;
        if (!bestStableEnough) req *= 2.0;
      }

      if (pairDist >= 2.0) req *= 1.25;
      if (pingBlock) req *= 2.25;
      if (freezeOnFlicker) req *= 3.0;

      double hardOverride =
          Math.max(req, Math.max(immediateDelta, keepMargin) * HARD_LOCK_OVERRIDE_MULT);
      if (hardLocked && !forcedRefresh && advantage < hardOverride) {
        shouldSwitch = false;
      } else if (advantage < req
          && !(forcedRefresh
              && candStable
              && bestStableEnough
              && advantage >= Math.max(keepMargin, immediateDelta) * 1.45)) {
        shouldSwitch = false;
      }
    }

    // Logger.recordOutput("sticky_hard_lock_until_s", hardLockUntilSec);
    // Logger.recordOutput("sticky_hard_locked", hardLocked);
    // Logger.recordOutput("flickerLong", flickerLong);
    // Logger.recordOutput("flickeringNow", flickeringNow);
    // Logger.recordOutput("stillLong", stillLong);
    // Logger.recordOutput("robotFlickerLong", robotFlickerLong);
    // Logger.recordOutput("advantage", advantage);
    // Logger.recordOutput("forceReq", forceReq);
    // Logger.recordOutput("immediateDelta", immediateDelta);
    // Logger.recordOutput("keepMargin", keepMargin);
    // Logger.recordOutput("candStableShort", candStableShort);
    // Logger.recordOutput("candStable", candStable);
    // Logger.recordOutput("bestStableEnough", bestStableEnough);
    // Logger.recordOutput("bestStableAge_s", bestStableAge);
    // Logger.recordOutput("switchBackBlocked", switchBackBlocked);
    // Logger.recordOutput("forceMoveOn", forceMoveOn);
    // Logger.recordOutput("holdPassed", holdPassed);
    // Logger.recordOutput("stickyStale", stickyStale);
    // Logger.recordOutput("forcedRefresh", forcedRefresh);
    // Logger.recordOutput("shouldSwitch", shouldSwitch);
    // Logger.recordOutput("ping_block_until_s", pingBlockUntilSec);
    // Logger.recordOutput("ping_blocked_now", pingBlock);
    // Logger.recordOutput("freezeOnFlicker", freezeOnFlicker);
    // Logger.recordOutput("eq_eps_used", idEps);
    // Logger.recordOutput("closePair", closePair);
    // Logger.recordOutput("pairDist", pairDist);

    // Logger.recordOutput("sticky_best_hash", best != null ? best.hashCode() : 0);
    // Logger.recordOutput("sticky_sticky_hash", sticky != null ? sticky.hashCode() : 0);
    // Logger.recordOutput("sticky_candidate_hash", candidate != null ? candidate.hashCode() : 0);
    // Logger.recordOutput("sticky_lastSticky_hash", lastSticky != null ? lastSticky.hashCode() :
    // 0);
    // Logger.recordOutput("sticky_best_sticky_dist_m", dist(best, sticky, distanceFn));
    // Logger.recordOutput("sticky_eq_best_sticky", eq(best, sticky, distanceFn, idEps));
    // Logger.recordOutput("sticky_same_eps", sameEps);

    if (shouldSwitch) {
      T out = commitSwitch(now, best, distanceFn, pingEps);
      recordOut(now, out, distanceFn, idEps);
      return out;
    }

    recordOut(now, sticky, distanceFn, idEps);
    return sticky;
  }

  private void recordOut(double now, T out, ToDoubleBiFunction<T, T> distanceFn, double sameEps) {
    boolean changed = !eqN(lastOut, out, distanceFn, sameEps);
    if (changed) lastOutChangeSec = now;
    // Logger.recordOutput("sticky_out_changed", changed);
    // Logger.recordOutput(
    // "sticky_out_changed_age_s", lastOutChangeSec > 0.0 ? (now - lastOutChangeSec) : -1.0);
    lastOut = out;
  }
}
