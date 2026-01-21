package org.curtinfrc.frc2026.util.Repulsor;

import edu.wpi.first.wpilibj.Timer;
import java.util.HashMap;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.function.Predicate;
import java.util.function.ToDoubleBiFunction;
import java.util.function.ToDoubleFunction;
import org.littletonrobotics.junction.Logger;

public final class StickyTarget<T> {
  private final double candidateStableSec;
  private final double maxStaleSec;
  private final double switchBackCooldownSec;

  private static final double FORCE_SWITCH_STILL_SEC = 0.20;
  private static final double FORCE_SWITCH_FLICKER_SEC = 0.30;

  private static final double STICKY_INVALID_DEBOUNCE_SEC = 0.65;

  private static final double BASE_HARD_LOCK_SEC = 0.45;
  private static final double HARD_LOCK_PER_M_SEC = 0.28;
  private static final double HARD_LOCK_MAX_SEC = 2.10;

  private static final double EARLY_SWITCH_MULT = 2.25;
  private static final double SWITCH_BACK_EPS_MIN = 0.65;

  private static final double PINGPONG_WINDOW_SEC = 0.80;
  private static final double PINGPONG_BLOCK_SEC = 1.10;

  private T lastOut;
  private double lastOutChangeSec = -1e9;

  private double stickyInvalidSinceSec = -1e9;

  private T lastBestSeen;
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

  private final Map<T, Double> emaScore = new HashMap<>(8);
  private final double emaTauSec = 0.18;

  public StickyTarget(double candidateStableSec, double maxStaleSec, double switchBackCooldownSec) {
    this.candidateStableSec = Math.max(0.0, candidateStableSec);
    this.maxStaleSec = Math.max(0.0, maxStaleSec);
    this.switchBackCooldownSec = Math.max(0.0, switchBackCooldownSec);
    clear();
  }

  public Optional<T> get() {
    return Optional.ofNullable(sticky);
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
    flickerSinceSec = -1e9;
    stickyInvalidSinceSec = -1e9;
    hardLockUntilSec = -1e9;
    pingA = null;
    pingB = null;
    pingSinceSec = -1e9;
    pingBlockUntilSec = -1e9;
  }

  private double clampDt(double dt) {
    if (dt < 1e-3) return 1e-3;
    if (dt > 0.10) return 0.10;
    return dt;
  }

  private double ema(T key, double raw, double dt) {
    if (key == null) return raw;
    double prev = emaScore.getOrDefault(key, raw);
    double tau = Math.max(1e-3, emaTauSec);
    double a = 1.0 - Math.exp(-dt / tau);
    double v = prev + (raw - prev) * a;
    emaScore.put(key, v);
    return v;
  }

  private boolean eq(T a, T b, ToDoubleBiFunction<T, T> distanceFn, double sameEps) {
    if (a == null || b == null) return false;
    if (Objects.equals(a, b)) return true;
    if (distanceFn == null) return false;
    return distanceFn.applyAsDouble(a, b) <= Math.max(0.0, sameEps);
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

  private void armHardLock(double now, double switchDistM) {
    double lock = BASE_HARD_LOCK_SEC + HARD_LOCK_PER_M_SEC * Math.max(0.0, switchDistM);
    if (lock > HARD_LOCK_MAX_SEC) lock = HARD_LOCK_MAX_SEC;
    if (lock < BASE_HARD_LOCK_SEC) lock = BASE_HARD_LOCK_SEC;
    hardLockUntilSec = now + lock;
  }

  private void notePingPong(double now, T from, T to, ToDoubleBiFunction<T, T> distanceFn) {
    if (from == null || to == null) return;

    if (pingA == null || pingB == null) {
      pingA = from;
      pingB = to;
      pingSinceSec = now;
      return;
    }

    boolean sameAB = Objects.equals(from, pingA) && Objects.equals(to, pingB);
    boolean sameBA = Objects.equals(from, pingB) && Objects.equals(to, pingA);

    if (sameAB || sameBA) {
      double age = now - pingSinceSec;
      if (age <= PINGPONG_WINDOW_SEC) {
        pingBlockUntilSec = Math.max(pingBlockUntilSec, now + PINGPONG_BLOCK_SEC);
      } else {
        pingSinceSec = now;
      }
      return;
    }

    pingA = from;
    pingB = to;
    pingSinceSec = now;
  }

  private boolean pingBlocked(
      double now, T next, ToDoubleBiFunction<T, T> distanceFn, double sameEps) {
    if (now >= pingBlockUntilSec) return false;
    if (next == null) return false;
    if (pingA != null && eq(next, pingA, distanceFn, Math.max(sameEps, SWITCH_BACK_EPS_MIN)))
      return true;
    if (pingB != null && eq(next, pingB, distanceFn, Math.max(sameEps, SWITCH_BACK_EPS_MIN)))
      return true;
    return false;
  }

  private T commitSwitch(double now, T next, ToDoubleBiFunction<T, T> distanceFn) {
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
    notePingPong(now, prev, sticky, distanceFn);

    return sticky;
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

    final Predicate<T> valid = validFn != null ? validFn : (x -> true);
    final ToDoubleFunction<T> score = scoreFn != null ? scoreFn : (x -> 0.0);

    boolean bestOk = best != null && valid.test(best);
    boolean stickyOk = sticky != null && valid.test(sticky);

    if (!bestOk) {
      if (stickyOk) {
        recordOut(now, sticky, distanceFn, sameEps);
        return sticky;
      }
      sticky = null;
      candidate = null;
      lastBestSeen = null;
      flickerSinceSec = -1e9;
      stickyInvalidSinceSec = -1e9;
      hardLockUntilSec = -1e9;
      recordOut(now, sticky, distanceFn, sameEps);
      return best;
    }

    if (sticky != null && !stickyOk) {
      if (stickyInvalidSinceSec < 0.0) stickyInvalidSinceSec = now;
      double invalidAge = now - stickyInvalidSinceSec;
      Logger.recordOutput("sticky_invalid_age_s", invalidAge);
      if (invalidAge < STICKY_INVALID_DEBOUNCE_SEC) {
        recordOut(now, sticky, distanceFn, sameEps);
        return sticky;
      }

      double switchBackEps = Math.max(SWITCH_BACK_EPS_MIN, Math.max(0.0, sameEps) * 3.0);
      boolean blocked =
          pingBlocked(now, best, distanceFn, sameEps)
              || (lastSticky != null
                  && eq(best, lastSticky, distanceFn, switchBackEps)
                  && (now - lastSwitchSec) < switchBackCooldownSec);

      if (blocked) {
        recordOut(now, sticky, distanceFn, sameEps);
        return sticky;
      }

      T out = commitSwitch(now, best, distanceFn);
      recordOut(now, out, distanceFn, sameEps);
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
      flickerSinceSec = -1e9;
      stickyInvalidSinceSec = -1e9;
      hardLockUntilSec = -1e9;
      pingA = null;
      pingB = null;
      pingSinceSec = -1e9;
      pingBlockUntilSec = -1e9;
      recordOut(now, sticky, distanceFn, sameEps);
      return sticky;
    }

    boolean bestChanged = (lastBestSeen != null) && !eq(best, lastBestSeen, distanceFn, sameEps);
    lastBestSeen = best;

    boolean bestIsSticky = eq(best, sticky, distanceFn, sameEps);

    if (!bestIsSticky && bestChanged) {
      if (flickerSinceSec < 0.0) flickerSinceSec = now;
    } else if (bestIsSticky) {
      flickerSinceSec = -1e9;
    }

    if (bestIsSticky) {
      stickyLastBestSeenSec = now;
      candidate = null;
      candidateSinceSec = -1e9;
      if (Objects.equals(best, sticky)) sticky = best;
      recordOut(now, sticky, distanceFn, sameEps);
      return sticky;
    }

    if (candidate == null || !eq(candidate, best, distanceFn, sameEps)) {
      candidate = best;
      candidateSinceSec = now;
    }

    double bestS = ema(best, bestScore, dt);
    double stickyRaw = score.applyAsDouble(sticky);
    double stickyS = ema(sticky, stickyRaw, dt);

    double extra = 0.0;
    if (transitionExtraFn != null)
      extra = Math.max(0.0, transitionExtraFn.applyAsDouble(sticky, best));

    double advantage = (bestS - stickyS) - extra;

    double candAge = now - candidateSinceSec;
    boolean candStable = candAge >= candidateStableSec;
    boolean candStableShort = candAge >= Math.min(candidateStableSec, 0.18);

    boolean holdPassed = (now - stickySinceSec) >= Math.max(0.0, minHoldSec);
    boolean stickyStale = (now - stickyLastBestSeenSec) >= maxStaleSec;
    boolean forcedRefresh = (now - stickySinceSec) >= maxStaleSec;

    boolean flickerLong =
        flickerSinceSec > 0.0 && (now - flickerSinceSec) >= FORCE_SWITCH_FLICKER_SEC;
    boolean stillLong = stillSec >= FORCE_SWITCH_STILL_SEC;
    boolean robotFlickerLong = robotFlickerSec >= FORCE_SWITCH_FLICKER_SEC;

    double switchBackEps = Math.max(SWITCH_BACK_EPS_MIN, Math.max(0.0, sameEps) * 3.0);
    boolean isSwitchBack = lastSticky != null && eq(best, lastSticky, distanceFn, switchBackEps);

    boolean switchBackBlocked = isSwitchBack && (now - lastSwitchSec) < switchBackCooldownSec;

    double forceReq = Math.max(0.01, Math.min(keepMargin, 0.10) * 0.5);

    boolean forceMoveOn =
        (stillLong || flickerLong || robotFlickerLong)
            && advantage >= forceReq
            && (!switchBackBlocked
                || advantage >= (forceReq + 0.15 * Math.max(0.0, immediateDelta)));

    boolean shouldSwitch = false;

    if (forceMoveOn) {
      shouldSwitch = true;
    } else if (candStableShort && advantage >= immediateDelta) {
      shouldSwitch = true;
    } else if (!switchBackBlocked) {
      if (holdPassed && candStable && advantage >= keepMargin) shouldSwitch = true;
      else if (candStable
          && (stickyStale || forcedRefresh)
          && advantage >= Math.max(0.02, keepMargin * 0.25)) shouldSwitch = true;
    } else {
      if (candStable && advantage >= (immediateDelta * 1.25)) shouldSwitch = true;
    }

    boolean hardLocked = now < hardLockUntilSec;

    if (shouldSwitch) {
      double sinceSwitch = now - lastSwitchSec;
      double req = Math.max(immediateDelta, keepMargin) * EARLY_SWITCH_MULT;
      if (hardLocked) req *= 1.55;
      else if (lastSwitchSec > 0.0 && sinceSwitch < BASE_HARD_LOCK_SEC) req *= 1.35;
      if (isSwitchBack) req *= 1.75;
      if (dist(sticky, best, distanceFn) >= 2.0) req *= 1.25;

      if (pingBlocked(now, best, distanceFn, sameEps)) req *= 2.0;

      if (advantage < req
          && !(forcedRefresh
              && candStable
              && advantage >= Math.max(keepMargin, immediateDelta) * 1.35)) {
        shouldSwitch = false;
      }
    }

    Logger.recordOutput("sticky_hard_lock_until_s", hardLockUntilSec);
    Logger.recordOutput("sticky_hard_locked", hardLocked);
    Logger.recordOutput("flickerLong", flickerLong);
    Logger.recordOutput("stillLong", stillLong);
    Logger.recordOutput("robotFlickerLong", robotFlickerLong);
    Logger.recordOutput("advantage", advantage);
    Logger.recordOutput("forceReq", forceReq);
    Logger.recordOutput("immediateDelta", immediateDelta);
    Logger.recordOutput("keepMargin", keepMargin);
    Logger.recordOutput("candStableShort", candStableShort);
    Logger.recordOutput("switchBackBlocked", switchBackBlocked);
    Logger.recordOutput("forceMoveOn", forceMoveOn);
    Logger.recordOutput("holdPassed", holdPassed);
    Logger.recordOutput("candStable", candStable);
    Logger.recordOutput("stickyStale", stickyStale);
    Logger.recordOutput("forcedRefresh", forcedRefresh);
    Logger.recordOutput("shouldSwitch", shouldSwitch);
    Logger.recordOutput("ping_block_until_s", pingBlockUntilSec);

    if (shouldSwitch) {
      T out = commitSwitch(now, best, distanceFn);
      recordOut(now, out, distanceFn, sameEps);
      return out;
    }

    recordOut(now, sticky, distanceFn, sameEps);
    return sticky;
  }

  private void recordOut(double now, T out, ToDoubleBiFunction<T, T> distanceFn, double sameEps) {
    boolean changed = !eqN(lastOut, out, distanceFn, sameEps);
    if (changed) lastOutChangeSec = now;
    Logger.recordOutput("sticky_out_changed", changed);
    Logger.recordOutput(
        "sticky_out_changed_age_s", lastOutChangeSec > 0.0 ? (now - lastOutChangeSec) : -1.0);
    lastOut = out;
  }
}
