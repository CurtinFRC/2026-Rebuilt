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

package org.curtinfrc.frc2026.util.Repulsor.Target;

import edu.wpi.first.wpilibj.Timer;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Predicate;
import java.util.function.ToDoubleBiFunction;
import java.util.function.ToDoubleFunction;

public final class StickyTarget<T> {
  private final double candidateStableSec;
  private final double maxStaleSec;
  private final double switchBackCooldownSec;
  private final DoubleSupplier nowSecSupplier;

  private final StickyTargetState<T> st = new StickyTargetState<>();
  private final StickyTargetEma<T> ema = new StickyTargetEma<>();
  private final StickyTargetSeen<T> seen = new StickyTargetSeen<>();
  private final StickyTargetCanon<T> canon = new StickyTargetCanon<>();
  private final StickyTargetHardLock hardLock = new StickyTargetHardLock();
  private final StickyTargetPingPong<T> pingPong = new StickyTargetPingPong<>();

  public StickyTarget(double candidateStableSec, double maxStaleSec, double switchBackCooldownSec) {
    this(candidateStableSec, maxStaleSec, switchBackCooldownSec, Timer::getFPGATimestamp);
  }

  StickyTarget(
      double candidateStableSec,
      double maxStaleSec,
      double switchBackCooldownSec,
      DoubleSupplier nowSecSupplier) {
    this.candidateStableSec = Math.max(0.0, candidateStableSec);
    this.maxStaleSec = Math.max(0.0, maxStaleSec);
    this.switchBackCooldownSec = Math.max(0.0, switchBackCooldownSec);
    this.nowSecSupplier = nowSecSupplier != null ? nowSecSupplier : Timer::getFPGATimestamp;

    double base =
        this.maxStaleSec > 1e-6 ? this.maxStaleSec : TargetConfig.DEFAULT_SEEN_TIMEOUT_SEC;
    setSeenTimeoutSec(Math.max(0.12, Math.min(2.5, base)));

    clear();
  }

  private double nowSec() {
    double now = nowSecSupplier.getAsDouble();
    return Double.isFinite(now) ? now : Timer.getFPGATimestamp();
  }

  public Optional<T> get() {
    return Optional.ofNullable(st.sticky);
  }

  public void setSeenTimeoutSec(double sec) {
    if (!Double.isFinite(sec)) return;
    seen.setTimeoutSec(Math.max(0.12, sec));
  }

  public void setEmaEvictSec(double sec) {
    if (!Double.isFinite(sec)) return;
    ema.setEvictSec(Math.max(0.25, sec));
  }

  public void noteSeen(T value) {
    if (value == null) return;
    double now = nowSec();
    seen.noteSeen(now, value);
  }

  public void noteSeen(Iterable<T> values) {
    if (values == null) return;
    double now = nowSec();
    seen.noteSeen(now, values);
  }

  public void clear() {
    st.clear();
    ema.clear();
    seen.clear();
    canon.clear();
    hardLock.clear();
    pingPong.clear();
  }

  public void force(T value) {
    double now = nowSec();

    st.sticky = value;
    st.candidate = null;
    st.candidateSinceSec = -1e9;
    st.stickySinceSec = now;
    st.stickyLastBestSeenSec = now;
    st.lastSticky = null;
    st.lastSwitchSec = -1e9;
    st.lastBestSeen = value;
    st.bestLastChangeSec = now;
    st.flickerSinceSec = -1e9;
    st.stickyInvalidSinceSec = -1e9;

    hardLock.clear();
    pingPong.clear();

    st.bestMissingSinceSec = -1e9;
    st.bestValidSinceSec = now;
    st.bestValidKey = value;

    if (value != null) {
      seen.noteSeen(now, value);
      canon.add(now, value);
    }
  }

  public void forceInvalidate() {
    double now = nowSec();

    st.sticky = null;
    st.candidate = null;
    st.lastSticky = null;
    st.lastSwitchSec = now;
    st.stickySinceSec = -1e9;
    st.stickyLastBestSeenSec = -1e9;
    st.candidateSinceSec = -1e9;

    st.bestMissingSinceSec = -1e9;
    st.bestValidSinceSec = -1e9;
    st.bestValidKey = null;

    st.lastOut = null;
    st.lastOutChangeSec = now;

    hardLock.clear();
    pingPong.clear();
    ema.clear();
    seen.clear();
  }

  private T commitSwitch(double now, T next, ToDoubleBiFunction<T, T> distanceFn, double eps) {
    T prev = st.sticky;

    st.lastSticky = prev;
    st.lastSwitchSec = now;

    st.sticky = next;
    st.stickySinceSec = now;
    st.stickyLastBestSeenSec = now;

    st.candidate = null;
    st.candidateSinceSec = -1e9;
    st.flickerSinceSec = -1e9;
    st.stickyInvalidSinceSec = -1e9;

    double d = StickyTargetMath.dist(prev, st.sticky, distanceFn);
    hardLock.arm(now, d);

    pingPong.noteSwitch(now, prev, st.sticky, distanceFn, eps, d);

    if (st.sticky != null) seen.noteSeen(now, st.sticky);

    return st.sticky;
  }

  private T fallbackOut(
      double now, Predicate<T> valid, ToDoubleBiFunction<T, T> distanceFn, double sameEps) {

    if (st.sticky != null) return st.sticky;

    if (st.lastOut != null && valid.test(st.lastOut) && seen.seenRecently(now, st.lastOut)) {
      st.sticky = st.lastOut;
      st.stickySinceSec = now;
      st.stickyLastBestSeenSec = now;

      st.candidate = null;
      st.candidateSinceSec = -1e9;

      st.lastSticky = null;
      st.lastSwitchSec = -1e9;

      hardLock.clear();
      pingPong.clear();

      seen.noteSeen(now, st.lastOut);

      recordOut(now, st.sticky, distanceFn, sameEps);
      return st.sticky;
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

    final double now = nowSec();
    final double dt =
        st.lastUpdateSec > 0.0 ? StickyTargetMath.clampDt(now - st.lastUpdateSec) : 0.02;
    st.lastUpdateSec = now;

    ema.evict(now);

    final Predicate<T> valid = validFn != null ? validFn : (x -> true);
    final ToDoubleFunction<T> score = scoreFn != null ? scoreFn : (x -> 0.0);

    double motionBoost = 0.0;
    if (Double.isFinite(robotFlickerSec) && robotFlickerSec > 0.0) {
      motionBoost = Math.max(motionBoost, Math.min(1.0, robotFlickerSec / 0.40));
    }

    boolean bestOkRaw0 = best != null && valid.test(best);
    boolean stickyOkRaw0 = st.sticky != null && valid.test(st.sticky);

    double baseEps = Math.max(0.0, sameEps);
    double eqEps = StickyTargetMath.adaptiveEps(baseEps, 0.0, motionBoost);
    double idEps = Math.max(0.04, Math.min(0.10, eqEps));

    best = bestOkRaw0 ? canon.canonicalize(now, best, distanceFn, idEps) : best;
    if (stickyOkRaw0) st.sticky = canon.canonicalize(now, st.sticky, distanceFn, idEps);
    if (st.candidate != null)
      st.candidate = canon.canonicalize(now, st.candidate, distanceFn, idEps);
    if (st.lastSticky != null)
      st.lastSticky = canon.canonicalize(now, st.lastSticky, distanceFn, idEps);
    if (st.lastBestSeen != null)
      st.lastBestSeen = canon.canonicalize(now, st.lastBestSeen, distanceFn, idEps);
    pingPong.canonicalizePing(now, canon, distanceFn, idEps);

    boolean bestOkRaw = best != null && valid.test(best);
    if (bestOkRaw) {
      seen.noteSeen(now, best);

      boolean sameKey =
          st.bestValidKey != null
              && (java.util.Objects.equals(best, st.bestValidKey)
                  || StickyTargetMath.eq(best, st.bestValidKey, distanceFn, idEps));

      if (!sameKey) {
        st.bestValidKey = best;
        st.bestValidSinceSec = now;
      } else if (st.bestValidSinceSec < 0.0) {
        st.bestValidSinceSec = now;
      }

      st.bestMissingSinceSec = -1e9;
    } else {
      st.bestValidKey = null;
      st.bestValidSinceSec = -1e9;
      if (st.bestMissingSinceSec < 0.0) st.bestMissingSinceSec = now;
    }

    boolean bestOkDebounced =
        bestOkRaw && ((now - st.bestValidSinceSec) >= TargetConfig.BEST_VALID_DEBOUNCE_SEC);
    boolean bestOk = bestOkDebounced || (st.sticky == null && bestOkRaw);

    boolean stickyGeomOk = st.sticky != null && valid.test(st.sticky);
    boolean stickySeenOk = st.sticky != null && seen.seenRecently(now, st.sticky);
    boolean stickyOk = stickyGeomOk && stickySeenOk;

    if (!bestOk) {
      boolean brieflyMissing =
          st.bestMissingSinceSec > 0.0
              && (now - st.bestMissingSinceSec) < TargetConfig.BEST_MISSING_DEBOUNCE_SEC;

      if (st.sticky != null) {
        if (!stickyGeomOk || !stickySeenOk) {
          if (st.stickyInvalidSinceSec < 0.0) st.stickyInvalidSinceSec = now;
        } else {
          st.stickyInvalidSinceSec = -1e9;
        }

        double invalidAge = st.stickyInvalidSinceSec > 0.0 ? (now - st.stickyInvalidSinceSec) : 0.0;

        if (stickyGeomOk && stickySeenOk) {
          recordOut(now, st.sticky, distanceFn, idEps);
          return st.sticky;
        }

        if (invalidAge < TargetConfig.STICKY_INVALID_DEBOUNCE_SEC) {
          recordOut(now, st.sticky, distanceFn, idEps);
          return st.sticky;
        }

        double missingAge = st.bestMissingSinceSec > 0.0 ? (now - st.bestMissingSinceSec) : 0.0;
        boolean allowHold =
            brieflyMissing
                || missingAge < TargetConfig.BEST_MISSING_DROP_SEC
                || (maxStaleSec > 1e-6 && (now - st.stickySinceSec) < Math.max(0.10, maxStaleSec));

        if (allowHold) {
          recordOut(now, st.sticky, distanceFn, idEps);
          return st.sticky;
        }
      }

      st.sticky = null;
      st.candidate = null;
      st.lastBestSeen = null;
      st.bestLastChangeSec = -1e9;
      st.flickerSinceSec = -1e9;

      hardLock.clear();
      st.lastSticky = null;
      st.lastSwitchSec = -1e9;
      pingPong.clear();

      st.bestMissingSinceSec = st.bestMissingSinceSec < 0.0 ? now : st.bestMissingSinceSec;
      return fallbackOut(now, valid, distanceFn, idEps);
    }

    boolean stickyPresenceBad = st.sticky != null && stickyGeomOk && !stickySeenOk;
    if (stickyPresenceBad && st.stickyInvalidSinceSec < 0.0) st.stickyInvalidSinceSec = now;

    if (st.sticky != null && (!stickyGeomOk || !stickySeenOk)) {
      if (st.stickyInvalidSinceSec < 0.0) st.stickyInvalidSinceSec = now;
      double invalidAge = now - st.stickyInvalidSinceSec;

      if (invalidAge < TargetConfig.STICKY_INVALID_DEBOUNCE_SEC) {
        recordOut(now, st.sticky, distanceFn, idEps);
        return st.sticky;
      }

      double switchBackEps =
          Math.max(TargetConfig.SWITCH_BACK_EPS_MIN, Math.max(0.0, baseEps) * 3.0);
      boolean stickyGeomInvalid = !stickyGeomOk;
      boolean blocked =
          pingPong.isBlocked(now, best, distanceFn, switchBackEps)
              || (st.lastSticky != null
                  && StickyTargetMath.eq(best, st.lastSticky, distanceFn, switchBackEps)
                  && (now - st.lastSwitchSec) < switchBackCooldownSec);

      // If current sticky target is geometrically invalid (e.g., no longer a valid collect point),
      // do not let switch-back/ping-pong protection pin us to it.
      if (blocked && !stickyGeomInvalid) {
        recordOut(now, st.sticky, distanceFn, idEps);
        return st.sticky;
      }

      T out = commitSwitch(now, best, distanceFn, switchBackEps);
      st.lastBestSeen = best;
      st.bestLastChangeSec = now;

      recordOut(now, out, distanceFn, idEps);
      return out;
    } else {
      st.stickyInvalidSinceSec = -1e9;
    }

    if (st.sticky == null) {
      st.sticky = best;
      st.stickySinceSec = now;
      st.stickyLastBestSeenSec = now;

      st.candidate = null;
      st.candidateSinceSec = -1e9;

      st.lastBestSeen = best;
      st.bestLastChangeSec = now;
      st.flickerSinceSec = -1e9;

      st.stickyInvalidSinceSec = -1e9;

      hardLock.clear();
      pingPong.clear();

      seen.noteSeen(now, best);

      recordOut(now, st.sticky, distanceFn, idEps);
      return st.sticky;
    }

    boolean bestChanged;
    if (st.lastBestSeen == null) {
      bestChanged = true;
      st.lastBestSeen = best;
      st.bestLastChangeSec = now;
      st.flickerSinceSec = -1e9;
    } else {
      double changeEps = StickyTargetMath.adaptiveEps(baseEps, 0.35, motionBoost);
      bestChanged = !StickyTargetMath.eq(best, st.lastBestSeen, distanceFn, changeEps);
      if (bestChanged) {
        double delta = st.bestLastChangeSec > 0.0 ? (now - st.bestLastChangeSec) : 1e9;
        st.bestLastChangeSec = now;

        if (delta <= TargetConfig.FLICKER_DELTA_SEC) {
          if (st.flickerSinceSec < 0.0) st.flickerSinceSec = now - delta;
        } else if (delta >= TargetConfig.FLICKER_RESET_STABLE_SEC) {
          st.flickerSinceSec = -1e9;
        }

        st.lastBestSeen = best;
      } else {
        if (st.bestLastChangeSec > 0.0
            && (now - st.bestLastChangeSec) >= TargetConfig.FLICKER_RESET_STABLE_SEC) {
          st.flickerSinceSec = -1e9;
        }
      }
    }

    boolean bestIsSticky =
        StickyTargetMath.eq(
            best, st.sticky, distanceFn, StickyTargetMath.adaptiveEps(baseEps, 0.0, motionBoost));
    if (bestIsSticky) {
      st.stickyLastBestSeenSec = now;
      st.candidate = null;
      st.candidateSinceSec = -1e9;
      st.flickerSinceSec = -1e9;

      if (java.util.Objects.equals(best, st.sticky)) st.sticky = best;

      seen.noteSeen(now, st.sticky);

      recordOut(now, st.sticky, distanceFn, idEps);
      return st.sticky;
    }

    double candIdEps = StickyTargetMath.adaptiveEps(baseEps, 0.65, motionBoost);
    if (st.candidate == null || !StickyTargetMath.eq(st.candidate, best, distanceFn, candIdEps)) {
      st.candidate = best;
      st.candidateSinceSec = now;
    }

    double bestRaw = (scoreFn != null) ? score.applyAsDouble(best) : bestScore;
    double stickyRaw = score.applyAsDouble(st.sticky);

    bestRaw = StickyTargetMath.safeScore(bestRaw);
    stickyRaw = StickyTargetMath.safeScore(stickyRaw);

    double bestS = ema.update(now, best, bestRaw, dt);
    double stickyS = ema.update(now, st.sticky, stickyRaw, dt);

    double extra = 0.0;
    if (transitionExtraFn != null) {
      extra =
          Math.max(
              0.0, StickyTargetMath.safeScore(transitionExtraFn.applyAsDouble(st.sticky, best)));
    }

    double advantage = (bestS - stickyS) - extra;

    double candAge = now - st.candidateSinceSec;
    double bestStableAge = st.bestLastChangeSec > 0.0 ? (now - st.bestLastChangeSec) : candAge;

    boolean candStable = candAge >= candidateStableSec;
    boolean candStableShort = candAge >= Math.min(candidateStableSec, 0.22);
    boolean bestStableEnough = bestStableAge >= TargetConfig.MIN_SWITCH_STABLE_SEC;

    boolean holdPassed = (now - st.stickySinceSec) >= Math.max(0.0, minHoldSec);
    boolean stickyStale = (now - st.stickyLastBestSeenSec) >= maxStaleSec;
    boolean forcedRefresh = (now - st.stickySinceSec) >= maxStaleSec;

    boolean flickerLong =
        st.flickerSinceSec > 0.0
            && (now - st.flickerSinceSec) >= TargetConfig.FORCE_SWITCH_FLICKER_SEC;
    boolean flickeringNow =
        st.bestLastChangeSec > 0.0
            && (now - st.bestLastChangeSec) <= TargetConfig.FLICKER_DELTA_SEC;

    boolean stillLong = stillSec >= TargetConfig.FORCE_SWITCH_STILL_SEC;
    boolean robotFlickerLong = robotFlickerSec >= TargetConfig.FORCE_SWITCH_FLICKER_SEC;

    double switchBackEps = Math.max(TargetConfig.SWITCH_BACK_EPS_MIN, Math.max(0.0, baseEps) * 3.0);
    boolean isSwitchBack =
        st.lastSticky != null
            && StickyTargetMath.eq(best, st.lastSticky, distanceFn, switchBackEps);
    boolean switchBackBlocked = isSwitchBack && (now - st.lastSwitchSec) < switchBackCooldownSec;

    double pingEps =
        Math.max(switchBackEps, StickyTargetMath.adaptiveEps(baseEps, 0.75, motionBoost));
    boolean pingBlock = pingPong.isBlocked(now, best, distanceFn, pingEps);

    boolean hardLocked = hardLock.isLocked(now);

    if ((flickerLong && flickeringNow) || robotFlickerLong) {
      hardLock.extendToAtLeast(now + TargetConfig.FLICKER_FREEZE_EXTEND_SEC);
      hardLocked = true;
    }

    double pairDist = StickyTargetMath.dist(st.sticky, best, distanceFn);
    double closeThresh = Math.max(0.0, baseEps) * 2.0 + TargetConfig.CLOSE_PAIR_DIST_ADD;
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

    double closePairHoldSecReq =
        Math.max(0.0, candidateStableSec * TargetConfig.CLOSE_PAIR_HOLD_SCALE);
    double closePairAdvReq =
        Math.max(
            0.02,
            Math.min(immediateDelta, Math.max(keepMargin, 0.0))
                * TargetConfig.CLOSE_PAIR_ADV_REQ_MULT);
    boolean closePairAssist =
        closePair
            && !freezeOnFlicker
            && !switchBackBlocked
            && !pingBlock
            && !hardLocked
            && bestStableEnough
            && candAge >= closePairHoldSecReq
            && advantage >= closePairAdvReq;

    boolean shouldSwitch = false;
    boolean closeAssistSwitch = false;

    if (forceMoveOn) {
      shouldSwitch = true;
    } else if (closePairAssist) {
      shouldSwitch = true;
      closeAssistSwitch = true;
    } else if (!freezeOnFlicker
        && bestStableEnough
        && candStableShort
        && advantage >= immediateDelta) {
      shouldSwitch = true;
    } else if (!switchBackBlocked && !freezeOnFlicker) {
      if (bestStableEnough && holdPassed && candStable && advantage >= keepMargin) {
        shouldSwitch = true;
      } else if (bestStableEnough
          && candStable
          && (stickyStale || forcedRefresh)
          && advantage >= Math.max(0.02, keepMargin * 0.25)) {
        shouldSwitch = true;
      }
    } else {
      if (bestStableEnough && candStable && advantage >= (immediateDelta * 1.65)) {
        shouldSwitch = true;
      }
    }

    if (shouldSwitch) {
      double req;
      double hardOverride;
      if (closeAssistSwitch) {
        req = closePairAdvReq;
        hardOverride = closePairAdvReq * TargetConfig.CLOSE_PAIR_HARDLOCK_REQ_MULT;
      } else {
        double sinceSwitch = now - st.lastSwitchSec;
        req = Math.max(immediateDelta, keepMargin) * TargetConfig.EARLY_SWITCH_MULT;

        if (closePair) req *= TargetConfig.CLOSE_PAIR_REQ_MULT;

        if (hardLocked) req *= 2.10;
        else if (st.lastSwitchSec > 0.0 && sinceSwitch < TargetConfig.BASE_HARD_LOCK_SEC)
          req *= 1.45;

        if (isSwitchBack) {
          req *= 2.15;
          if (!candStable) req *= 1.35;
          if (!bestStableEnough) req *= 2.0;
        }

        if (pairDist >= 2.0) req *= 1.25;
        if (pingBlock) req *= 2.25;
        if (freezeOnFlicker) req *= 3.0;

        hardOverride =
            Math.max(
                req, Math.max(immediateDelta, keepMargin) * TargetConfig.HARD_LOCK_OVERRIDE_MULT);
      }

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

    if (shouldSwitch) {
      T out = commitSwitch(now, best, distanceFn, pingEps);
      recordOut(now, out, distanceFn, idEps);
      return out;
    }

    recordOut(now, st.sticky, distanceFn, idEps);
    return st.sticky;
  }

  private void recordOut(double now, T out, ToDoubleBiFunction<T, T> distanceFn, double sameEps) {
    boolean changed = !StickyTargetMath.eqN(st.lastOut, out, distanceFn, sameEps);
    if (changed) st.lastOutChangeSec = now;
    st.lastOut = out;
  }
}
