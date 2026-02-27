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
package org.curtinfrc.frc2026.util.Repulsor.Predictive.Runtime;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.Predicate;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Internal.CollectEval;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Internal.IntentAggCont;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.CollectProbe;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.PointCandidate;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.PredictiveFieldStateOps;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.SpatialDyn;
import org.littletonrobotics.junction.Logger;

public final class PredictiveCollectNearestResolutionStep {
  private PredictiveCollectNearestResolutionStep() {}

  static final double EARLY_SWITCH_RICHER_UNITS_ABS = 0.08;
  static final double EARLY_SWITCH_RICHER_UNITS_REL = 1.55;
  static final double EARLY_SWITCH_ETA_DELTA_MAX_S = 0.85;
  static final double EARLY_SWITCH_SCORE_FLOOR_DELTA = 0.45;
  static final double ESCAPE_SAME_TARGET_EPS_M = 0.05;

  static boolean allowCommitWindowRicherSwitch(CollectEval current, CollectEval candidate) {
    if (current == null || candidate == null) return false;
    if (!Double.isFinite(current.units) || !Double.isFinite(candidate.units)) return false;
    if (!Double.isFinite(current.eta) || !Double.isFinite(candidate.eta)) return false;
    if (!Double.isFinite(current.score) || !Double.isFinite(candidate.score)) return false;

    boolean significantlyRicher =
        candidate.units >= current.units + EARLY_SWITCH_RICHER_UNITS_ABS
            || candidate.units >= current.units * EARLY_SWITCH_RICHER_UNITS_REL;
    boolean etaAcceptable = candidate.eta <= current.eta + EARLY_SWITCH_ETA_DELTA_MAX_S;
    boolean scoreAcceptable = candidate.score >= current.score - EARLY_SWITCH_SCORE_FLOOR_DELTA;
    return significantlyRicher && etaAcceptable && scoreAcceptable;
  }

  static boolean shouldDropEscapedCurrentTarget(
      boolean escape, Translation2d currentTarget, Translation2d chosenPoint) {
    if (!escape || currentTarget == null || chosenPoint == null) return false;
    return currentTarget.getDistance(chosenPoint) <= ESCAPE_SAME_TARGET_EPS_M;
  }

  public static PointCandidate resolve(
      PredictiveFieldStateOps ops,
      Translation2d ourPos,
      double cap,
      int goal,
      double cellM,
      SpatialDyn dyn,
      IntentAggCont enemyIntent,
      IntentAggCont allyIntent,
      Predicate<Translation2d> inShootBand,
      Translation2d bestP,
      Translation2d bestTouch,
      Rotation2d bestHeading,
      CollectEval bestE,
      double totalEv,
      double minUnits,
      int minCount,
      double minEv,
      double onHalf,
      double onR,
      double minHardUnits,
      double footprintMinUnits,
      double rCore,
      double rSnap,
      double rCentroid) {

    Translation2d chosenPt = bestP;
    Translation2d chosenTouch = bestTouch;
    Rotation2d chosenHeading = bestHeading;
    CollectEval chosenE = bestE;

    double now = Timer.getFPGATimestamp();

    if (ops.currentCollectTarget != null && inShootBand.test(ops.currentCollectTarget)) {
      ops.addDepletedMark(
          ops.currentCollectTarget, 0.65, 1.25, PredictiveFieldStateOps.DEPLETED_TTL_S, false);
      ops.addDepletedRing(
          ops.currentCollectTarget, 0.35, 0.95, 0.80, PredictiveFieldStateOps.DEPLETED_TTL_S);
      ops.recordRegionAttempt(dyn, ops.currentCollectTarget, now, false);
      ops.currentCollectTarget = null;
      ops.currentCollectTouch = null;
      ops.collectArrivalTs = -1.0;
    }

    boolean escape =
        ops.currentCollectTarget != null
            && ops.shouldEscapeCurrentCollect(ourPos, dyn, totalEv, minUnits, minCount, cellM);

    if (shouldDropEscapedCurrentTarget(escape, ops.currentCollectTarget, chosenPt)
        && ops.currentCollectTarget != null) {
      ops.addDepletedMark(
          ops.currentCollectTarget, 0.65, 1.10, PredictiveFieldStateOps.DEPLETED_TTL_S, false);
      ops.recordRegionAttempt(dyn, ops.currentCollectTarget, now, false);
      ops.currentCollectTarget = null;
      ops.currentCollectTouch = null;
      ops.currentCollectScore = -1e18;
      ops.currentCollectUnits = 0.0;
      ops.currentCollectEta = 0.0;
      ops.collectArrivalTs = -1.0;
      return null;
    }

    if (ops.currentCollectTarget != null
        && !ops.footprintOk(
            dyn, ops.currentCollectTarget, ops.currentCollectHeading, rCore, footprintMinUnits)) {
      ops.addDepletedMark(
          ops.currentCollectTarget, 0.70, 1.85, PredictiveFieldStateOps.DEPLETED_TTL_S, false);
      ops.addDepletedRing(
          ops.currentCollectTarget, 0.35, 0.95, 0.80, PredictiveFieldStateOps.DEPLETED_TTL_S);
      ops.recordRegionAttempt(dyn, ops.currentCollectTarget, now, false);
      ops.currentCollectTarget = null;
      ops.currentCollectTouch = null;
      ops.collectArrivalTs = -1.0;
      escape = true;
    }

    double commitS = ops.collectCommitWindow(ops.currentCollectEta);
    double switchMargin =
        (PredictiveFieldStateOps.COLLECT_SWITCH_BASE
            + PredictiveFieldStateOps.COLLECT_SWITCH_ETA_W * Math.max(0.0, ops.currentCollectEta));

    if (ops.currentCollectTarget != null && !escape) {
      double age = now - ops.currentCollectChosenTs;

      Translation2d curTouch =
          ops.resolveCollectTouch(
              dyn, ops.currentCollectTarget, ops.currentCollectHeading, rCore, rSnap, rCentroid);

      if (curTouch == null
          || inShootBand.test(curTouch)
          || inShootBand.test(ops.currentCollectTarget)) {
        ops.recordRegionAttempt(dyn, ops.currentCollectTarget, now, false);
        ops.currentCollectTarget = null;
        ops.currentCollectTouch = null;
        ops.collectArrivalTs = -1.0;
        return null;
      }

      CollectEval curE =
          ops.evalCollectPoint(
              ourPos, cap, ops.currentCollectTarget, goal, cellM, dyn, enemyIntent, allyIntent);
      curE.banditBonus = ops.regionBanditBonus(dyn, curTouch, now);
      curE.score += curE.banditBonus;
      curE.score += PredictiveFieldStateOps.COLLECT_KEEP_BONUS;

      double curOnUnits = dyn.valueInSquare(curTouch, onHalf);
      int curOnCount = dyn.countResourcesWithin(curTouch, onR);
      double curEv = dyn.evidenceMassWithin(curTouch, PredictiveFieldStateOps.EVIDENCE_R);
      int curCore = dyn.countResourcesWithin(curTouch, rCore);
      Translation2d curNN = dyn.nearestResourceTo(curTouch, rCore);

      curE.units = curOnUnits;
      curE.count = curOnCount;
      curE.evidence = curEv;
      curE.coreCount = curCore;
      curE.coreDist = curNN != null ? curNN.getDistance(curTouch) : 1e9;

      ops.currentCollectScore = curE.score;
      ops.currentCollectUnits = curE.units;
      ops.currentCollectEta = curE.eta;

      if (age >= 0.0 && age <= commitS) {
        boolean commitOverride = allowCommitWindowRicherSwitch(curE, chosenE);
        if (!commitOverride && chosenE.score <= ops.currentCollectScore + switchMargin) {
          chosenPt = ops.currentCollectTarget;
          chosenE = curE;
          chosenTouch = curTouch;
          chosenHeading = ops.currentCollectHeading;
        } else {
          if (inShootBand.test(chosenPt) || inShootBand.test(chosenTouch)) {
            chosenPt = ops.currentCollectTarget;
            chosenE = curE;
            chosenTouch = curTouch;
            chosenHeading = ops.currentCollectHeading;
          } else {
            ops.recordRegionAttempt(dyn, ops.currentCollectTarget, now, false);
            ops.currentCollectTarget = chosenPt;
            ops.currentCollectTouch = chosenTouch;
            ops.currentCollectHeading = chosenHeading;
            ops.currentCollectChosenTs = now;
            ops.currentCollectScore = chosenE.score;
            ops.currentCollectUnits = chosenE.units;
            ops.currentCollectEta = chosenE.eta;
            ops.recordRegionAttempt(dyn, ops.currentCollectTarget, now, false);
          }
        }
      } else {
        if (chosenE.score > ops.currentCollectScore + switchMargin
            && !inShootBand.test(chosenPt)
            && !inShootBand.test(chosenTouch)) {
          ops.recordRegionAttempt(dyn, ops.currentCollectTarget, now, false);
          ops.currentCollectTarget = chosenPt;
          ops.currentCollectTouch = chosenTouch;
          ops.currentCollectHeading = chosenHeading;
          ops.currentCollectChosenTs = now;
          ops.currentCollectScore = chosenE.score;
          ops.currentCollectUnits = chosenE.units;
          ops.currentCollectEta = chosenE.eta;
          ops.recordRegionAttempt(dyn, ops.currentCollectTarget, now, false);
        } else {
          chosenPt = ops.currentCollectTarget;
          chosenE = curE;
          chosenTouch = curTouch;
          chosenHeading = ops.currentCollectHeading;
        }
      }
    } else {
      if (inShootBand.test(chosenPt) || inShootBand.test(chosenTouch)) {
        return null;
      }

      if (ops.currentCollectTarget == null
          || chosenPt.getDistance(ops.currentCollectTarget) > 0.05
          || escape) {
        if (escape && ops.currentCollectTarget != null) {
          ops.recordRegionAttempt(dyn, ops.currentCollectTarget, now, false);
        }
        ops.currentCollectTarget = chosenPt;
        ops.currentCollectTouch = chosenTouch;
        ops.currentCollectHeading = chosenHeading;
        ops.currentCollectChosenTs = now;
        ops.currentCollectScore = chosenE.score;
        ops.currentCollectUnits = chosenE.units;
        ops.currentCollectEta = chosenE.eta;
        ops.recordRegionAttempt(dyn, ops.currentCollectTarget, now, false);
      } else {
        ops.currentCollectTarget = chosenPt;
        ops.currentCollectTouch = chosenTouch;
        ops.currentCollectHeading = chosenHeading;
        ops.currentCollectScore = chosenE.score;
        ops.currentCollectUnits = chosenE.units;
        ops.currentCollectEta = chosenE.eta;
      }
    }

    if (ops.currentCollectTarget != null) {
      if (inShootBand.test(ops.currentCollectTarget)) {
        ops.addDepletedMark(
            ops.currentCollectTarget, 0.65, 1.25, PredictiveFieldStateOps.DEPLETED_TTL_S, false);
        ops.addDepletedRing(
            ops.currentCollectTarget, 0.35, 0.95, 0.80, PredictiveFieldStateOps.DEPLETED_TTL_S);
        ops.recordRegionAttempt(dyn, ops.currentCollectTarget, now, false);
        ops.currentCollectTarget = null;
        ops.currentCollectTouch = null;
        ops.collectArrivalTs = -1.0;
        return null;
      }

      if (ops.currentCollectTouch == null) {
        ops.currentCollectTouch =
            ops.resolveCollectTouch(
                dyn, ops.currentCollectTarget, ops.currentCollectHeading, rCore, rSnap, rCentroid);
      }

      if (!ops.footprintOk(
          dyn, ops.currentCollectTarget, ops.currentCollectHeading, rCore, footprintMinUnits)) {
        ops.addDepletedMark(
            ops.currentCollectTarget, 0.70, 1.85, PredictiveFieldStateOps.DEPLETED_TTL_S, false);
        ops.addDepletedRing(
            ops.currentCollectTarget, 0.35, 0.95, 0.80, PredictiveFieldStateOps.DEPLETED_TTL_S);
        ops.recordRegionAttempt(dyn, ops.currentCollectTarget, now, false);
        ops.currentCollectTarget = null;
        ops.currentCollectTouch = null;
        ops.collectArrivalTs = -1.0;
        return null;
      }

      Translation2d anchoredTouch =
          ops.currentCollectTouch != null
              ? ops.enforceHardStopOnFuel(
                  dyn, ops.currentCollectTouch, rCore, rSnap, rCentroid, 0.10)
              : null;

      if (anchoredTouch == null || inShootBand.test(anchoredTouch)) {
        ops.addDepletedMark(
            ops.currentCollectTarget, 0.65, 1.25, PredictiveFieldStateOps.DEPLETED_TTL_S, false);
        ops.addDepletedRing(
            ops.currentCollectTarget, 0.35, 0.95, 0.80, PredictiveFieldStateOps.DEPLETED_TTL_S);
        ops.recordRegionAttempt(dyn, ops.currentCollectTarget, now, false);
        ops.currentCollectTarget = null;
        ops.currentCollectTouch = null;
        ops.collectArrivalTs = -1.0;
        return null;
      }

      ops.currentCollectTouch = anchoredTouch;
      ops.currentCollectHeading =
          ops.face(ops.currentCollectTarget, ops.currentCollectTouch, ops.currentCollectHeading);
      ops.currentCollectTarget =
          PredictiveFieldStateOps.COLLECT_INTAKE
              .get()
              .snapCenterSoFootprintTouchesPoint(
                  ops.currentCollectTarget, ops.currentCollectHeading, ops.currentCollectTouch);

      if (inShootBand.test(ops.currentCollectTarget)) {
        ops.addDepletedMark(
            ops.currentCollectTarget, 0.65, 1.25, PredictiveFieldStateOps.DEPLETED_TTL_S, false);
        ops.addDepletedRing(
            ops.currentCollectTarget, 0.35, 0.95, 0.80, PredictiveFieldStateOps.DEPLETED_TTL_S);
        ops.recordRegionAttempt(dyn, ops.currentCollectTarget, now, false);
        ops.currentCollectTarget = null;
        ops.currentCollectTouch = null;
        ops.collectArrivalTs = -1.0;
        return null;
      }
    }

    boolean arrived =
        ops.currentCollectTarget != null
            && ourPos.getDistance(ops.currentCollectTarget)
                <= PredictiveFieldStateOps.COLLECT_ARRIVE_R;

    if (arrived) {
      if (ops.collectArrivalTs < 0.0) ops.collectArrivalTs = now;
      double held = now - ops.collectArrivalTs;
      if (held >= PredictiveFieldStateOps.COLLECT_ARRIVE_VERIFY_S) {
        Translation2d touch = ops.currentCollectTouch;
        if (touch == null) {
          touch =
              ops.resolveCollectTouch(
                  dyn,
                  ops.currentCollectTarget,
                  ops.currentCollectHeading,
                  rCore,
                  rSnap,
                  rCentroid);
          ops.currentCollectTouch = touch;
        }

        Translation2d nn = touch != null ? dyn.nearestResourceTo(touch, rCore) : null;
        int cc = touch != null ? dyn.countResourcesWithin(touch, rCore) : 0;

        if (touch == null
            || nn == null
            || cc < 1
            || inShootBand.test(touch)
            || inShootBand.test(ops.currentCollectTarget)) {
          ops.addDepletedMark(
              ops.currentCollectTarget, 0.70, 1.85, PredictiveFieldStateOps.DEPLETED_TTL_S, false);
          ops.addDepletedMark(
              ops.currentCollectTarget,
              0.95,
              1.10,
              PredictiveFieldStateOps.COLLECT_FAIL_COOLDOWN_S,
              false);
          ops.addDepletedRing(
              ops.currentCollectTarget,
              0.35,
              1.05,
              0.95,
              PredictiveFieldStateOps.COLLECT_FAIL_COOLDOWN_S);
          ops.recordRegionAttempt(dyn, ops.currentCollectTarget, now, false);
          ops.currentCollectTarget = null;
          ops.currentCollectTouch = null;
          ops.currentCollectScore = -1e18;
          ops.currentCollectUnits = 0.0;
          ops.currentCollectEta = 0.0;
          ops.collectArrivalTs = -1.0;
          ops.lastReturnedCollect = null;
          ops.lastReturnedCollectTs = 0.0;
          return null;
        }

        CollectProbe pr = ops.probeCollect(touch, 0.65);
        double needU = Math.max(0.02, minUnits * 0.80);
        int needC = Math.max(1, minCount);
        if (pr.units < needU || pr.count < needC) {
          ops.addDepletedMark(
              ops.currentCollectTarget, 0.70, 1.85, PredictiveFieldStateOps.DEPLETED_TTL_S, false);
          ops.addDepletedMark(
              ops.currentCollectTarget,
              0.95,
              1.10,
              PredictiveFieldStateOps.COLLECT_FAIL_COOLDOWN_S,
              false);
          ops.addDepletedRing(
              ops.currentCollectTarget,
              0.35,
              1.05,
              0.95,
              PredictiveFieldStateOps.COLLECT_FAIL_COOLDOWN_S);
          ops.recordRegionAttempt(dyn, ops.currentCollectTarget, now, false);
          ops.currentCollectTarget = null;
          ops.currentCollectTouch = null;
          ops.currentCollectScore = -1e18;
          ops.currentCollectUnits = 0.0;
          ops.currentCollectEta = 0.0;
          ops.collectArrivalTs = -1.0;
          ops.lastReturnedCollect = null;
          ops.lastReturnedCollectTs = 0.0;
          return null;
        }
      }
    } else {
      ops.collectArrivalTs = -1.0;
    }

    if (ops.currentCollectTarget == null || inShootBand.test(ops.currentCollectTarget)) return null;

    if (ops.currentCollectTouch == null) {
      ops.currentCollectTouch =
          ops.resolveCollectTouch(
              dyn, ops.currentCollectTarget, ops.currentCollectHeading, rCore, rSnap, rCentroid);
    }
    if (ops.currentCollectTouch == null || inShootBand.test(ops.currentCollectTouch)) {
      ops.addDepletedMark(
          ops.currentCollectTarget, 0.65, 1.25, PredictiveFieldStateOps.DEPLETED_TTL_S, false);
      ops.addDepletedRing(
          ops.currentCollectTarget, 0.35, 0.95, 0.80, PredictiveFieldStateOps.DEPLETED_TTL_S);
      ops.recordRegionAttempt(dyn, ops.currentCollectTarget, now, false);
      ops.currentCollectTarget = null;
      ops.collectArrivalTs = -1.0;
      return null;
    }

    double finalOnUnits = dyn.valueInSquare(ops.currentCollectTouch, onHalf);
    int finalOnCount = dyn.countResourcesWithin(ops.currentCollectTouch, onR);
    double finalEv =
        dyn.evidenceMassWithin(ops.currentCollectTouch, PredictiveFieldStateOps.EVIDENCE_R);

    if (finalOnUnits < minHardUnits || finalOnCount < 1 || finalEv < minEv) {
      Translation2d anchoredTouch =
          ops.enforceHardStopOnFuel(dyn, ops.currentCollectTouch, rCore, rSnap, rCentroid, 0.10);
      if (anchoredTouch != null && !inShootBand.test(anchoredTouch)) {
        ops.currentCollectTouch = anchoredTouch;
        ops.currentCollectHeading =
            ops.face(ops.currentCollectTarget, ops.currentCollectTouch, ops.currentCollectHeading);
        ops.currentCollectTarget =
            PredictiveFieldStateOps.COLLECT_INTAKE
                .get()
                .snapCenterSoFootprintTouchesPoint(
                    ops.currentCollectTarget, ops.currentCollectHeading, ops.currentCollectTouch);
        if (inShootBand.test(ops.currentCollectTarget)) {
          ops.addDepletedMark(
              ops.currentCollectTarget, 0.65, 1.25, PredictiveFieldStateOps.DEPLETED_TTL_S, false);
          ops.addDepletedRing(
              ops.currentCollectTarget, 0.35, 0.95, 0.80, PredictiveFieldStateOps.DEPLETED_TTL_S);
          ops.recordRegionAttempt(dyn, ops.currentCollectTarget, now, false);
          ops.currentCollectTarget = null;
          ops.currentCollectTouch = null;
          ops.collectArrivalTs = -1.0;
          return null;
        }
        finalOnUnits = dyn.valueInSquare(ops.currentCollectTouch, onHalf);
        finalOnCount = dyn.countResourcesWithin(ops.currentCollectTouch, onR);
        finalEv =
            dyn.evidenceMassWithin(ops.currentCollectTouch, PredictiveFieldStateOps.EVIDENCE_R);
      } else {
        ops.addDepletedMark(
            ops.currentCollectTarget, 0.65, 1.25, PredictiveFieldStateOps.DEPLETED_TTL_S, false);
        ops.addDepletedRing(
            ops.currentCollectTarget, 0.35, 0.95, 0.80, PredictiveFieldStateOps.DEPLETED_TTL_S);
        ops.recordRegionAttempt(dyn, ops.currentCollectTarget, now, false);
        ops.currentCollectTarget = null;
        ops.currentCollectTouch = null;
        ops.collectArrivalTs = -1.0;
        return null;
      }
    }

    Translation2d nnCore = dyn.nearestResourceTo(ops.currentCollectTouch, rCore);
    int cCoreFinal = dyn.countResourcesWithin(ops.currentCollectTouch, rCore);
    if (nnCore == null || cCoreFinal < 1) {
      ops.addDepletedMark(
          ops.currentCollectTarget, 0.65, 1.25, PredictiveFieldStateOps.DEPLETED_TTL_S, false);
      ops.addDepletedRing(
          ops.currentCollectTarget, 0.35, 0.95, 0.80, PredictiveFieldStateOps.DEPLETED_TTL_S);
      ops.recordRegionAttempt(dyn, ops.currentCollectTarget, now, false);
      ops.currentCollectTarget = null;
      ops.currentCollectTouch = null;
      ops.collectArrivalTs = -1.0;
      return null;
    }

    CollectEval finalE =
        ops.evalCollectPoint(
            ourPos, cap, ops.currentCollectTarget, goal, cellM, dyn, enemyIntent, allyIntent);
    finalE.banditBonus = ops.regionBanditBonus(dyn, ops.currentCollectTouch, now);
    finalE.score += finalE.banditBonus;

    finalE.units = finalOnUnits;
    finalE.count = finalOnCount;
    finalE.evidence = finalEv;
    finalE.coreCount = cCoreFinal;
    finalE.coreDist = nnCore.getDistance(ops.currentCollectTouch);

    if (finalE.units < minUnits * 0.65
        || finalE.count < Math.max(1, minCount - 1)
        || finalE.evidence < minEv
        || finalE.coreCount < 1
        || inShootBand.test(ops.currentCollectTarget)
        || inShootBand.test(ops.currentCollectTouch)) {
      ops.addDepletedMark(
          ops.currentCollectTarget, 0.65, 1.25, PredictiveFieldStateOps.DEPLETED_TTL_S, false);
      ops.addDepletedRing(
          ops.currentCollectTarget, 0.35, 0.95, 0.80, PredictiveFieldStateOps.DEPLETED_TTL_S);
      ops.recordRegionAttempt(dyn, ops.currentCollectTarget, now, false);
      ops.currentCollectTarget = null;
      ops.currentCollectTouch = null;
      ops.collectArrivalTs = -1.0;
      return null;
    }

    ops.lastReturnedCollect = ops.currentCollectTarget;
    ops.lastReturnedCollectTs = now;

    Logger.recordOutput(
        "Repulsor/ChosenCollect", new Pose2d(ops.currentCollectTarget, ops.currentCollectHeading));

    return new PointCandidate(
        ops.currentCollectTarget,
        ops.currentCollectHeading,
        finalE.eta,
        finalE.value,
        finalE.enemyPressure,
        finalE.allyCongestion,
        finalE.enemyIntent,
        finalE.allyIntent,
        finalE.score);
  }
}
