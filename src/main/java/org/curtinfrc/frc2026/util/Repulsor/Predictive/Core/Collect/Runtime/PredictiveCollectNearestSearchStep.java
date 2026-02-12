package org.curtinfrc.frc2026.util.Repulsor.Predictive.Core.Collect.Runtime;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.function.Predicate;
import java.util.function.ToDoubleFunction;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Core.Internal.CollectEval;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Core.Internal.FootprintEval;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Core.Internal.HeadingPick;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Core.Internal.IntentAggCont;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Core.PredictiveFieldStateOps;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Core.SpatialDyn;

public final class PredictiveCollectNearestSearchStep {
  private PredictiveCollectNearestSearchStep() {}

  public static PredictiveCollectNearestSearchResult search(
      PredictiveFieldStateOps ops,
      Translation2d[] seeds,
      ArrayList<Integer> shortlist,
      int maxCheck,
      Translation2d ourPos,
      double cap,
      int goal,
      double cellM,
      SpatialDyn dyn,
      IntentAggCont enemyIntent,
      IntentAggCont allyIntent,
      Predicate<Translation2d> inShootBand,
      ToDoubleFunction<Translation2d> wallPenalty,
      double nearHalf,
      double nearR,
      double onHalf,
      double onR,
      double minHardUnits,
      double minUnits,
      int minCount,
      double minEv,
      double rCore,
      double rSnap,
      double rCentroid,
      double footprintMinUnits) {

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
      if (regionUnitsNear
              < Math.min(PredictiveFieldStateOps.COLLECT_COARSE_MIN_REGION_UNITS, minUnits * 0.90)
          || cNear0 < 1) continue;

      double step = Math.max(0.05, cellM * PredictiveFieldStateOps.COLLECT_FINE_OFFSETS_SCALE);
      int g = Math.max(2, PredictiveFieldStateOps.COLLECT_FINE_OFFSETS_GRID);
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

          Translation2d fuelTouch =
              ops.enforceHardStopOnFuel(dyn, p0, rCore, rSnap, rCentroid, 0.10);
          if (fuelTouch == null || inShootBand.test(fuelTouch)) continue;

          double fuelOn = dyn.valueInSquare(fuelTouch, onHalf);
          int cOn = dyn.countResourcesWithin(fuelTouch, onR);
          double evHard = dyn.evidenceMassWithin(fuelTouch, PredictiveFieldStateOps.EVIDENCE_R);
          if (fuelOn < minHardUnits || cOn < 1 || evHard < minEv) continue;

          int cCore = dyn.countResourcesWithin(fuelTouch, rCore);
          if (cCore < 1) continue;

          Rotation2d baseHeading = PredictiveFieldStateOps.face(p0, fuelTouch, new Rotation2d());
          HeadingPick pick =
              ops.bestHeadingForFootprint(
                  dyn, p0, fuelTouch, baseHeading, rCore, footprintMinUnits);
          if (pick == null || inShootBand.test(pick.center)) continue;
          Rotation2d heading = pick.heading;
          Translation2d p = pick.center;

          CollectEval e =
              ops.evalCollectPoint(ourPos, cap, p, goal, cellM, dyn, enemyIntent, allyIntent);

          e.units = fuelOn;
          e.count = cOn;
          e.evidence = evHard;
          e.coreCount = cCore;
          Translation2d nn = dyn.nearestResourceTo(fuelTouch, rCore);
          e.coreDist = nn != null ? nn.getDistance(fuelTouch) : 1e9;

          e.regionUnits = regionUnitsNear;
          e.banditBonus = ops.regionBanditBonus(dyn, fuelTouch, now0);
          e.score += e.banditBonus;
          e.score -= wallPenalty.applyAsDouble(fuelTouch);
          e.score -= wallPenalty.applyAsDouble(p) * 0.85;
          e.score -=
              ops.depletedPenaltySoft(fuelTouch) * PredictiveFieldStateOps.DEPLETED_PEN_W * 1.35;
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
        Translation2d anchor =
            ops.enforceHardStopOnFuel(dyn, center, rCore, rSnap, rCentroid, 0.10);
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
                  ops.enforceHardStopOnFuel(dyn, p0, rCore, rSnap, rCentroid, 0.08);
              if (fuelTouch == null || inShootBand.test(fuelTouch)) continue;

              int cCore = dyn.countResourcesWithin(fuelTouch, rCore);
              if (cCore < 1) continue;

              double fuelOn = dyn.valueInSquare(fuelTouch, onHalf);
              int cOn = dyn.countResourcesWithin(fuelTouch, onR);
              double evHard = dyn.evidenceMassWithin(fuelTouch, PredictiveFieldStateOps.EVIDENCE_R);
              if (fuelOn < minHardUnits || cOn < 1 || evHard < minEv) continue;

              Rotation2d baseHeading = ops.face(p0, fuelTouch, new Rotation2d());
              HeadingPick pick =
                  ops.bestHeadingForFootprint(
                      dyn, p0, fuelTouch, baseHeading, rCore, footprintMinUnits);
              if (pick == null || inShootBand.test(pick.center)) continue;
              Rotation2d heading = pick.heading;
              Translation2d p = pick.center;

              CollectEval e =
                  ops.evalCollectPoint(ourPos, cap, p, goal, cellM, dyn, enemyIntent, allyIntent);

              e.units = fuelOn;
              e.count = cOn;
              e.evidence = evHard;
              e.coreCount = cCore;
              Translation2d nn = dyn.nearestResourceTo(fuelTouch, rCore);
              e.coreDist = nn != null ? nn.getDistance(fuelTouch) : 1e9;

              e.regionUnits = regionUnitsNear;
              e.banditBonus = ops.regionBanditBonus(dyn, fuelTouch, now0);
              e.score += e.banditBonus;
              e.score -= wallPenalty.applyAsDouble(fuelTouch);
              e.score -= wallPenalty.applyAsDouble(p) * 0.85;
              e.score -=
                  ops.depletedPenaltySoft(fuelTouch)
                      * PredictiveFieldStateOps.DEPLETED_PEN_W
                      * 1.35;
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

    return new PredictiveCollectNearestSearchResult(
        bestP,
        bestTouch,
        bestHeading,
        bestE,
        topScore,
        topEta,
        topUnits,
        topRegion,
        topDep,
        topAvoid,
        topEP,
        topAC,
        topAct,
        topEv,
        topN);
  }
}
