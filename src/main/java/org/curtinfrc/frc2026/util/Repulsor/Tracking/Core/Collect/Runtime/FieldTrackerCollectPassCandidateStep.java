package org.curtinfrc.frc2026.util.Repulsor.Tracking.Core.Collect.Runtime;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.HashMap;
import java.util.function.Function;
import java.util.function.Predicate;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Core.Model.CollectProbe;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Core.Model.PointCandidate;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Core.Collect.FieldTrackerCollectObjectiveLoop;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Core.Collect.FieldTrackerCollectObjectiveMath;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Core.Internal.NearestPoint;

public final class FieldTrackerCollectPassCandidateStep {
  private FieldTrackerCollectPassCandidateStep() {}

  public static FieldTrackerCollectPassCandidateResult choose(
      FieldTrackerCollectObjectiveLoop loop, FieldTrackerCollectPassContext ctx, int goalUnits) {
    HashMap<Long, CollectProbe> probeCache = new HashMap<>(512);
    HashMap<Long, Boolean> footprintCache = new HashMap<>(512);
    HashMap<Long, Boolean> nearFuelCache = new HashMap<>(512);
    HashMap<Long, Boolean> validCache = new HashMap<>(512);
    HashMap<Long, Boolean> validRelaxedCache = new HashMap<>(512);
    HashMap<Long, Double> scoreCache = new HashMap<>(512);

    java.util.function.ToLongFunction<Translation2d> pointKey =
        p -> {
          long kx = (long) Math.round(p.getX() * 1000.0);
          long ky = (long) Math.round(p.getY() * 1000.0);
          return (kx << 32) ^ (ky & 0xffffffffL);
        };

    Function<Translation2d, CollectProbe> probe =
        p -> {
          if (p == null) return null;
          long key = pointKey.applyAsLong(p);
          if (probeCache.containsKey(key)) return probeCache.get(key);
          CollectProbe pr = loop.predictor.probeCollect(p, 0.75);
          probeCache.put(key, pr);
          return pr;
        };

    Predicate<Translation2d> footprintHasFuel =
        p -> {
          if (p == null) return false;
          long key = pointKey.applyAsLong(p);
          Boolean cached = footprintCache.get(key);
          if (cached != null) return cached;
          boolean ok1 =
              loop.predictor.footprintHasCollectResource(
                  p, FieldTrackerCollectObjectiveLoop.COLLECT_CELL_M);
          if (footprintCache.size() > 2048) footprintCache.clear();
          footprintCache.put(key, ok1);
          return ok1;
        };

    Predicate<Translation2d> hasNearFuel =
        p -> {
          if (p == null) return false;
          long key = pointKey.applyAsLong(p);
          Boolean cached = nearFuelCache.get(key);
          if (cached != null) return cached;
          boolean ok1 =
              loop.predictor.nearestCollectResource(
                      p, FieldTrackerCollectObjectiveLoop.COLLECT_VALID_NEAR_FUEL_M)
                  != null;
          if (nearFuelCache.size() > 2048) nearFuelCache.clear();
          nearFuelCache.put(key, ok1);
          return ok1;
        };

    Predicate<Translation2d> collectValid =
        p -> {
          if (p == null) return false;
          long key = pointKey.applyAsLong(p);
          Boolean cached = validCache.get(key);
          if (cached != null) return cached;
          CollectProbe pr = probe.apply(p);
          if (pr == null) {
            validCache.put(key, false);
            return false;
          }
          if (pr.count < 1
              || pr.units
                  < Math.max(
                      0.02,
                      FieldTrackerCollectObjectiveLoop.COLLECT_RESOURCE_SNAP_MIN_UNITS * 0.75)) {
            validCache.put(key, false);
            return false;
          }
          if (!hasNearFuel.test(p)) {
            validCache.put(key, false);
            return false;
          }
          boolean ok1 = footprintHasFuel.test(p);
          validCache.put(key, ok1);
          return ok1;
        };

    Predicate<Translation2d> collectValidRelaxed =
        p -> {
          if (p == null) return false;
          long key = pointKey.applyAsLong(p);
          Boolean cached = validRelaxedCache.get(key);
          if (cached != null) return cached;
          CollectProbe pr = probe.apply(p);
          if (pr == null) {
            validRelaxedCache.put(key, false);
            return false;
          }
          if (pr.count < 1
              || pr.units
                  < Math.max(
                      0.02,
                      FieldTrackerCollectObjectiveLoop.COLLECT_RESOURCE_SNAP_MIN_UNITS * 0.75)) {
            validRelaxedCache.put(key, false);
            return false;
          }
          boolean ok1 = footprintHasFuel.test(p);
          validRelaxedCache.put(key, ok1);
          return ok1;
        };

    Function<Translation2d, Double> collectUnits =
        p -> {
          CollectProbe pr = probe.apply(p);
          if (pr == null) return 0.0;
          return pr.units;
        };

    PointCandidate best = null;

    for (int attempt = 0; attempt < 5; attempt++) {
      best =
          loop.predictor.rankCollectNearest(
              ctx.robotPos(),
              ctx.cap(),
              ctx.usePts(),
              FieldTrackerCollectObjectiveLoop.COLLECT_CELL_M,
              goalUnits,
              Math.min(160, Math.max(32, ctx.usePts().length)));

      if (best != null) {
        loop.lastBest = best;
        break;
      }

      if (best == null && loop.lastBest != null) {
        best = loop.lastBest;
        break;
      }

      best =
          loop.predictor.rankCollectHierarchical(
              ctx.robotPos(),
              ctx.cap(),
              ctx.usePts(),
              FieldTrackerCollectObjectiveLoop.COLLECT_CELL_M,
              goalUnits,
              FieldTrackerCollectObjectiveLoop.COLLECT_COARSE_TOPK,
              FieldTrackerCollectObjectiveLoop.COLLECT_REFINE_GRID);

      if (best != null && collectValid.test(best.point)) {
        break;
      }

      best =
          loop.predictor.rankCollectPoints(
              ctx.robotPos(),
              ctx.cap(),
              ctx.usePts(),
              goalUnits,
              Math.max(24, ctx.usePts().length));

      if (best != null && collectValid.test(best.point)) {
        break;
      }

      Translation2d hot =
          loop.predictor.bestCollectHotspot(
              ctx.usePts(), FieldTrackerCollectObjectiveLoop.COLLECT_CELL_M);
      if (hot != null) {
        NearestPoint snap = FieldTrackerCollectObjectiveMath.nearestPointTo(hot, ctx.usePts());
        if (snap.p != null
            && snap.d <= FieldTrackerCollectObjectiveLoop.COLLECT_HOTSPOT_SNAP_RADIUS_M
            && collectValid.test(snap.p)) {
          best =
              new PointCandidate(
                  snap.p,
                  new Rotation2d(),
                  ctx.robotPos().getDistance(snap.p) / Math.max(0.1, ctx.cap()),
                  0.0,
                  0.0,
                  0.0,
                  0.0,
                  0.0,
                  -1e9);
          break;
        }
      }
    }

    Translation2d rawCandidate = (best != null ? best.point : null);

    if (rawCandidate == null || !collectValid.test(rawCandidate)) {
      Translation2d fallback = null;
      double bestU = 0.0;
      boolean anyStrict = false;
      for (Translation2d p : ctx.usePts()) {
        if (p == null) continue;
        CollectProbe pr = probe.apply(p);
        if (pr == null
            || pr.count < 1
            || pr.units
                < Math.max(
                    0.02,
                    FieldTrackerCollectObjectiveLoop.COLLECT_RESOURCE_SNAP_MIN_UNITS * 0.75)) {
          continue;
        }
        if (!hasNearFuel.test(p)) continue;
        if (!footprintHasFuel.test(p)) continue;
        anyStrict = true;
        double u = pr.units;
        if (u > bestU) {
          bestU = u;
          fallback = p;
        }
      }
      if (fallback == null && !anyStrict) {
        for (Translation2d p : ctx.usePts()) {
          if (p == null) continue;
          CollectProbe pr = probe.apply(p);
          if (pr == null
              || pr.count < 1
              || pr.units
                  < Math.max(
                      0.02,
                      FieldTrackerCollectObjectiveLoop.COLLECT_RESOURCE_SNAP_MIN_UNITS * 0.75))
            continue;
          if (!footprintHasFuel.test(p)) continue;
          double u = pr.units;
          if (u > bestU) {
            bestU = u;
            fallback = p;
          }
        }
      }
      if (fallback == null || !collectValid.test(fallback)) {
        Translation2d near =
            loop.predictor.nearestCollectResource(
                ctx.robotPos(), FieldTrackerCollectObjectiveLoop.COLLECT_SNAP_TO_NEAREST_FUEL_M);
        if (near != null && collectValidRelaxed.test(near)) {
          fallback = near;
        }
      }
      if (fallback == null || !collectValidRelaxed.test(fallback)) {
        Translation2d lastBestPoint = (loop.lastBest != null) ? loop.lastBest.point : null;
        if (lastBestPoint != null && collectValidRelaxed.test(lastBestPoint)) {
          fallback = lastBestPoint;
        }
      }
      if (fallback == null || !collectValidRelaxed.test(fallback)) {
        if (rawCandidate != null && collectValidRelaxed.test(rawCandidate)) {
          fallback = rawCandidate;
        }
      }
      if (fallback == null || !collectValidRelaxed.test(fallback)) {
        loop.clearCollectSticky();
        return new FieldTrackerCollectPassCandidateResult(
            best,
            null,
            collectValid,
            footprintHasFuel,
            p -> -1e18,
            new Pose2d(ctx.robotPos(), ctx.robotPoseBlue().getRotation()));
      }
      rawCandidate = fallback;
    }

    Function<Translation2d, Double> scoreResource =
        p -> {
          if (p == null) return -1e18;
          long key = pointKey.applyAsLong(p);
          Double cached = scoreCache.get(key);
          if (cached != null) return cached;

          if (!collectValid.test(p)) {
            scoreCache.put(key, -1e18);
            return -1e18;
          }

          double u = collectUnits.apply(p);

          Translation2d d = ctx.safePushedFromRobot().apply(p);
          if (d == null) d = p;

          d = ctx.clampToFieldRobotSafe().apply(d);
          if (ctx.inForbidden().test(d)) d = ctx.nudgeOutOfForbidden().apply(d);
          d = ctx.clampToFieldRobotSafe().apply(d);
          if (ctx.inForbidden().test(d) || ctx.violatesWall().test(d)) return -1e18;

          double eta = ctx.robotPos().getDistance(d) / Math.max(0.2, ctx.cap());

          double scoreV = (u * 1.0) - (0.55 * eta);
          scoreCache.put(key, scoreV);
          return scoreV;
        };

    Translation2d bestCandidate = rawCandidate;

    boolean allowCentroidCandidate =
        (ctx.robotPos().getDistance(rawCandidate)
                <= FieldTrackerCollectObjectiveLoop.COLLECT_NEARBY_RADIUS_M)
            && (loop.collectStickyStillSec < 0.20);

    if (allowCentroidCandidate && ctx.nearbyCentroid() != null) {
      Translation2d c = ctx.clampToFieldRobotSafe().apply(ctx.nearbyCentroid());
      if (ctx.inForbidden().test(c)) c = ctx.nudgeOutOfForbidden().apply(c);
      c = ctx.clampToFieldRobotSafe().apply(c);

      if (!ctx.inForbidden().test(c) && !ctx.violatesWall().test(c) && collectValid.test(c)) {
        double sc = scoreResource.apply(c);
        double sb = scoreResource.apply(bestCandidate);
        if (sc >= sb - 0.08) bestCandidate = c;
      }
    }

    Translation2d relockCand =
        loop.relockCollectPointToLiveFuel(
            bestCandidate,
            ctx.clampToFieldRobotSafe(),
            ctx.inForbidden(),
            ctx.violatesWall(),
            ctx.nudgeOutOfForbidden());

    if (relockCand != null && collectValid.test(relockCand)) {
      double sr = scoreResource.apply(relockCand);
      double sb = scoreResource.apply(bestCandidate);
      if (sr >= sb - 0.04) bestCandidate = relockCand;
    }

    bestCandidate =
        FieldTrackerCollectObjectiveMath.canonicalizeCollectPoint(bestCandidate, ctx.usePts());

    return new FieldTrackerCollectPassCandidateResult(
        best, bestCandidate, collectValid, footprintHasFuel, scoreResource, null);
  }
}
