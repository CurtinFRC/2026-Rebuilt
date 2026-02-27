package org.curtinfrc.frc2026.util.Repulsor.Predictive;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.HashMap;
import java.util.List;
import java.util.Set;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Internal.CollectEval;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Internal.IntentAggCont;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Internal.ResourceRegions;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.DynamicObject;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.PointCandidate;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.ResourceSpec;
import org.junit.jupiter.api.Test;

class PredictiveCollectSecondaryRankersTest {
  private static final double EPS = 1e-6;

  @Test
  void anchorHierarchicalPointToFuelSnapsNearFuel() {
    SpatialDyn dyn =
        makeDyn(
            List.of(
                new DynamicObject(
                    "f1", "fuel", new Translation2d(4.0, 2.0), new Translation2d(), 0.05)),
            1.0);

    Translation2d anchored =
        PredictiveCollectSecondaryRankers.anchorHierarchicalPointToFuel(
            dyn, new Translation2d(4.4, 2.0), 0.14, 0.06, 0.03);

    assertNotNull(anchored);
    assertEquals(4.0, anchored.getX(), EPS);
    assertEquals(2.0, anchored.getY(), EPS);
  }

  @Test
  void anchorHierarchicalPointToFuelRejectsWhenNoNearbyFuel() {
    SpatialDyn dyn =
        makeDyn(
            List.of(
                new DynamicObject(
                    "f1", "fuel", new Translation2d(4.0, 2.0), new Translation2d(), 0.05)),
            1.0);

    Translation2d anchored =
        PredictiveCollectSecondaryRankers.anchorHierarchicalPointToFuel(
            dyn, new Translation2d(6.0, 2.0), 0.14, 0.06, 0.03);

    assertNull(anchored);
  }

  @Test
  void anchorHierarchicalPointToFuelRejectsWeakFuel() {
    SpatialDyn dyn =
        makeDyn(
            List.of(
                new DynamicObject(
                    "f1", "fuel", new Translation2d(4.0, 2.0), new Translation2d(), 0.05)),
            0.01);

    Translation2d anchored =
        PredictiveCollectSecondaryRankers.anchorHierarchicalPointToFuel(
            dyn, new Translation2d(4.1, 2.0), 0.14, 0.10, 0.05);

    assertNull(anchored);
  }

  @Test
  void bestCollectHotspotAnchorsToRealFuel() {
    SpatialDyn dyn =
        makeDyn(
            List.of(
                new DynamicObject(
                    "f1", "fuel", new Translation2d(4.0, 2.0), new Translation2d(), 0.05)),
            1.0);

    TestApi api = new TestApi(dyn);
    Translation2d out =
        PredictiveCollectSecondaryRankers.bestCollectHotspot(
            api, new Translation2d[] {new Translation2d(4.30, 2.0)}, 0.75);

    assertNotNull(out);
    assertTrue(out.getDistance(new Translation2d(4.0, 2.0)) <= 0.25);
  }

  @Test
  void rankCollectPointsReturnsNullWhenNoFuelNearCandidates() {
    SpatialDyn dyn =
        makeDyn(
            List.of(
                new DynamicObject(
                    "f1", "fuel", new Translation2d(4.0, 2.0), new Translation2d(), 0.05)),
            1.0);
    TestApi api = new TestApi(dyn);

    PointCandidate out =
        PredictiveCollectSecondaryRankers.rankCollectPoints(
            api,
            new Translation2d(2.0, 2.0),
            3.0,
            new Translation2d[] {new Translation2d(6.2, 2.0)},
            1,
            8);

    assertNull(out);
  }

  @Test
  void rankCollectPointsAnchorsOffFuelCandidateToRealFuel() {
    SpatialDyn dyn =
        makeDyn(
            List.of(
                new DynamicObject(
                    "f1", "fuel", new Translation2d(4.0, 2.0), new Translation2d(), 0.05)),
            1.0);
    TestApi api = new TestApi(dyn);

    PointCandidate out =
        PredictiveCollectSecondaryRankers.rankCollectPoints(
            api,
            new Translation2d(3.2, 2.0),
            3.0,
            new Translation2d[] {new Translation2d(4.30, 2.0)},
            1,
            8);

    assertNotNull(out);
    assertTrue(out.point.getDistance(new Translation2d(4.0, 2.0)) <= 0.25);
  }

  private static final class TestApi implements PredictiveCollectSecondaryRankers.Api {
    private final SpatialDyn dyn;
    private Translation2d[] candidates = new Translation2d[0];

    TestApi(SpatialDyn dyn) {
      this.dyn = dyn;
    }

    @Override
    public SpatialDyn cachedDyn() {
      return dyn;
    }

    @Override
    public void setCollectContext(
        Translation2d ourPos, double ourSpeedCap, int goalUnits, double cellM) {}

    @Override
    public void sweepDepletedMarks() {}

    @Override
    public Translation2d[] buildCollectCandidates(Translation2d[] gridPoints, SpatialDyn dyn) {
      candidates = gridPoints != null ? gridPoints : new Translation2d[0];
      return candidates;
    }

    @Override
    public double dynamicMinUnits(double totalEvidence) {
      return 0.06;
    }

    @Override
    public int dynamicMinCount(double totalEvidence) {
      return 1;
    }

    @Override
    public double minEvidence(double totalEvidence) {
      return 0.03;
    }

    @Override
    public ResourceRegions buildResourceRegions(SpatialDyn dyn, int maxRegions) {
      return new ResourceRegions(new Translation2d[0], new double[0]);
    }

    @Override
    public IntentAggCont enemyIntentToRegions(ResourceRegions regs) {
      return new IntentAggCont(new Translation2d[0], new double[0], 0, 1.0);
    }

    @Override
    public IntentAggCont allyIntentToRegions(ResourceRegions regs) {
      return new IntentAggCont(new Translation2d[0], new double[0], 0, 1.0);
    }

    @Override
    public double estimateTravelTime(Translation2d a, Translation2d b, double speed) {
      return a.getDistance(b) / Math.max(0.2, speed);
    }

    @Override
    public CollectEval evalCollectPoint(
        Translation2d ourPos,
        double ourSpeedCap,
        Translation2d p,
        int goalUnits,
        double cellM,
        SpatialDyn dyn,
        IntentAggCont enemyIntent,
        IntentAggCont allyIntent) {
      CollectEval e = new CollectEval();
      e.p = p;
      e.eta = estimateTravelTime(ourPos, p, ourSpeedCap);
      e.units = dyn.valueInSquare(p, 0.38);
      e.count = dyn.countResourcesWithin(p, 0.42);
      e.evidence = dyn.evidenceMassWithin(p, 0.85);
      e.value = e.units;
      e.enemyPressure = 0.0;
      e.allyCongestion = 0.0;
      e.enemyIntent = 0.0;
      e.allyIntent = 0.0;
      e.depleted = 0.0;
      e.score = e.units - (0.1 * e.eta);
      return e;
    }

    @Override
    public double allyRadialDensity(Translation2d p, double sigma) {
      return 0.0;
    }

    @Override
    public double enemyRadialDensity(Translation2d p, double sigma) {
      return 0.0;
    }

    @Override
    public double regionBanditBonus(SpatialDyn dyn, Translation2d p, double now) {
      return 0.0;
    }

    @Override
    public void addDepletedMark(
        Translation2d p, double radiusM, double strength, double ttlS, boolean merge) {}

    @Override
    public void addDepletedRing(
        Translation2d p, double r0, double r1, double strength, double ttlS) {}

    @Override
    public void setLastReturnedCollect(Translation2d p, double nowS) {}
  }

  private static SpatialDyn makeDyn(List<DynamicObject> objects, double fuelUnitValue) {
    HashMap<String, ResourceSpec> specs = new HashMap<>();
    specs.put("fuel", new ResourceSpec(0.10, fuelUnitValue, 0.06));
    return new SpatialDyn(objects, specs, new HashMap<>(), Set.of("fuel"), p -> true);
  }
}
