package org.curtinfrc.frc2026.util.Repulsor.Predictive;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.HashMap;
import java.util.List;
import java.util.Set;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.DynamicObject;
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

  private static SpatialDyn makeDyn(List<DynamicObject> objects, double fuelUnitValue) {
    HashMap<String, ResourceSpec> specs = new HashMap<>();
    specs.put("fuel", new ResourceSpec(0.10, fuelUnitValue, 0.06));
    return new SpatialDyn(objects, specs, new HashMap<>(), Set.of("fuel"), p -> true);
  }
}
