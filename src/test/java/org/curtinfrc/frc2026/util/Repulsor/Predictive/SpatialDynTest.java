package org.curtinfrc.frc2026.util.Repulsor.Predictive;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.HashMap;
import java.util.List;
import java.util.Set;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.DynamicObject;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.ResourceSpec;
import org.junit.jupiter.api.Test;

class SpatialDynTest {
  private static final double EPS = 1e-9;

  @Test
  void nearestResourceToOnlyUsesCollectTypes() {
    HashMap<String, ResourceSpec> specs = new HashMap<>();
    specs.put("fuel", new ResourceSpec(0.10, 1.0, 0.06));
    specs.put("algae", new ResourceSpec(0.10, 1.0, 0.06));

    SpatialDyn dyn =
        new SpatialDyn(
            List.of(
                new DynamicObject(
                    "a1", "algae", new Translation2d(2.00, 2.00), new Translation2d(), 0.05),
                new DynamicObject(
                    "f1", "fuel", new Translation2d(2.30, 2.00), new Translation2d(), 0.05)),
            specs,
            new HashMap<>(),
            Set.of("fuel"),
            p -> true);

    Translation2d nearTight = dyn.nearestResourceTo(new Translation2d(2.00, 2.00), 0.12);
    Translation2d nearWide = dyn.nearestResourceTo(new Translation2d(2.00, 2.00), 0.40);

    assertNull(nearTight);
    assertEquals(2.30, nearWide.getX(), EPS);
    assertEquals(2.00, nearWide.getY(), EPS);
  }
}
