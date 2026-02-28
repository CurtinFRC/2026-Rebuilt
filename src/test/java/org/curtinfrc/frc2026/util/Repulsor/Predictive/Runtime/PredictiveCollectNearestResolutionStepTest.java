package org.curtinfrc.frc2026.util.Repulsor.Predictive.Runtime;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Translation2d;
import java.lang.reflect.Constructor;
import java.util.HashMap;
import java.util.List;
import java.util.Set;
import java.util.function.Predicate;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Internal.CollectEval;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.DynamicObject;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.ResourceSpec;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.SpatialDyn;
import org.junit.jupiter.api.Test;

class PredictiveCollectNearestResolutionStepTest {
  private static final double EPS = 1e-9;

  @Test
  void allowCommitWindowRicherSwitchTrueForClearlyRicherCandidate() {
    CollectEval current = new CollectEval();
    current.units = 0.10;
    current.eta = 1.20;
    current.score = 2.00;

    CollectEval candidate = new CollectEval();
    candidate.units = 0.24;
    candidate.eta = 1.65;
    candidate.score = 1.80;

    assertTrue(
        PredictiveCollectNearestResolutionStep.allowCommitWindowRicherSwitch(current, candidate));
  }

  @Test
  void allowCommitWindowRicherSwitchFalseWhenEtaTooFar() {
    CollectEval current = new CollectEval();
    current.units = 0.10;
    current.eta = 1.20;
    current.score = 2.00;

    CollectEval candidate = new CollectEval();
    candidate.units = 0.24;
    candidate.eta = 2.20;
    candidate.score = 1.80;

    assertFalse(
        PredictiveCollectNearestResolutionStep.allowCommitWindowRicherSwitch(current, candidate));
  }

  @Test
  void shouldDropEscapedCurrentTargetTrueWhenEscapingSamePoint() {
    Translation2d cur = new Translation2d(2.0, 1.0);
    Translation2d chosen = new Translation2d(2.02, 1.0);
    assertTrue(
        PredictiveCollectNearestResolutionStep.shouldDropEscapedCurrentTarget(true, cur, chosen));
  }

  @Test
  void shouldDropEscapedCurrentTargetFalseWhenChosenDiffers() {
    Translation2d cur = new Translation2d(2.0, 1.0);
    Translation2d chosen = new Translation2d(2.20, 1.0);
    assertFalse(
        PredictiveCollectNearestResolutionStep.shouldDropEscapedCurrentTarget(true, cur, chosen));
  }

  @Test
  void nearestFreshResourceToPrefersFreshFuel() {
    SpatialDyn dyn =
        makeDyn(
            List.of(
                new DynamicObject(
                    "stale", "fuel", new Translation2d(4.00, 2.00), new Translation2d(), 0.80),
                new DynamicObject(
                    "fresh", "fuel", new Translation2d(4.10, 2.00), new Translation2d(), 0.05)),
            1.0);

    Translation2d out =
        PredictiveCollectNearestResolutionStep.nearestFreshResourceTo(
            dyn, new Translation2d(4.02, 2.00), 0.25, 0.45);

    assertNotNull(out);
    assertTrue(Math.abs(out.getX() - 4.10) <= EPS);
    assertTrue(Math.abs(out.getY() - 2.00) <= EPS);
  }

  @Test
  void nearestFreshResourceToRejectsStaleOnlyFuel() {
    SpatialDyn dyn =
        makeDyn(
            List.of(
                new DynamicObject(
                    "stale", "fuel", new Translation2d(4.00, 2.00), new Translation2d(), 0.80)),
            1.0);

    Translation2d out =
        PredictiveCollectNearestResolutionStep.nearestFreshResourceTo(
            dyn, new Translation2d(4.00, 2.00), 0.25, 0.45);

    assertNull(out);
  }

  private static SpatialDyn makeDyn(List<DynamicObject> objects, double fuelUnitValue) {
    try {
      HashMap<String, ResourceSpec> specs = new HashMap<>();
      specs.put("fuel", new ResourceSpec(0.10, fuelUnitValue, 0.06));
      Constructor<SpatialDyn> ctor =
          SpatialDyn.class.getDeclaredConstructor(
              List.class, HashMap.class, HashMap.class, Set.class, Predicate.class);
      ctor.setAccessible(true);
      return ctor.newInstance(
          objects, specs, new HashMap<>(), Set.of("fuel"), (Predicate<Translation2d>) p -> true);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }
  }
}
