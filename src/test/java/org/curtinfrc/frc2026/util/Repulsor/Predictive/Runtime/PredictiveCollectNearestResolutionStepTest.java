package org.curtinfrc.frc2026.util.Repulsor.Predictive.Runtime;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Translation2d;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Internal.CollectEval;
import org.junit.jupiter.api.Test;

class PredictiveCollectNearestResolutionStepTest {
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
}
