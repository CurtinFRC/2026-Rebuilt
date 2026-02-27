package org.curtinfrc.frc2026.util.Repulsor.Tracking.Collect.Runtime;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class FieldTrackerCollectPassDriveStepTest {
  @Test
  void shouldAutoSwitchFromStillFalseBelowThreshold() {
    assertFalse(FieldTrackerCollectPassDriveStep.shouldAutoSwitchFromStill(0.99));
  }

  @Test
  void shouldAutoSwitchFromStillTrueAtThreshold() {
    assertTrue(FieldTrackerCollectPassDriveStep.shouldAutoSwitchFromStill(1.0));
  }

  @Test
  void shouldForceSwitchFromHubFrontTrapRequiresTrapAndTrigger() {
    assertTrue(
        FieldTrackerCollectPassDriveStep.shouldForceSwitchFromHubFrontTrap(
            true, 0.40, 0.0, 0.0, 0, 0.0));
    assertTrue(
        FieldTrackerCollectPassDriveStep.shouldForceSwitchFromHubFrontTrap(
            true, 0.0, 0.25, 0.0, 0, 0.0));
    assertTrue(
        FieldTrackerCollectPassDriveStep.shouldForceSwitchFromHubFrontTrap(
            true, 0.0, 0.0, 0.20, 0, 0.0));
    assertTrue(
        FieldTrackerCollectPassDriveStep.shouldForceSwitchFromHubFrontTrap(
            true, 0.0, 0.0, 0.0, 2, 1.0));

    assertFalse(
        FieldTrackerCollectPassDriveStep.shouldForceSwitchFromHubFrontTrap(
            false, 1.0, 1.0, 1.0, 0, 0.0));
    assertFalse(
        FieldTrackerCollectPassDriveStep.shouldForceSwitchFromHubFrontTrap(
            true, 0.40, 0.0, 0.0, 3, 1.0));
  }
}
