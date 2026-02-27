package org.curtinfrc.frc2026.util.Repulsor.Tracking;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import java.util.List;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.DynamicObject;
import org.junit.jupiter.api.Test;

class FieldTrackerDynamicTrackerTest {
  @Test
  void snapshotDynamicsPreservesRealAgeInSimulation() {
    FieldTrackerDynamicTracker tracker = new FieldTrackerDynamicTracker();

    long nowNs = System.nanoTime();
    long oldNs = nowNs - 2_000_000_000L;
    tracker.ingestTracked("fuel-1", "fuel", new Pose3d(2.0, 3.0, 0.0, new Rotation3d()), oldNs);

    List<DynamicObject> dyn = tracker.snapshotDynamics();
    assertEquals(1, dyn.size());
    assertTrue(dyn.get(0).ageS > 1.5);
  }
}
