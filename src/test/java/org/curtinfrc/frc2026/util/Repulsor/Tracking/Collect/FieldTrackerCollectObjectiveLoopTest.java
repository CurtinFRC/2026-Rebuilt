package org.curtinfrc.frc2026.util.Repulsor.Tracking.Collect;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.DynamicObject;
import org.junit.jupiter.api.Test;

class FieldTrackerCollectObjectiveLoopTest {
  @Test
  void countLiveCollectResourcesWithinIgnoresStaleObservations() {
    FieldTrackerCollectObjectiveLoop loop =
        new FieldTrackerCollectObjectiveLoop(
            null,
            () -> new Translation2d[] {new Translation2d(1.0, 2.0)},
            List::of,
            type -> "fuel".equalsIgnoreCase(type));

    Translation2d center = new Translation2d(1.0, 2.0);
    List<DynamicObject> dyn =
        List.of(
            new DynamicObject("fresh", "fuel", center, new Translation2d(), 0.10),
            new DynamicObject(
                "stale",
                "fuel",
                new Translation2d(1.02, 2.0),
                new Translation2d(),
                FieldTrackerCollectObjectiveLoop.COLLECT_LIVE_OBS_MAX_AGE_S + 0.20),
            new DynamicObject("nonfuel", "other", center, new Translation2d(), 0.05));

    int n = loop.countLiveCollectResourcesWithin(dyn, center, 0.10);
    assertEquals(1, n);
  }
}
