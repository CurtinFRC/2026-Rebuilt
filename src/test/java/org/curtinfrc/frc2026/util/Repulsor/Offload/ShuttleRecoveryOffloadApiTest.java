package org.curtinfrc.frc2026.util.Repulsor.Offload;

import static org.junit.jupiter.api.Assertions.assertNotNull;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.List;
import org.junit.jupiter.api.Test;

class ShuttleRecoveryOffloadApiTest {
  @Test
  void shouldExposeNewOffloadTaskIds() {
    assertNotNull(OffloadTaskIds.PREDICTIVE_SELECT_SHUTTLE_RECOVERY_POINT);
    assertNotNull(OffloadTaskIds.FIELD_TRACKER_NEXT_SHUTTLE_RECOVERY_GOAL_BLUE);
  }

  @Test
  void shouldGeneratePredictiveAndTrackerOffloadWrappers() throws Exception {
    Class<?> predictiveWrapper =
        Class.forName(
            "org.curtinfrc.frc2026.util.Repulsor.Offload.PredictiveFieldStateOffloadEntrypoints_Offloaded");
    Class<?> predictiveRequest =
        Class.forName(
            "org.curtinfrc.frc2026.util.Repulsor.Offload.PredictiveFieldStateOffloadEntrypoints_selectShuttleRecoveryPoint_OffloadRequest");

    assertNotNull(
        predictiveWrapper.getMethod(
            "selectShuttleRecoveryPoint_offload",
            Pose2d.class,
            double.class,
            int.class,
            boolean.class,
            List.class));
    assertNotNull(
        predictiveWrapper.getMethod(
            "selectShuttleRecoveryPoint_offloadAsync",
            Pose2d.class,
            double.class,
            int.class,
            boolean.class,
            List.class));
    assertNotNull(
        predictiveWrapper.getMethod(
            "selectShuttleRecoveryPoint_offloadExecute", predictiveRequest));

    Class<?> trackerWrapper =
        Class.forName(
            "org.curtinfrc.frc2026.util.Repulsor.Offload.FieldTrackerOffloadEntrypoints_Offloaded");
    Class<?> trackerRequest =
        Class.forName(
            "org.curtinfrc.frc2026.util.Repulsor.Offload.FieldTrackerOffloadEntrypoints_nextShuttleRecoveryGoalBlue_OffloadRequest");

    assertNotNull(
        trackerWrapper.getMethod(
            "nextShuttleRecoveryGoalBlue_offload",
            Pose2d.class,
            double.class,
            int.class,
            boolean.class,
            List.class));
    assertNotNull(
        trackerWrapper.getMethod(
            "nextShuttleRecoveryGoalBlue_offloadAsync",
            Pose2d.class,
            double.class,
            int.class,
            boolean.class,
            List.class));
    assertNotNull(
        trackerWrapper.getMethod("nextShuttleRecoveryGoalBlue_offloadExecute", trackerRequest));
  }
}
