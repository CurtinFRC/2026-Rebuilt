package org.curtinfrc.frc2026.util.Repulsor.Offload;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.List;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.FieldTrackerLocalAccess;

@SuppressWarnings("unused")
public final class FieldTrackerOffloadEntrypoints {
  private FieldTrackerOffloadEntrypoints() {}

  @Offloadable(
      id = OffloadTaskIds.FIELD_TRACKER_NEXT_SHUTTLE_RECOVERY_GOAL_BLUE,
      version = 1,
      timeoutMs = 16,
      fallback = true)
  public static Pose2d nextShuttleRecoveryGoalBlue(
      Pose2d robotPoseBlue,
      double ourSpeedCap,
      int goalUnits,
      boolean flipRedToBlue,
      List<ShuttleRecoveryDynamicObjectDTO> dynamicObjects) {
    return FieldTrackerLocalAccess.nextAllianceShuttleRecoveryGoalBlueLocal(
        robotPoseBlue, ourSpeedCap, goalUnits, flipRedToBlue, dynamicObjects);
  }
}
