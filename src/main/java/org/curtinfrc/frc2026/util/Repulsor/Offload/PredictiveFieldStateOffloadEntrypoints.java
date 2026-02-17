package org.curtinfrc.frc2026.util.Repulsor.Offload;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.List;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.PredictiveFieldStateLocalAccess;

@SuppressWarnings("unused")
public final class PredictiveFieldStateOffloadEntrypoints {
  private PredictiveFieldStateOffloadEntrypoints() {}

  @Offloadable(
      id = OffloadTaskIds.PREDICTIVE_SELECT_SHUTTLE_RECOVERY_POINT,
      version = 1,
      timeoutMs = 16,
      fallback = true)
  public static ShuttleRecoveryPointDTO selectShuttleRecoveryPoint(
      Pose2d robotPoseBlue,
      double ourSpeedCap,
      int goalUnits,
      boolean flipRedToBlue,
      List<ShuttleRecoveryDynamicObjectDTO> dynamicObjects) {
    return PredictiveFieldStateLocalAccess.selectShuttleRecoveryPointLocal(
        robotPoseBlue, ourSpeedCap, goalUnits, flipRedToBlue, dynamicObjects);
  }
}
