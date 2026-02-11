package org.curtinfrc.frc2026.util.Repulsor.Tracking.Core.Collect;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import java.util.function.Predicate;
import java.util.function.Supplier;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldMapBuilder.CategorySpec;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Core.Model.DynamicObject;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Core.PredictiveFieldStateCore;

final class FieldTrackerCollectObjectiveRuntime {
  private final FieldTrackerCollectObjectiveLoop loop;

  FieldTrackerCollectObjectiveRuntime(
      PredictiveFieldStateCore predictor,
      Supplier<Translation2d[]> collectObjectivePoints,
      Supplier<List<DynamicObject>> dynamicsSupplier,
      Predicate<String> collectTypePredicate) {
    this.loop =
        new FieldTrackerCollectObjectiveLoop(
            predictor, collectObjectivePoints, dynamicsSupplier, collectTypePredicate);
  }

  void resetAll() {
    loop.resetAll();
  }

  Pose2d nextObjectiveGoalBlue(
      Pose2d robotPoseBlue, double ourSpeedCap, int goalUnits, CategorySpec cat) {
    return loop.nextObjectiveGoalBlue(robotPoseBlue, ourSpeedCap, goalUnits, cat);
  }

  void clearState() {
    loop.clearState();
  }
}
