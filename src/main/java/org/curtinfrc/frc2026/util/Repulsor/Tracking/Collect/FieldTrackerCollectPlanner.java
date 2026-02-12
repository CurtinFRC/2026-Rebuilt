/*
 * Copyright (C) 2026 Paul Hodges
 *
 * This file is part of Repulsor.
 *
 * Repulsor is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Repulsor is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Repulsor. If not, see https://www.gnu.org/licenses/.
 */
package org.curtinfrc.frc2026.util.Repulsor.Tracking.Collect;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import java.util.function.Predicate;
import java.util.function.Supplier;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldMapBuilder.CategorySpec;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.DynamicObject;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.PredictiveFieldStateRuntime;

public final class FieldTrackerCollectPlanner {
  private final FieldTrackerCollectObjectiveRuntime engine;

  public FieldTrackerCollectPlanner(
      PredictiveFieldStateRuntime predictor,
      Supplier<Translation2d[]> collectObjectivePoints,
      Supplier<List<DynamicObject>> dynamicsSupplier,
      Predicate<String> collectTypePredicate) {
    this.engine =
        new FieldTrackerCollectObjectiveRuntime(
            predictor, collectObjectivePoints, dynamicsSupplier, collectTypePredicate);
  }

  public void resetAll() {
    engine.resetAll();
  }

  public Pose2d nextObjectiveGoalBlue(
      Pose2d robotPoseBlue, double ourSpeedCap, int goalUnits, CategorySpec cat) {
    return engine.nextObjectiveGoalBlue(robotPoseBlue, ourSpeedCap, goalUnits, cat);
  }

  public void clearState() {
    engine.clearState();
  }
}
