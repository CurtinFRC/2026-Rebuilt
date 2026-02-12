package org.curtinfrc.frc2026.util.Repulsor.Tracking.Core.Collect.Runtime;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.function.Function;
import java.util.function.Predicate;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Core.Model.PointCandidate;

public record FieldTrackerCollectPassCandidateResult(
    PointCandidate best,
    Translation2d bestCandidate,
    Predicate<Translation2d> collectValid,
    Predicate<Translation2d> footprintHasFuel,
    Function<Translation2d, Double> scoreResource,
    Pose2d immediatePose) {}
