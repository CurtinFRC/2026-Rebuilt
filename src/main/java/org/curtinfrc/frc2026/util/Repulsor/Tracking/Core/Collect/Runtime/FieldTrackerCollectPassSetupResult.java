package org.curtinfrc.frc2026.util.Repulsor.Tracking.Core.Collect.Runtime;

import edu.wpi.first.math.geometry.Pose2d;

public record FieldTrackerCollectPassSetupResult(
    FieldTrackerCollectPassContext context, Pose2d immediatePose) {}
