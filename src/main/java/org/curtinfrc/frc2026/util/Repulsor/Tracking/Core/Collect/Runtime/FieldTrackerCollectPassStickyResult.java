package org.curtinfrc.frc2026.util.Repulsor.Tracking.Core.Collect.Runtime;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public record FieldTrackerCollectPassStickyResult(
    Translation2d desiredCollectPoint, Translation2d desiredDriveTarget, Pose2d immediatePose) {}
