package org.curtinfrc.frc2026.util.Repulsor.Tracking.Core.Collect.Runtime;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import java.util.function.Function;
import java.util.function.Predicate;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Core.Model.DynamicObject;

public record FieldTrackerCollectPassContext(
    Pose2d robotPoseBlue,
    Translation2d robotPos,
    double cap,
    Translation2d[] usePts,
    boolean robotInCenterBand,
    List<DynamicObject> dynAll,
    List<DynamicObject> dynUse,
    int lockHalf,
    int sensedHalf,
    long nowNs,
    double dt,
    double midX,
    double leftBandX0,
    double leftBandX1,
    double rightBandX0,
    double rightBandX1,
    Function<Translation2d, Translation2d> clampToFieldRobotSafe,
    Predicate<Translation2d> inForbidden,
    Predicate<Translation2d> violatesWall,
    Function<Translation2d, Translation2d> nudgeOutOfForbidden,
    Function<Translation2d, Translation2d> safePushedFromRobot,
    Function<Translation2d, Pose2d> holdPose,
    int nearbyFuelCount,
    Translation2d nearbyCentroid) {}
