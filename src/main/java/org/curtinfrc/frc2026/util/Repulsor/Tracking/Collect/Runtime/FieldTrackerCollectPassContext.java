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
package org.curtinfrc.frc2026.util.Repulsor.Tracking.Collect.Runtime;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import java.util.function.Function;
import java.util.function.Predicate;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.DynamicObject;

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
