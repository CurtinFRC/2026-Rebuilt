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
package org.curtinfrc.frc2026.util.Repulsor.Predictive.Collect.Runtime;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Internal.CollectEval;

public record PredictiveCollectNearestSearchResult(
    Translation2d bestPoint,
    Translation2d bestTouch,
    Rotation2d bestHeading,
    CollectEval bestEval,
    double[] topScore,
    double[] topEta,
    double[] topUnits,
    double[] topRegion,
    double[] topDep,
    double[] topAvoid,
    double[] topEnemyPressure,
    double[] topAllyCongestion,
    double[] topActivity,
    double[] topEvidence,
    int topCount) {}
