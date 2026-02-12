package org.curtinfrc.frc2026.util.Repulsor.Predictive.Core.Collect.Runtime;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Core.Internal.CollectEval;

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
