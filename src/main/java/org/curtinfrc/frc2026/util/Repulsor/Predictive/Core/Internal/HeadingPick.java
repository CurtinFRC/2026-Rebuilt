package org.curtinfrc.frc2026.util.Repulsor.Predictive.Core.Internal;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public final class HeadingPick {
  public final Translation2d center;
  public final Rotation2d heading;
  public final FootprintEval eval;

  public HeadingPick(Translation2d center, Rotation2d heading, FootprintEval eval) {
    this.center = center;
    this.heading = heading;
    this.eval = eval;
  }
}
