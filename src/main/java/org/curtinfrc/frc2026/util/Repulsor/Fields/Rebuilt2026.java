// File: src/main/java/org/curtinfrc/frc2025/util/Repulsor/Fields/Rebuilt2026.java
package org.curtinfrc.frc2026.util.Repulsor.Fields;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import java.util.List;
import org.curtinfrc.frc2026.util.Repulsor.Constants;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.HorizontalObstacle;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacle;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.VerticalObstacle;
import org.curtinfrc.frc2026.util.Repulsor.FieldTracker;
import org.curtinfrc.frc2026.util.Repulsor.FieldTracker.GameElement;
import org.curtinfrc.frc2026.util.Repulsor.FieldTracker.GameElement.Alliance;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldMapBuilder.CategorySpec;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.HeightSetpoint;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.RepulsorSetpoint;

public final class Rebuilt2026 implements FieldDefinition {
  private static final double CORNER_CHAMFER = 0;

  @Override
  public GameElement[] build(FieldTracker ft) {
    var b = new FieldMapBuilder(ft);

    Pose2d blueOutpost = Setpoints.Rebuilt2026.OUTPOST_COLLECT.approximateBluePose();
    Pose2d redOutpost = Setpoints.Rebuilt2026.OUTPOST_COLLECT.approximateRedPose();

    Pose2d blueHub = Setpoints.Rebuilt2026.HUB_SHOOT.approximateBluePose();
    Pose2d redHub = Setpoints.Rebuilt2026.HUB_SHOOT.approximateRedPose();

    b.begin()
        .alliance(Alliance.kBlue)
        .capacity(999)
        .pose(new Pose3d(blueHub.getX(), blueHub.getY(), 0, null))
        .related(new RepulsorSetpoint(Setpoints.Rebuilt2026.HUB_SHOOT, HeightSetpoint.NET))
        .category(CategorySpec.kScore)
        .add();

    b.begin()
        .alliance(Alliance.kRed)
        .capacity(999)
        .pose(new Pose3d(redHub.getX(), redHub.getY(), 0, null))
        .related(new RepulsorSetpoint(Setpoints.Rebuilt2026.HUB_SHOOT, HeightSetpoint.NET))
        .category(CategorySpec.kScore)
        .add();

    b.begin()
        .alliance(Alliance.kBlue)
        .capacity(999)
        .pose(new Pose3d(blueOutpost.getX(), blueOutpost.getY(), 0, null))
        .related(new RepulsorSetpoint(Setpoints.Rebuilt2026.OUTPOST_COLLECT, HeightSetpoint.NONE))
        .category(CategorySpec.kCollect)
        .add();

    b.begin()
        .alliance(Alliance.kRed)
        .capacity(999)
        .pose(new Pose3d(redOutpost.getX(), redOutpost.getY(), 0, null))
        .related(new RepulsorSetpoint(Setpoints.Rebuilt2026.OUTPOST_COLLECT, HeightSetpoint.NONE))
        .category(CategorySpec.kCollect)
        .add();

    return b.build();
  }

  @Override
  public String gameName() {
    return "REBUILT";
  }

  @Override
  public int gameYear() {
    return 2026;
  }

  @Override
  public List<Obstacle> fieldObstacles() {
    return List.of();
  }

  @Override
  public List<Obstacle> walls() {
    return List.of(
        new HorizontalObstacle(0.0, 2, true),
        new HorizontalObstacle(Constants.FIELD_WIDTH, 1.4, false),
        new VerticalObstacle(0.0, 2, true),
        new VerticalObstacle(Constants.FIELD_LENGTH, 1.4, false)
        // new DiagonalWallObstacle(
        //     new edu.wpi.first.math.geometry.Translation2d(0.0, CORNER_CHAMFER),
        //     new edu.wpi.first.math.geometry.Translation2d(CORNER_CHAMFER, 0.0),
        //     2.0,
        //     2.0),
        // new DiagonalWallObstacle(
        //     new edu.wpi.first.math.geometry.Translation2d(
        //         Constants.FIELD_LENGTH - CORNER_CHAMFER, 0.0),
        //     new edu.wpi.first.math.geometry.Translation2d(Constants.FIELD_LENGTH,
        // CORNER_CHAMFER),
        //     2.0,
        //     2.0),
        // new DiagonalWallObstacle(
        //     new edu.wpi.first.math.geometry.Translation2d(
        //         0.0, Constants.FIELD_WIDTH - CORNER_CHAMFER),
        //     new edu.wpi.first.math.geometry.Translation2d(CORNER_CHAMFER, Constants.FIELD_WIDTH),
        //     2.0,
        //     2.0),
        // new DiagonalWallObstacle(
        //     new edu.wpi.first.math.geometry.Translation2d(
        //         Constants.FIELD_LENGTH - CORNER_CHAMFER, Constants.FIELD_WIDTH),
        //     new edu.wpi.first.math.geometry.Translation2d(
        //         Constants.FIELD_LENGTH, Constants.FIELD_WIDTH - CORNER_CHAMFER),
        //     2.0,
        //     2.0)
        );
  }
}
