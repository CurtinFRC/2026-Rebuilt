package org.curtinfrc.frc2026.util.Repulsor.Fields;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.Arrays;
import java.util.List;
import org.curtinfrc.frc2026.util.Repulsor.Constants;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacle;
import org.curtinfrc.frc2026.util.Repulsor.FieldTracker;
import org.curtinfrc.frc2026.util.Repulsor.FieldTracker.GameElement;
import org.curtinfrc.frc2026.util.Repulsor.FieldTracker.GameElement.Alliance;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldMapBuilder.CategorySpec;
import org.curtinfrc.frc2026.util.Repulsor.Heatmap;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.HeightSetpoint;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.RepulsorSetpoint;

public final class Reefscape2025 implements FieldDefinition {

  private static final double CORNER_CHAMFER = 1.5;

  private static final List<Obstacle> FIELD_OBSTACLES =
      List.of(
          new FieldPlanner.TeardropObstacle(
              new Translation2d(4.49, 4.00), 1.2, 2.2, 1.03, 3.0, 2.0),
          new FieldPlanner.TeardropObstacle(
              new Translation2d(13.08, 4.00), 1.2, 2.2, 1.03, 3.0, 2.0));

  private static final List<Obstacle> WALLS =
      List.of(
          new FieldPlanner.HorizontalObstacle(0.0, 2.0, true),
          new FieldPlanner.HorizontalObstacle(Constants.FIELD_WIDTH, 1.4, false),
          new FieldPlanner.VerticalObstacle(0.0, 2.0, true),
          new FieldPlanner.VerticalObstacle(Constants.FIELD_LENGTH, 1.4, false),
          new FieldPlanner.DiagonalWallObstacle(
              new Translation2d(0.0, CORNER_CHAMFER),
              new Translation2d(CORNER_CHAMFER, 0.0),
              2.0,
              2.0),
          new FieldPlanner.DiagonalWallObstacle(
              new Translation2d(Constants.FIELD_LENGTH - CORNER_CHAMFER, 0.0),
              new Translation2d(Constants.FIELD_LENGTH, CORNER_CHAMFER),
              2.0,
              2.0),
          new FieldPlanner.DiagonalWallObstacle(
              new Translation2d(0.0, Constants.FIELD_WIDTH - CORNER_CHAMFER),
              new Translation2d(CORNER_CHAMFER, Constants.FIELD_WIDTH),
              2.0,
              2.0),
          new FieldPlanner.DiagonalWallObstacle(
              new Translation2d(Constants.FIELD_LENGTH - CORNER_CHAMFER, Constants.FIELD_WIDTH),
              new Translation2d(Constants.FIELD_LENGTH, Constants.FIELD_WIDTH - CORNER_CHAMFER),
              2.0,
              2.0));

  @Override
  public List<Obstacle> fieldObstacles() {
    return FIELD_OBSTACLES;
  }

  @Override
  public List<Obstacle> walls() {
    return WALLS;
  }

  @Override
  public GameElement[] build(FieldTracker ft) {
    var b = new FieldMapBuilder(ft);

    List<Pose3d> blueFaces =
        Arrays.asList(
            new Pose3d(1.00, 0.00, 0, null),
            new Pose3d(0.50, 0.86, 0, null),
            new Pose3d(-0.50, 0.86, 0, null),
            new Pose3d(-1.00, 0.00, 0, null),
            new Pose3d(-0.50, -0.86, 0, null),
            new Pose3d(0.50, -0.86, 0, null));

    List<RepulsorSetpoint> reefSP =
        Arrays.asList(
            new RepulsorSetpoint(Setpoints.Reefscape2025.A, HeightSetpoint.L2),
            new RepulsorSetpoint(Setpoints.Reefscape2025.B, HeightSetpoint.L2),
            new RepulsorSetpoint(Setpoints.Reefscape2025.C, HeightSetpoint.L2),
            new RepulsorSetpoint(Setpoints.Reefscape2025.D, HeightSetpoint.L2),
            new RepulsorSetpoint(Setpoints.Reefscape2025.E, HeightSetpoint.L2),
            new RepulsorSetpoint(Setpoints.Reefscape2025.F, HeightSetpoint.L2));

    b.bulk(
        blueFaces,
        Alliance.kBlue,
        1,
        FieldMapBuilder.small(),
        0,
        go -> true,
        reefSP,
        CategorySpec.kScore);

    b.begin()
        .alliance(Alliance.kBlue)
        .capacity(6)
        .pose(new Pose3d(2.50, 3.00, 0, null))
        .primitivePipe(FieldMapBuilder.medium(), 0, 0, 0)
        .filterType("coral")
        .related(
            new RepulsorSetpoint(Setpoints.Reefscape2025.LEFT_HP, HeightSetpoint.CORAL_STATION))
        .category(CategorySpec.kCollect)
        .add();

    b.begin()
        .alliance(Alliance.kBlue)
        .capacity(6)
        .pose(new Pose3d(2.50, -3.00, 0, null))
        .primitivePipe(FieldMapBuilder.medium(), 0, 0, 0)
        .filterType("coral")
        .related(
            new RepulsorSetpoint(Setpoints.Reefscape2025.RIGHT_HP, HeightSetpoint.CORAL_STATION))
        .category(CategorySpec.kCollect)
        .add();

    b.begin()
        .alliance(Alliance.kBlue)
        .capacity(3)
        .pose(new Pose3d(0.75, 3.80, 0, null))
        .category(CategorySpec.kEndgame)
        .add();

    b.begin()
        .alliance(Alliance.kBlue)
        .capacity(3)
        .pose(new Pose3d(0.75, -3.80, 0, null))
        .category(CategorySpec.kEndgame)
        .add();

    List<Pose3d> redFaces =
        Arrays.asList(
            new Pose3d(15.00, 0.00, 0, null),
            new Pose3d(14.50, 0.86, 0, null),
            new Pose3d(13.50, 0.86, 0, null),
            new Pose3d(13.00, 0.00, 0, null),
            new Pose3d(13.50, -0.86, 0, null),
            new Pose3d(14.50, -0.86, 0, null));

    b.bulk(
        redFaces,
        Alliance.kRed,
        1,
        FieldMapBuilder.small(),
        0,
        go -> true,
        reefSP,
        CategorySpec.kScore);

    b.begin()
        .alliance(Alliance.kRed)
        .capacity(6)
        .pose(new Pose3d(11.50, 3.00, 0, null))
        .primitivePipe(FieldMapBuilder.medium(), 0, 0, 0)
        .filterType("coral")
        .related(
            new RepulsorSetpoint(Setpoints.Reefscape2025.LEFT_HP, HeightSetpoint.CORAL_STATION))
        .category(CategorySpec.kCollect)
        .add();

    b.begin()
        .alliance(Alliance.kRed)
        .capacity(6)
        .pose(new Pose3d(11.50, -3.00, 0, null))
        .primitivePipe(FieldMapBuilder.medium(), 0, 0, 0)
        .filterType("coral")
        .related(
            new RepulsorSetpoint(Setpoints.Reefscape2025.RIGHT_HP, HeightSetpoint.CORAL_STATION))
        .category(CategorySpec.kCollect)
        .add();

    b.begin()
        .alliance(Alliance.kRed)
        .capacity(3)
        .pose(new Pose3d(15.75, 3.80, 0, null))
        .category(CategorySpec.kEndgame)
        .add();

    b.begin()
        .alliance(Alliance.kRed)
        .capacity(3)
        .pose(new Pose3d(15.75, -3.80, 0, null))
        .category(CategorySpec.kEndgame)
        .add();

    return b.build();
  }

  @Override
  public Heatmap getHeatmap() {
    return Heatmap.builder().build();
  }

  @Override
  public String gameName() {
    return "REEFSCAPE";
  }

  @Override
  public int gameYear() {
    return 2025;
  }
}
