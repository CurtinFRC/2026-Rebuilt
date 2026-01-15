package org.curtinfrc.frc2026.util.Repulsor.Fields;

import static org.curtinfrc.frc2026.util.Repulsor.Constants.FIELD_LENGTH;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import org.curtinfrc.frc2026.util.Repulsor.Constants;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.HorizontalObstacle;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacle;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.RectangleObstacle;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.SquareObstacle;
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

  private static final double GRID_CELL_M = 0.25;
  private static final double GRID_Z_MIN_M = -0.05;
  private static final double GRID_Z_MAX_M = 0.35;

  private static final double GRID_REGION_HALF_X_M = 4.0;
  private static final double GRID_REGION_HALF_Y_M = 2.0;

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

    double cx = Constants.FIELD_LENGTH * 0.5;
    double cy = Constants.FIELD_WIDTH * 0.5;

    double x0 = cx - GRID_REGION_HALF_X_M;
    double x1 = cx + GRID_REGION_HALF_X_M;
    double y0 = cy - GRID_REGION_HALF_Y_M;
    double y1 = cy + GRID_REGION_HALF_Y_M;

    int nx = (int) Math.floor((x1 - x0) / GRID_CELL_M);
    int ny = (int) Math.floor((y1 - y0) / GRID_CELL_M);

    for (int ix = 0; ix <= nx; ix++) {
      double x = x0 + ix * GRID_CELL_M + GRID_CELL_M * 0.5;
      for (int iy = 0; iy <= ny; iy++) {
        double y = y0 + iy * GRID_CELL_M + GRID_CELL_M * 0.5;
        b.begin()
            .alliance(Alliance.kBlue)
            .capacity(999)
            .pose(new Pose3d(x, y, 0, null))
            .primitiveFloorSquare(GRID_CELL_M, GRID_Z_MIN_M, GRID_Z_MAX_M)
            .filterType("fuel")
            .category(CategorySpec.kCollect)
            .add();
      }
    }

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
    double maxRangeY = 0.9;
    double maxRangeX = 1.2;
    double strength = 1;
    return List.of(
        new SquareObstacle(new Translation2d(4.625594, Constants.FIELD_WIDTH / 2), 1.1938, 2, 3),
        new SquareObstacle(
            new Translation2d(FIELD_LENGTH - 4.625594, Constants.FIELD_WIDTH / 2), 1.1938, 2, 3),
        new RectangleObstacle(
            new Translation2d((FIELD_LENGTH / 2) - 3.63982, 1.4224),
            1.1938,
            0.255336,
            strength,
            maxRangeX,
            maxRangeY),
        new RectangleObstacle(
            new Translation2d((FIELD_LENGTH / 2) - 3.63982, Constants.FIELD_WIDTH - 1.4224),
            1.1938,
            0.255336,
            strength,
            maxRangeX,
            maxRangeY),
        new RectangleObstacle(
            new Translation2d((FIELD_LENGTH / 2) + 3.63982, 1.4224),
            1.1938,
            0.255336,
            strength,
            maxRangeX,
            maxRangeY),
        new RectangleObstacle(
            new Translation2d((FIELD_LENGTH / 2) + 3.63982, Constants.FIELD_WIDTH - 1.4224),
            1.1938,
            0.255336,
            strength,
            maxRangeX,
            maxRangeY));
  }

  @Override
  public List<Obstacle> walls() {
    return List.of(
        new HorizontalObstacle(0.0, 1, true),
        new HorizontalObstacle(Constants.FIELD_WIDTH, 1, false),
        new VerticalObstacle(0.0, 2, true),
        new VerticalObstacle(Constants.FIELD_LENGTH, 2, false));
  }
}
