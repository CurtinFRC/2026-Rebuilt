package org.curtinfrc.frc2026.util.Repulsor.Offload;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.FieldPlanner;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacle;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacles.HorizontalObstacle;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacles.PointObstacle;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacles.SnowmanObstacle;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacles.SquareObstacle;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacles.VerticalObstacle;
import org.curtinfrc.frc2026.util.Repulsor.Force;
import org.curtinfrc.frc2026.util.Repulsor.Shooting.Constraints;
import org.curtinfrc.frc2026.util.Repulsor.Shooting.DragShotPlannerLocalAccess;
import org.curtinfrc.frc2026.util.Repulsor.Shooting.GamePiecePhysics;
import org.curtinfrc.frc2026.util.Repulsor.Shooting.ShotSolution;
import org.curtinfrc.frc2026.util.Repulsor.VisionPlanner;

public final class DragShotOffloadMapper {
  private DragShotOffloadMapper() {}

  public static DragShotAutoRequestDTO toRequest(
      GamePiecePhysics gamePiece,
      Translation2d targetFieldPosition,
      double targetHeightMeters,
      Pose2d robotPose,
      double shooterReleaseHeightMeters,
      double robotHalfLengthMeters,
      double robotHalfWidthMeters,
      List<? extends Obstacle> dynamicObstacles,
      Constraints constraints) {

    DragShotAutoRequestDTO request = new DragShotAutoRequestDTO();
    request.setGamePiece(toGamePieceDto(gamePiece));
    request.setTargetFieldPosition(toTranslationDto(targetFieldPosition));
    request.setTargetHeightMeters(targetHeightMeters);
    request.setRobotPose(toPoseDto(robotPose));
    request.setShooterReleaseHeightMeters(shooterReleaseHeightMeters);
    request.setRobotHalfLengthMeters(robotHalfLengthMeters);
    request.setRobotHalfWidthMeters(robotHalfWidthMeters);
    request.setConstraints(toConstraintsDto(constraints));

    List<ObstacleDTO> obstacleDtos = new ArrayList<>();
    if (dynamicObstacles != null) {
      for (Obstacle obstacle : dynamicObstacles) {
        ObstacleDTO dto = toObstacleDto(obstacle);
        if (dto == null) {
          return null;
        }
        obstacleDtos.add(dto);
      }
    }
    request.setDynamicObstacles(obstacleDtos);

    return request;
  }

  public static DragShotAutoResponseDTO executeLocal(DragShotAutoRequestDTO request) {
    if (request == null) {
      return DragShotAutoResponseDTO.empty();
    }

    GamePiecePhysics gamePiece = fromGamePieceDto(request.getGamePiece());
    Translation2d targetFieldPosition = fromTranslationDto(request.getTargetFieldPosition());
    Pose2d robotPose = fromPoseDto(request.getRobotPose());
    List<Obstacle> dynamicObstacles = fromObstacleDtos(request.getDynamicObstacles());
    Constraints constraints = fromConstraintsDto(request.getConstraints());

    Optional<ShotSolution> solution =
        DragShotPlannerLocalAccess.findBestShotAutoLocal(
            gamePiece,
            targetFieldPosition,
            request.getTargetHeightMeters(),
            robotPose,
            request.getShooterReleaseHeightMeters(),
            request.getRobotHalfLengthMeters(),
            request.getRobotHalfWidthMeters(),
            dynamicObstacles,
            constraints);

    return toResponse(solution);
  }

  public static Optional<ShotSolution> fromResponse(DragShotAutoResponseDTO response) {
    if (response == null || !response.isPresent() || response.getSolution() == null) {
      return Optional.empty();
    }

    ShotSolutionDTO dto = response.getSolution();
    ShotSolution solution =
        new ShotSolution(
            fromTranslationDto(dto.getShooterPosition()),
            Rotation2d.fromRadians(dto.getShooterYawRadians()),
            dto.getLaunchSpeedMetersPerSecond(),
            Rotation2d.fromRadians(dto.getLaunchAngleRadians()),
            dto.getTimeToPlaneSeconds(),
            fromTranslationDto(dto.getImpactFieldPosition()),
            dto.getVerticalErrorMeters());

    return Optional.of(solution);
  }

  public static DragShotAutoResponseDTO toResponse(Optional<ShotSolution> solutionOptional) {
    if (solutionOptional == null || solutionOptional.isEmpty()) {
      return DragShotAutoResponseDTO.empty();
    }

    ShotSolution solution = solutionOptional.get();
    ShotSolutionDTO dto = new ShotSolutionDTO();
    dto.setShooterPosition(toTranslationDto(solution.shooterPosition()));
    dto.setShooterYawRadians(solution.shooterYaw().getRadians());
    dto.setLaunchSpeedMetersPerSecond(solution.launchSpeedMetersPerSecond());
    dto.setLaunchAngleRadians(solution.launchAngle().getRadians());
    dto.setTimeToPlaneSeconds(solution.timeToPlaneSeconds());
    dto.setImpactFieldPosition(toTranslationDto(solution.impactFieldPosition()));
    dto.setVerticalErrorMeters(solution.verticalErrorMeters());

    DragShotAutoResponseDTO response = new DragShotAutoResponseDTO();
    response.setPresent(true);
    response.setSolution(dto);
    return response;
  }

  private static GamePiecePhysicsDTO toGamePieceDto(GamePiecePhysics gamePiece) {
    if (gamePiece == null) {
      return new GamePiecePhysicsDTO();
    }
    return new GamePiecePhysicsDTO(
        gamePiece.massKg(),
        gamePiece.crossSectionAreaM2(),
        gamePiece.dragCoefficient(),
        gamePiece.airDensityKgPerM3());
  }

  private static GamePiecePhysics fromGamePieceDto(GamePiecePhysicsDTO dto) {
    if (dto == null) {
      return new SnapshotGamePiecePhysics(0, 0, 0, 1.225);
    }
    return new SnapshotGamePiecePhysics(
        dto.getMassKg(),
        dto.getCrossSectionAreaM2(),
        dto.getDragCoefficient(),
        dto.getAirDensityKgPerM3());
  }

  private static ConstraintsDTO toConstraintsDto(Constraints constraints) {
    if (constraints == null) {
      return new ConstraintsDTO();
    }
    return new ConstraintsDTO(
        constraints.minLaunchSpeedMetersPerSecond(),
        constraints.maxLaunchSpeedMetersPerSecond(),
        constraints.minLaunchAngleDeg(),
        constraints.maxLaunchAngleDeg(),
        constraints.shotStyle().name());
  }

  private static Constraints fromConstraintsDto(ConstraintsDTO dto) {
    if (dto == null) {
      return new Constraints(0, 0, 0, 0);
    }

    Constraints.ShotStyle style;
    try {
      style = Constraints.ShotStyle.valueOf(dto.getShotStyle());
    } catch (Exception ignored) {
      style = Constraints.ShotStyle.ANY;
    }

    return new Constraints(
        dto.getMinLaunchSpeedMetersPerSecond(),
        dto.getMaxLaunchSpeedMetersPerSecond(),
        dto.getMinLaunchAngleDeg(),
        dto.getMaxLaunchAngleDeg(),
        style);
  }

  private static Pose2dDTO toPoseDto(Pose2d pose) {
    if (pose == null) {
      return new Pose2dDTO();
    }
    return new Pose2dDTO(
        pose.getTranslation().getX(),
        pose.getTranslation().getY(),
        pose.getRotation().getRadians());
  }

  private static Pose2d fromPoseDto(Pose2dDTO dto) {
    if (dto == null) {
      return new Pose2d();
    }
    return new Pose2d(dto.getX(), dto.getY(), Rotation2d.fromRadians(dto.getThetaRadians()));
  }

  private static Translation2dDTO toTranslationDto(Translation2d translation) {
    if (translation == null) {
      return new Translation2dDTO();
    }
    return new Translation2dDTO(translation.getX(), translation.getY());
  }

  private static Translation2d fromTranslationDto(Translation2dDTO dto) {
    if (dto == null) {
      return new Translation2d();
    }
    return new Translation2d(dto.getX(), dto.getY());
  }

  private static ObstacleDTO toObstacleDto(Obstacle obstacle) {
    if (obstacle == null) {
      return null;
    }

    ObstacleDTO dto = new ObstacleDTO();
    dto.setStrength(obstacle.strength);
    dto.setPositive(obstacle.positive);

    if (obstacle instanceof VisionPlanner.VisionObstacle vision) {
      dto.setKind("ELLIPSE");
      dto.setX(vision.loc.getX());
      dto.setY(vision.loc.getY());
      dto.setSizeX(vision.sizeX);
      dto.setSizeY(vision.sizeY);
      return dto;
    }

    if (obstacle instanceof PointObstacle point) {
      dto.setKind("CIRCLE");
      dto.setX(point.loc.getX());
      dto.setY(point.loc.getY());
      dto.setRadius(point.radius);
      return dto;
    }

    if (obstacle instanceof SnowmanObstacle snowman) {
      dto.setKind("CIRCLE");
      dto.setX(snowman.loc.getX());
      dto.setY(snowman.loc.getY());
      dto.setRadius(snowman.radius);
      return dto;
    }

    if (obstacle instanceof HorizontalObstacle horizontal) {
      dto.setKind("LINE_H");
      dto.setY(horizontal.y);
      return dto;
    }

    if (obstacle instanceof VerticalObstacle vertical) {
      dto.setKind("LINE_V");
      dto.setX(vertical.x);
      return dto;
    }

    if (obstacle instanceof SquareObstacle square) {
      dto.setKind("BOX");
      dto.setX(square.center.getX());
      dto.setY(square.center.getY());
      dto.setSizeX(square.halfSize * 2.0);
      dto.setSizeY(square.halfSize * 2.0);
      return dto;
    }

    return null;
  }

  private static List<Obstacle> fromObstacleDtos(List<ObstacleDTO> obstacleDtos) {
    List<Obstacle> output = new ArrayList<>();
    if (obstacleDtos == null) {
      return output;
    }

    for (ObstacleDTO dto : obstacleDtos) {
      output.add(new SerializedObstacle(dto));
    }
    return output;
  }

  private static final class SnapshotGamePiecePhysics extends GamePiecePhysics {
    private final double massKg;
    private final double crossSectionAreaM2;
    private final double dragCoefficient;
    private final double airDensityKgPerM3;

    SnapshotGamePiecePhysics(
        double massKg,
        double crossSectionAreaM2,
        double dragCoefficient,
        double airDensityKgPerM3) {
      this.massKg = massKg;
      this.crossSectionAreaM2 = crossSectionAreaM2;
      this.dragCoefficient = dragCoefficient;
      this.airDensityKgPerM3 = airDensityKgPerM3;
    }

    @Override
    public double massKg() {
      return massKg;
    }

    @Override
    public double crossSectionAreaM2() {
      return crossSectionAreaM2;
    }

    @Override
    public double dragCoefficient() {
      return dragCoefficient;
    }

    @Override
    public double airDensityKgPerM3() {
      return airDensityKgPerM3;
    }
  }

  private static final class SerializedObstacle extends Obstacle {
    private final String kind;
    private final double x;
    private final double y;
    private final double sizeX;
    private final double sizeY;
    private final double radius;

    SerializedObstacle(ObstacleDTO dto) {
      super(dto == null ? 0.0 : dto.getStrength(), dto == null || dto.isPositive());
      kind = dto == null ? "UNKNOWN" : dto.getKind();
      x = dto == null ? 0.0 : dto.getX();
      y = dto == null ? 0.0 : dto.getY();
      sizeX = dto == null ? 0.0 : dto.getSizeX();
      sizeY = dto == null ? 0.0 : dto.getSizeY();
      radius = dto == null ? 0.0 : dto.getRadius();
    }

    @Override
    public Force getForceAtPosition(Translation2d position, Translation2d target) {
      return new Force();
    }

    @Override
    public boolean intersectsRectangle(Translation2d[] rectCorners) {
      if ("LINE_H".equals(kind)) {
        for (Translation2d corner : rectCorners) {
          if (Math.abs(corner.getY() - y) < 0.1) {
            return true;
          }
        }
        return false;
      }

      if ("LINE_V".equals(kind)) {
        for (Translation2d corner : rectCorners) {
          if (Math.abs(corner.getX() - x) < 0.1) {
            return true;
          }
        }
        return false;
      }

      Translation2d center = new Translation2d(x, y);

      if ("CIRCLE".equals(kind)) {
        if (FieldPlanner.isPointInPolygon(center, rectCorners)) {
          return true;
        }

        for (Translation2d corner : rectCorners) {
          if (corner.getDistance(center) < radius) {
            return true;
          }
        }

        for (int i = 0; i < rectCorners.length; i++) {
          Translation2d a = rectCorners[i];
          Translation2d b = rectCorners[(i + 1) % rectCorners.length];
          if (FieldPlanner.distanceFromPointToSegment(center, a, b) < radius) {
            return true;
          }
        }
        return false;
      }

      if ("ELLIPSE".equals(kind)) {
        if (FieldPlanner.isPointInPolygon(center, rectCorners)) {
          return true;
        }

        double rx = sizeX / 2.0;
        double ry = sizeY / 2.0;
        for (Translation2d corner : rectCorners) {
          double dx = corner.getX() - center.getX();
          double dy = corner.getY() - center.getY();
          if ((dx * dx) / (rx * rx) + (dy * dy) / (ry * ry) <= 1.0) {
            return true;
          }
        }

        double boundingRadius = Math.max(rx, ry);
        for (int i = 0; i < rectCorners.length; i++) {
          Translation2d a = rectCorners[i];
          Translation2d b = rectCorners[(i + 1) % rectCorners.length];
          if (FieldPlanner.distanceFromPointToSegment(center, a, b) < boundingRadius) {
            return true;
          }
        }
        return false;
      }

      if ("BOX".equals(kind)) {
        double halfX = sizeX * 0.5;
        double halfY = sizeY * 0.5;
        double minX = center.getX() - halfX;
        double maxX = center.getX() + halfX;
        double minY = center.getY() - halfY;
        double maxY = center.getY() + halfY;

        for (Translation2d corner : rectCorners) {
          if (corner.getX() >= minX
              && corner.getX() <= maxX
              && corner.getY() >= minY
              && corner.getY() <= maxY) {
            return true;
          }
        }
      }

      return false;
    }
  }
}
