package org.curtinfrc.frc2026.util.Repulsor.Offload;

import java.util.List;
import org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacle;

public final class RepulsorOffloadAdapterRegistrar implements OffloadAdapterRegistrar {
  @Override
  public void registerAdapters() {
    OffloadValueCodec.registerAdapter(
        "edu.wpi.first.math.geometry.Translation2d",
        Translation2dDTO.class,
        DragShotOffloadMapper::toTranslationDto,
        DragShotOffloadMapper::fromTranslationDto);

    OffloadValueCodec.registerAdapter(
        "edu.wpi.first.math.geometry.Pose2d",
        Pose2dDTO.class,
        DragShotOffloadMapper::toPoseDto,
        DragShotOffloadMapper::fromPoseDto);

    OffloadValueCodec.registerAdapter(
        "org.curtinfrc.frc2026.util.Repulsor.Shooting.GamePiecePhysics",
        GamePiecePhysicsDTO.class,
        DragShotOffloadMapper::toGamePieceDto,
        DragShotOffloadMapper::fromGamePieceDto);

    OffloadValueCodec.registerAdapter(
        "org.curtinfrc.frc2026.util.Repulsor.Shooting.Constraints",
        ConstraintsDTO.class,
        DragShotOffloadMapper::toConstraintsDto,
        DragShotOffloadMapper::fromConstraintsDto);

    OffloadValueCodec.registerAdapter(
        "org.curtinfrc.frc2026.util.Repulsor.Shooting.ShotSolution",
        ShotSolutionDTO.class,
        DragShotOffloadMapper::toShotSolutionDto,
        DragShotOffloadMapper::fromShotSolutionDto);

    OffloadValueCodec.registerAdapter(
        "java.util.Optional<org.curtinfrc.frc2026.util.Repulsor.Shooting.ShotSolution>",
        DragShotAutoResponseDTO.class,
        DragShotOffloadMapper::toOptionalShotSolutionDto,
        DragShotOffloadMapper::fromOptionalShotSolutionDto);

    OffloadValueCodec.registerAdapter(
        "java.util.List<? extends org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacle>",
        ObstacleListDTO.class,
        this::obstaclesToWire,
        this::obstaclesFromWire);
  }

  private ObstacleListDTO obstaclesToWire(List<? extends Obstacle> obstacles) {
    ObstacleListDTO wire = new ObstacleListDTO();
    wire.setObstacles(DragShotOffloadMapper.toObstacleDtos(obstacles));
    return wire;
  }

  private List<? extends Obstacle> obstaclesFromWire(ObstacleListDTO wire) {
    if (wire == null) {
      return List.of();
    }
    return DragShotOffloadMapper.fromObstacleDtos(wire.getObstacles());
  }

  public static class ObstacleListDTO {
    private List<ObstacleDTO> obstacles = List.of();

    public List<ObstacleDTO> getObstacles() {
      return obstacles;
    }

    public void setObstacles(List<ObstacleDTO> obstacles) {
      this.obstacles = obstacles;
    }
  }
}
