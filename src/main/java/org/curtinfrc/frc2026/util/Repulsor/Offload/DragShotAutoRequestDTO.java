package org.curtinfrc.frc2026.util.Repulsor.Offload;

import java.util.ArrayList;
import java.util.List;

public class DragShotAutoRequestDTO {
  private GamePiecePhysicsDTO gamePiece;
  private Translation2dDTO targetFieldPosition;
  private double targetHeightMeters;
  private Pose2dDTO robotPose;
  private double shooterReleaseHeightMeters;
  private double robotHalfLengthMeters;
  private double robotHalfWidthMeters;
  private List<ObstacleDTO> dynamicObstacles = new ArrayList<>();
  private ConstraintsDTO constraints;

  public DragShotAutoRequestDTO() {}

  public GamePiecePhysicsDTO getGamePiece() {
    return gamePiece;
  }

  public void setGamePiece(GamePiecePhysicsDTO gamePiece) {
    this.gamePiece = gamePiece;
  }

  public Translation2dDTO getTargetFieldPosition() {
    return targetFieldPosition;
  }

  public void setTargetFieldPosition(Translation2dDTO targetFieldPosition) {
    this.targetFieldPosition = targetFieldPosition;
  }

  public double getTargetHeightMeters() {
    return targetHeightMeters;
  }

  public void setTargetHeightMeters(double targetHeightMeters) {
    this.targetHeightMeters = targetHeightMeters;
  }

  public Pose2dDTO getRobotPose() {
    return robotPose;
  }

  public void setRobotPose(Pose2dDTO robotPose) {
    this.robotPose = robotPose;
  }

  public double getShooterReleaseHeightMeters() {
    return shooterReleaseHeightMeters;
  }

  public void setShooterReleaseHeightMeters(double shooterReleaseHeightMeters) {
    this.shooterReleaseHeightMeters = shooterReleaseHeightMeters;
  }

  public double getRobotHalfLengthMeters() {
    return robotHalfLengthMeters;
  }

  public void setRobotHalfLengthMeters(double robotHalfLengthMeters) {
    this.robotHalfLengthMeters = robotHalfLengthMeters;
  }

  public double getRobotHalfWidthMeters() {
    return robotHalfWidthMeters;
  }

  public void setRobotHalfWidthMeters(double robotHalfWidthMeters) {
    this.robotHalfWidthMeters = robotHalfWidthMeters;
  }

  public List<ObstacleDTO> getDynamicObstacles() {
    return dynamicObstacles;
  }

  public void setDynamicObstacles(List<ObstacleDTO> dynamicObstacles) {
    this.dynamicObstacles = dynamicObstacles;
  }

  public ConstraintsDTO getConstraints() {
    return constraints;
  }

  public void setConstraints(ConstraintsDTO constraints) {
    this.constraints = constraints;
  }
}
