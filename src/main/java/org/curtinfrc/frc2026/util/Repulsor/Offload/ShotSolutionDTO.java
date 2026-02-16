package org.curtinfrc.frc2026.util.Repulsor.Offload;

public class ShotSolutionDTO {
  private Translation2dDTO shooterPosition;
  private double shooterYawRadians;
  private double launchSpeedMetersPerSecond;
  private double launchAngleRadians;
  private double timeToPlaneSeconds;
  private Translation2dDTO impactFieldPosition;
  private double verticalErrorMeters;

  public ShotSolutionDTO() {}

  public Translation2dDTO getShooterPosition() {
    return shooterPosition;
  }

  public double getShooterYawRadians() {
    return shooterYawRadians;
  }

  public double getLaunchSpeedMetersPerSecond() {
    return launchSpeedMetersPerSecond;
  }

  public double getLaunchAngleRadians() {
    return launchAngleRadians;
  }

  public double getTimeToPlaneSeconds() {
    return timeToPlaneSeconds;
  }

  public Translation2dDTO getImpactFieldPosition() {
    return impactFieldPosition;
  }

  public double getVerticalErrorMeters() {
    return verticalErrorMeters;
  }

  public void setShooterPosition(Translation2dDTO shooterPosition) {
    this.shooterPosition = shooterPosition;
  }

  public void setShooterYawRadians(double shooterYawRadians) {
    this.shooterYawRadians = shooterYawRadians;
  }

  public void setLaunchSpeedMetersPerSecond(double launchSpeedMetersPerSecond) {
    this.launchSpeedMetersPerSecond = launchSpeedMetersPerSecond;
  }

  public void setLaunchAngleRadians(double launchAngleRadians) {
    this.launchAngleRadians = launchAngleRadians;
  }

  public void setTimeToPlaneSeconds(double timeToPlaneSeconds) {
    this.timeToPlaneSeconds = timeToPlaneSeconds;
  }

  public void setImpactFieldPosition(Translation2dDTO impactFieldPosition) {
    this.impactFieldPosition = impactFieldPosition;
  }

  public void setVerticalErrorMeters(double verticalErrorMeters) {
    this.verticalErrorMeters = verticalErrorMeters;
  }
}
