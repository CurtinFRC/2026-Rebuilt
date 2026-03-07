package org.curtinfrc.frc2026.util.Repulsor.Offload;

public final class FieldPlannerCalculateResultDTO {
  private double goalX;
  private double goalY;
  private double vxMetersPerSecond;
  private double vyMetersPerSecond;
  private double omegaRadians;
  private boolean hasErrMeters;
  private double errMeters;
  private double activeGoalX;
  private double activeGoalY;
  private double activeGoalThetaRadians;

  public double getGoalX() {
    return goalX;
  }

  public void setGoalX(double goalX) {
    this.goalX = goalX;
  }

  public double getGoalY() {
    return goalY;
  }

  public void setGoalY(double goalY) {
    this.goalY = goalY;
  }

  public double getVxMetersPerSecond() {
    return vxMetersPerSecond;
  }

  public void setVxMetersPerSecond(double vxMetersPerSecond) {
    this.vxMetersPerSecond = vxMetersPerSecond;
  }

  public double getVyMetersPerSecond() {
    return vyMetersPerSecond;
  }

  public void setVyMetersPerSecond(double vyMetersPerSecond) {
    this.vyMetersPerSecond = vyMetersPerSecond;
  }

  public double getOmegaRadians() {
    return omegaRadians;
  }

  public void setOmegaRadians(double omegaRadians) {
    this.omegaRadians = omegaRadians;
  }

  public boolean isHasErrMeters() {
    return hasErrMeters;
  }

  public void setHasErrMeters(boolean hasErrMeters) {
    this.hasErrMeters = hasErrMeters;
  }

  public double getErrMeters() {
    return errMeters;
  }

  public void setErrMeters(double errMeters) {
    this.errMeters = errMeters;
  }

  public double getActiveGoalX() {
    return activeGoalX;
  }

  public void setActiveGoalX(double activeGoalX) {
    this.activeGoalX = activeGoalX;
  }

  public double getActiveGoalY() {
    return activeGoalY;
  }

  public void setActiveGoalY(double activeGoalY) {
    this.activeGoalY = activeGoalY;
  }

  public double getActiveGoalThetaRadians() {
    return activeGoalThetaRadians;
  }

  public void setActiveGoalThetaRadians(double activeGoalThetaRadians) {
    this.activeGoalThetaRadians = activeGoalThetaRadians;
  }
}
