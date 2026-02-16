package org.curtinfrc.frc2026.util.Repulsor.Offload;

public class Pose2dDTO {
  private double x;
  private double y;
  private double thetaRadians;

  public Pose2dDTO() {}

  public Pose2dDTO(double x, double y, double thetaRadians) {
    this.x = x;
    this.y = y;
    this.thetaRadians = thetaRadians;
  }

  public double getX() {
    return x;
  }

  public double getY() {
    return y;
  }

  public double getThetaRadians() {
    return thetaRadians;
  }

  public void setX(double x) {
    this.x = x;
  }

  public void setY(double y) {
    this.y = y;
  }

  public void setThetaRadians(double thetaRadians) {
    this.thetaRadians = thetaRadians;
  }
}
