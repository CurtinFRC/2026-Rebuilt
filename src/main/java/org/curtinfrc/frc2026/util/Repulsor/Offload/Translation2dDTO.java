package org.curtinfrc.frc2026.util.Repulsor.Offload;

public class Translation2dDTO {
  private double x;
  private double y;

  public Translation2dDTO() {}

  public Translation2dDTO(double x, double y) {
    this.x = x;
    this.y = y;
  }

  public double getX() {
    return x;
  }

  public double getY() {
    return y;
  }

  public void setX(double x) {
    this.x = x;
  }

  public void setY(double y) {
    this.y = y;
  }
}
