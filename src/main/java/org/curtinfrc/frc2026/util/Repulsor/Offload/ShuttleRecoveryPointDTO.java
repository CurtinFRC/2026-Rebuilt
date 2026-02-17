package org.curtinfrc.frc2026.util.Repulsor.Offload;

public class ShuttleRecoveryPointDTO {
  private boolean found;
  private double x;
  private double y;
  private double yawDeg;
  private double score;

  public ShuttleRecoveryPointDTO() {}

  public static ShuttleRecoveryPointDTO notFound() {
    return new ShuttleRecoveryPointDTO();
  }

  public static ShuttleRecoveryPointDTO of(double x, double y, double yawDeg, double score) {
    ShuttleRecoveryPointDTO dto = new ShuttleRecoveryPointDTO();
    dto.found = true;
    dto.x = x;
    dto.y = y;
    dto.yawDeg = yawDeg;
    dto.score = score;
    return dto;
  }

  public boolean isFound() {
    return found;
  }

  public void setFound(boolean found) {
    this.found = found;
  }

  public double getX() {
    return x;
  }

  public void setX(double x) {
    this.x = x;
  }

  public double getY() {
    return y;
  }

  public void setY(double y) {
    this.y = y;
  }

  public double getYawDeg() {
    return yawDeg;
  }

  public void setYawDeg(double yawDeg) {
    this.yawDeg = yawDeg;
  }

  public double getScore() {
    return score;
  }

  public void setScore(double score) {
    this.score = score;
  }
}
