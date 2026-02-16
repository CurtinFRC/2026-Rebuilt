package org.curtinfrc.frc2026.util.Repulsor.Offload;

public class ConstraintsDTO {
  private double minLaunchSpeedMetersPerSecond;
  private double maxLaunchSpeedMetersPerSecond;
  private double minLaunchAngleDeg;
  private double maxLaunchAngleDeg;
  private String shotStyle;

  public ConstraintsDTO() {}

  public ConstraintsDTO(
      double minLaunchSpeedMetersPerSecond,
      double maxLaunchSpeedMetersPerSecond,
      double minLaunchAngleDeg,
      double maxLaunchAngleDeg,
      String shotStyle) {
    this.minLaunchSpeedMetersPerSecond = minLaunchSpeedMetersPerSecond;
    this.maxLaunchSpeedMetersPerSecond = maxLaunchSpeedMetersPerSecond;
    this.minLaunchAngleDeg = minLaunchAngleDeg;
    this.maxLaunchAngleDeg = maxLaunchAngleDeg;
    this.shotStyle = shotStyle;
  }

  public double getMinLaunchSpeedMetersPerSecond() {
    return minLaunchSpeedMetersPerSecond;
  }

  public double getMaxLaunchSpeedMetersPerSecond() {
    return maxLaunchSpeedMetersPerSecond;
  }

  public double getMinLaunchAngleDeg() {
    return minLaunchAngleDeg;
  }

  public double getMaxLaunchAngleDeg() {
    return maxLaunchAngleDeg;
  }

  public String getShotStyle() {
    return shotStyle;
  }

  public void setMinLaunchSpeedMetersPerSecond(double minLaunchSpeedMetersPerSecond) {
    this.minLaunchSpeedMetersPerSecond = minLaunchSpeedMetersPerSecond;
  }

  public void setMaxLaunchSpeedMetersPerSecond(double maxLaunchSpeedMetersPerSecond) {
    this.maxLaunchSpeedMetersPerSecond = maxLaunchSpeedMetersPerSecond;
  }

  public void setMinLaunchAngleDeg(double minLaunchAngleDeg) {
    this.minLaunchAngleDeg = minLaunchAngleDeg;
  }

  public void setMaxLaunchAngleDeg(double maxLaunchAngleDeg) {
    this.maxLaunchAngleDeg = maxLaunchAngleDeg;
  }

  public void setShotStyle(String shotStyle) {
    this.shotStyle = shotStyle;
  }
}
