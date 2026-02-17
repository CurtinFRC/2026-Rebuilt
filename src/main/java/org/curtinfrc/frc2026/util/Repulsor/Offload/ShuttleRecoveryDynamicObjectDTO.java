package org.curtinfrc.frc2026.util.Repulsor.Offload;

public class ShuttleRecoveryDynamicObjectDTO {
  private String id = "";
  private String type = "unknown";
  private double x;
  private double y;
  private double vx;
  private double vy;
  private double ageS;

  public ShuttleRecoveryDynamicObjectDTO() {}

  public String getId() {
    return id;
  }

  public void setId(String id) {
    this.id = id == null ? "" : id;
  }

  public String getType() {
    return type;
  }

  public void setType(String type) {
    this.type = type == null ? "unknown" : type;
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

  public double getVx() {
    return vx;
  }

  public void setVx(double vx) {
    this.vx = vx;
  }

  public double getVy() {
    return vy;
  }

  public void setVy(double vy) {
    this.vy = vy;
  }

  public double getAgeS() {
    return ageS;
  }

  public void setAgeS(double ageS) {
    this.ageS = ageS;
  }
}
