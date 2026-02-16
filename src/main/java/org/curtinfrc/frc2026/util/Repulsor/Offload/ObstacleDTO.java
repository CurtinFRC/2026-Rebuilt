package org.curtinfrc.frc2026.util.Repulsor.Offload;

public class ObstacleDTO {
  private String kind;
  private double strength;
  private boolean positive;
  private double x;
  private double y;
  private double sizeX;
  private double sizeY;
  private double radius;

  public ObstacleDTO() {}

  public String getKind() {
    return kind;
  }

  public void setKind(String kind) {
    this.kind = kind;
  }

  public double getStrength() {
    return strength;
  }

  public void setStrength(double strength) {
    this.strength = strength;
  }

  public boolean isPositive() {
    return positive;
  }

  public void setPositive(boolean positive) {
    this.positive = positive;
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

  public double getSizeX() {
    return sizeX;
  }

  public void setSizeX(double sizeX) {
    this.sizeX = sizeX;
  }

  public double getSizeY() {
    return sizeY;
  }

  public void setSizeY(double sizeY) {
    this.sizeY = sizeY;
  }

  public double getRadius() {
    return radius;
  }

  public void setRadius(double radius) {
    this.radius = radius;
  }
}
