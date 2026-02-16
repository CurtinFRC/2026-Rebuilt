package org.curtinfrc.frc2026.util.Repulsor.Offload;

public class GamePiecePhysicsDTO {
  private double massKg;
  private double crossSectionAreaM2;
  private double dragCoefficient;
  private double airDensityKgPerM3;

  public GamePiecePhysicsDTO() {}

  public GamePiecePhysicsDTO(
      double massKg, double crossSectionAreaM2, double dragCoefficient, double airDensityKgPerM3) {
    this.massKg = massKg;
    this.crossSectionAreaM2 = crossSectionAreaM2;
    this.dragCoefficient = dragCoefficient;
    this.airDensityKgPerM3 = airDensityKgPerM3;
  }

  public double getMassKg() {
    return massKg;
  }

  public double getCrossSectionAreaM2() {
    return crossSectionAreaM2;
  }

  public double getDragCoefficient() {
    return dragCoefficient;
  }

  public double getAirDensityKgPerM3() {
    return airDensityKgPerM3;
  }

  public void setMassKg(double massKg) {
    this.massKg = massKg;
  }

  public void setCrossSectionAreaM2(double crossSectionAreaM2) {
    this.crossSectionAreaM2 = crossSectionAreaM2;
  }

  public void setDragCoefficient(double dragCoefficient) {
    this.dragCoefficient = dragCoefficient;
  }

  public void setAirDensityKgPerM3(double airDensityKgPerM3) {
    this.airDensityKgPerM3 = airDensityKgPerM3;
  }
}
