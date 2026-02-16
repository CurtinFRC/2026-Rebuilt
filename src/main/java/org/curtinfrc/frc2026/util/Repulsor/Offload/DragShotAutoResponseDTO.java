package org.curtinfrc.frc2026.util.Repulsor.Offload;

public class DragShotAutoResponseDTO {
  private boolean present;
  private ShotSolutionDTO solution;

  public DragShotAutoResponseDTO() {}

  public static DragShotAutoResponseDTO empty() {
    DragShotAutoResponseDTO response = new DragShotAutoResponseDTO();
    response.setPresent(false);
    return response;
  }

  public boolean isPresent() {
    return present;
  }

  public void setPresent(boolean present) {
    this.present = present;
  }

  public ShotSolutionDTO getSolution() {
    return solution;
  }

  public void setSolution(ShotSolutionDTO solution) {
    this.solution = solution;
  }
}
