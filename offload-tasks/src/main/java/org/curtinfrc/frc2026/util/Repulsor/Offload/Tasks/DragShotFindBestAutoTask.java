package org.curtinfrc.frc2026.util.Repulsor.Offload.Tasks;

import org.curtinfrc.frc2026.util.Repulsor.Offload.DragShotAutoRequestDTO;
import org.curtinfrc.frc2026.util.Repulsor.Offload.DragShotAutoResponseDTO;
import org.curtinfrc.frc2026.util.Repulsor.Offload.DragShotPlannerOffloadEntrypoints;
import org.curtinfrc.frc2026.util.Repulsor.Offload.OffloadFunction;
import org.curtinfrc.frc2026.util.Repulsor.Offload.OffloadTaskIds;

public class DragShotFindBestAutoTask
    implements OffloadFunction<DragShotAutoRequestDTO, DragShotAutoResponseDTO> {

  @Override
  public String taskId() {
    return OffloadTaskIds.DRAG_SHOT_FIND_BEST_SHOT_AUTO;
  }

  @Override
  public Class<DragShotAutoRequestDTO> requestType() {
    return DragShotAutoRequestDTO.class;
  }

  @Override
  public Class<DragShotAutoResponseDTO> responseType() {
    return DragShotAutoResponseDTO.class;
  }

  @Override
  public int timeoutMs() {
    return 20;
  }

  @Override
  public DragShotAutoResponseDTO execute(DragShotAutoRequestDTO request) {
    return DragShotPlannerOffloadEntrypoints.findBestShotAuto(request);
  }
}
