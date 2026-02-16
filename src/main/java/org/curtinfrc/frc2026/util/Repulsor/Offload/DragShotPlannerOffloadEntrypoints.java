package org.curtinfrc.frc2026.util.Repulsor.Offload;

@SuppressWarnings("unused")
public final class DragShotPlannerOffloadEntrypoints {
  private DragShotPlannerOffloadEntrypoints() {}

  @Offloadable(
      id = OffloadTaskIds.DRAG_SHOT_FIND_BEST_SHOT_AUTO,
      version = 1,
      timeoutMs = 20,
      fallback = true)
  public static DragShotAutoResponseDTO findBestShotAuto(DragShotAutoRequestDTO request) {
    return DragShotOffloadMapper.executeLocal(request);
  }
}
