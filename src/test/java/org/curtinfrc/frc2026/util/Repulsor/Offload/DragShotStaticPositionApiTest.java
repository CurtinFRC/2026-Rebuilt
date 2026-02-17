package org.curtinfrc.frc2026.util.Repulsor.Offload;

import static org.junit.jupiter.api.Assertions.assertNotNull;

import edu.wpi.first.math.geometry.Translation2d;
import org.curtinfrc.frc2026.util.Repulsor.Shooting.Constraints;
import org.curtinfrc.frc2026.util.Repulsor.Shooting.DragShotPlanner;
import org.curtinfrc.frc2026.util.Repulsor.Shooting.GamePiecePhysics;
import org.junit.jupiter.api.Test;

class DragShotStaticPositionApiTest {
  @Test
  void shouldExposeStaticPositionSolveApi() throws Exception {
    assertNotNull(
        DragShotPlanner.class.getMethod(
            "calculateStaticShotAngleAndSpeed",
            GamePiecePhysics.class,
            Translation2d.class,
            Translation2d.class,
            double.class,
            double.class,
            Constraints.class));

    assertNotNull(
        DragShotPlanner.class.getMethod(
            "calculateStaticShotAngleAndSpeedAsync",
            GamePiecePhysics.class,
            Translation2d.class,
            Translation2d.class,
            double.class,
            double.class,
            Constraints.class));

    assertNotNull(
        DragShotPlanner.class.getMethod(
            "calculateStaticShotAngleAndSpeedLocal",
            GamePiecePhysics.class,
            Translation2d.class,
            Translation2d.class,
            double.class,
            double.class,
            Constraints.class));
  }

  @Test
  void shouldGenerateOffloadedWrapperMethodsForStaticPositionSolve() throws Exception {
    Class<?> wrapper =
        Class.forName(
            "org.curtinfrc.frc2026.util.Repulsor.Offload.DragShotPlannerOffloadEntrypoints_Offloaded");
    Class<?> requestType =
        Class.forName(
            "org.curtinfrc.frc2026.util.Repulsor.Offload.DragShotPlannerOffloadEntrypoints_calculateStaticShotAngleAndSpeed_OffloadRequest");

    assertNotNull(
        wrapper.getMethod(
            "calculateStaticShotAngleAndSpeed_offload",
            GamePiecePhysics.class,
            Translation2d.class,
            Translation2d.class,
            double.class,
            double.class,
            Constraints.class));

    assertNotNull(
        wrapper.getMethod(
            "calculateStaticShotAngleAndSpeed_offloadAsync",
            GamePiecePhysics.class,
            Translation2d.class,
            Translation2d.class,
            double.class,
            double.class,
            Constraints.class));

    assertNotNull(
        wrapper.getMethod("calculateStaticShotAngleAndSpeed_offloadExecute", requestType));

    Class<?> executeReturnType =
        wrapper
            .getMethod("calculateStaticShotAngleAndSpeed_offloadExecute", requestType)
            .getReturnType();
    assertNotNull(
        executeReturnType.getMethod("getResult"),
        "Generated response DTO should expose getResult()");
  }

  @Test
  void shouldDefineStaticPositionTaskId() {
    assertNotNull(OffloadTaskIds.DRAG_SHOT_CALC_STATIC_SHOT_ANGLE_SPEED);
  }
}
