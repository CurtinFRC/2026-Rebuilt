package org.curtinfrc.frc2026.util.Repulsor.Offload;

import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.stream.Stream;
import org.junit.jupiter.api.Test;

class FieldPlannerOffloadHalSafetySourceTest {
  private static final List<String> DISALLOWED_HAL_TOKENS =
      List.of(
          "Timer.getFPGATimestamp",
          "edu.wpi.first.wpilibj.Timer",
          "RobotController.getFPGATime",
          "edu.wpi.first.wpilibj.Filesystem",
          "Filesystem.getDeployDirectory",
          "edu.wpi.first.hal.JNIWrapper",
          "RuntimeLoader.loadLibrary");

  private static final String FIELD_PLANNER_BASE =
      "src/main/java/org/curtinfrc/frc2026/util/Repulsor/FieldPlanner";

  private static final List<String> OFFLOAD_CRITICAL_FILES =
      List.of(
          FIELD_PLANNER_BASE + "/FieldPlanner.java",
          FIELD_PLANNER_BASE + "/Helpers/FieldPlannerForceModel.java",
          "src/main/java/org/curtinfrc/frc2026/util/Repulsor/Offload/FieldPlannerOffloadLocalAccess.java",
          "src/main/java/org/curtinfrc/frc2026/util/Repulsor/Offload/FieldPlannerOffloadEntrypoints.java",
          "src/main/java/org/curtinfrc/frc2026/util/Repulsor/Offload/FieldPlannerPathingOffloadEntrypoints.java",
          "src/main/java/org/curtinfrc/frc2026/util/Repulsor/ExtraPathing.java");

  @Test
  void fieldPlannerOffloadCriticalSourcesShouldStayHalSafe() throws IOException {
    Path repoRoot = Path.of("").toAbsolutePath().normalize();
    List<Path> filesToCheck = new ArrayList<>();
    for (String relPath : OFFLOAD_CRITICAL_FILES) {
      filesToCheck.add(repoRoot.resolve(relPath));
    }
    Path obstaclesDir = repoRoot.resolve(FIELD_PLANNER_BASE).resolve("Obstacles");
    try (Stream<Path> walk = Files.walk(obstaclesDir)) {
      walk.filter(Files::isRegularFile)
          .filter(path -> path.toString().endsWith(".java"))
          .sorted(Comparator.comparing(Path::toString))
          .forEach(filesToCheck::add);
    }

    List<String> violations = new ArrayList<>();
    for (Path file : filesToCheck) {
      String source = Files.readString(file, StandardCharsets.UTF_8);
      String rel = repoRoot.relativize(file).toString().replace('\\', '/');
      for (String token : DISALLOWED_HAL_TOKENS) {
        if (source.contains(token)) {
          violations.add(rel + " contains disallowed HAL token: " + token);
        }
      }

      if (rel.startsWith(FIELD_PLANNER_BASE + "/Obstacles/")
          && (source.contains("edu.wpi.first.wpilibj")
              || source.contains("edu.wpi.first.hal")
              || source.contains("RobotBase")
              || source.contains("DriverStation"))) {
        violations.add(rel + " contains WPILib/HAL dependency in obstacle code");
      }
    }

    assertTrue(
        violations.isEmpty(),
        "HAL-sensitive APIs leaked into FieldPlanner offload-critical sources:\n"
            + String.join("\n", violations));
  }
}
