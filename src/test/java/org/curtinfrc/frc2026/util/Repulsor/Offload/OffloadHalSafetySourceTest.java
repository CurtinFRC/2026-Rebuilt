package org.curtinfrc.frc2026.util.Repulsor.Offload;

import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;

class OffloadHalSafetySourceTest {
  private static final List<String> BANNED_TOKENS =
      List.of(
          "edu.wpi.first.wpilibj",
          "DriverStation",
          "RobotBase",
          "Timer.getFPGATimestamp",
          "org.curtinfrc.frc2026.Constants");

  private static final List<String> OFFLOAD_SAFE_SOURCES =
      List.of(
          "src/main/java/org/curtinfrc/frc2026/util/Repulsor/Predictive/PredictiveFieldStateLocalAccess.java",
          "src/main/java/org/curtinfrc/frc2026/util/Repulsor/Predictive/PredictiveFieldStateRuntime.java",
          "src/main/java/org/curtinfrc/frc2026/util/Repulsor/Predictive/PredictiveFieldStateOps.java",
          "src/main/java/org/curtinfrc/frc2026/util/Repulsor/Predictive/PredictiveCollectPenaltyTracker.java",
          "src/main/java/org/curtinfrc/frc2026/util/Repulsor/Predictive/Runtime/PredictiveCollectNearestResolutionStep.java",
          "src/main/java/org/curtinfrc/frc2026/util/Repulsor/Predictive/Runtime/PredictiveCollectNearestSearchStep.java",
          "src/main/java/org/curtinfrc/frc2026/util/Repulsor/Predictive/Runtime/PredictiveCollectScoringRuntime.java",
          "src/main/java/org/curtinfrc/frc2026/util/Repulsor/Predictive/Runtime/PredictiveFieldStateTrackingRuntime.java",
          "src/main/java/org/curtinfrc/frc2026/util/Repulsor/Tracking/FieldTrackerLocalAccess.java");

  @Test
  void offloadedPredictiveAndTrackerSourcesShouldStayHalFree() throws IOException {
    List<String> violations = new ArrayList<>();
    Path repoRoot = Path.of("").toAbsolutePath();

    for (String relPath : OFFLOAD_SAFE_SOURCES) {
      Path file = repoRoot.resolve(relPath);
      String source = Files.readString(file, StandardCharsets.UTF_8);
      for (String token : BANNED_TOKENS) {
        if (source.contains(token)) {
          violations.add(relPath + " contains banned token: " + token);
        }
      }
    }

    assertTrue(
        violations.isEmpty(),
        "HAL-sensitive APIs leaked into offload-safe sources:\n" + String.join("\n", violations));
  }
}
