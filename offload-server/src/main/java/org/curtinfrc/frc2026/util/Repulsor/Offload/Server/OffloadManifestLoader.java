package org.curtinfrc.frc2026.util.Repulsor.Offload.Server;

import com.fasterxml.jackson.databind.ObjectMapper;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import org.curtinfrc.frc2026.util.Repulsor.Offload.OffloadHash;
import org.curtinfrc.frc2026.util.Repulsor.Offload.OffloadManifest;

final class OffloadManifestLoader {
  private OffloadManifestLoader() {}

  static ManifestInfo load(Path pluginDirectory) {
    Path manifestPath = pluginDirectory.resolve("offload-manifest.json");
    if (!Files.exists(manifestPath)) {
      return new ManifestInfo("", List.of());
    }

    try {
      byte[] bytes = Files.readAllBytes(manifestPath);
      OffloadManifest manifest = new ObjectMapper().readValue(bytes, OffloadManifest.class);
      List<String> taskIds = new ArrayList<>();
      if (manifest.getFunctions() != null) {
        for (OffloadManifest.FunctionEntry function : manifest.getFunctions()) {
          if (function.getId() != null && !function.getId().isBlank()) {
            taskIds.add(function.getId());
          }
        }
      }
      Collections.sort(taskIds);
      return new ManifestInfo(OffloadHash.sha256Hex(bytes), taskIds);
    } catch (IOException ex) {
      throw new IllegalStateException("Failed to read offload manifest from " + manifestPath, ex);
    }
  }

  record ManifestInfo(String hash, List<String> taskIds) {}
}
