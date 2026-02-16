package org.curtinfrc.frc2026.util.Repulsor.Offload;

import java.util.concurrent.CompletableFuture;

public interface OffloadGateway {
  CompletableFuture<byte[]> call(String taskId, byte[] payloadBytes, int timeoutMs);

  default boolean isHealthy() {
    return false;
  }
}
