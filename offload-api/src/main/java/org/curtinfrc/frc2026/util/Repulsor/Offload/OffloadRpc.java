package org.curtinfrc.frc2026.util.Repulsor.Offload;

import java.util.Objects;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.CompletionException;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;

public final class OffloadRpc {
  private static final OffloadGateway NOOP_GATEWAY =
      (taskId, payloadBytes, timeoutMs) ->
          CompletableFuture.failedFuture(
              new IllegalStateException("No offload gateway configured"));

  private static volatile OffloadGateway gateway = NOOP_GATEWAY;

  private static final Executor FALLBACK_EXECUTOR =
      Executors.newFixedThreadPool(Math.max(1, Runtime.getRuntime().availableProcessors() / 2));

  private OffloadRpc() {}

  public static void setGateway(OffloadGateway offloadGateway) {
    gateway = Objects.requireNonNull(offloadGateway);
  }

  public static boolean isGatewayConfigured() {
    return gateway != NOOP_GATEWAY;
  }

  public static void clearGateway() {
    gateway = NOOP_GATEWAY;
  }

  public static OffloadGateway gateway() {
    return gateway;
  }

  public static Executor fallbackExecutor() {
    return FALLBACK_EXECUTOR;
  }

  public static <RequestT, ResponseT> CompletableFuture<ResponseT> callTyped(
      String taskId, RequestT request, Class<ResponseT> responseType, int timeoutMs) {
    byte[] payloadBytes;
    try {
      payloadBytes = CborSerde.write(request);
    } catch (RuntimeException ex) {
      return CompletableFuture.failedFuture(ex);
    }

    return gateway
        .call(taskId, payloadBytes, timeoutMs)
        .thenApply(
            bytes -> {
              try {
                return CborSerde.read(bytes, responseType);
              } catch (RuntimeException ex) {
                throw new CompletionException(ex);
              }
            });
  }
}
