package org.curtinfrc.frc2026.util.Repulsor.Offload;

import java.util.Objects;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;

public final class OffloadCached<T> {
  private final AtomicReference<T> lastSuccessful = new AtomicReference<>();
  private final AtomicBoolean inFlight = new AtomicBoolean(false);

  public CompletableFuture<T> refreshOrLast(Supplier<CompletableFuture<T>> requestSupplier) {
    Objects.requireNonNull(requestSupplier);

    if (!inFlight.compareAndSet(false, true)) {
      return CompletableFuture.completedFuture(lastSuccessful.get());
    }

    CompletableFuture<T> request;
    try {
      request = requestSupplier.get();
    } catch (Exception ex) {
      inFlight.set(false);
      return CompletableFuture.completedFuture(lastSuccessful.get());
    }

    return request.handle(
        (value, error) -> {
          inFlight.set(false);
          if (error == null) {
            lastSuccessful.set(value);
            return value;
          }
          return lastSuccessful.get();
        });
  }

  public T getLastSuccessful() {
    return lastSuccessful.get();
  }
}
