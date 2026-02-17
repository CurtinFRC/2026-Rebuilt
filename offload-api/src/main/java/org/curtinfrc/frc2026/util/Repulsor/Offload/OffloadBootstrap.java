package org.curtinfrc.frc2026.util.Repulsor.Offload;

import java.util.Objects;

public final class OffloadBootstrap {
  private static volatile Runnable initializer = () -> {};

  private OffloadBootstrap() {}

  public static void registerInitializer(Runnable init) {
    initializer = Objects.requireNonNull(init);
  }

  public static void ensureInitialized() {
    initializer.run();
  }
}
