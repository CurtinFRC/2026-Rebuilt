package org.curtinfrc.frc2026.util.Repulsor.Offload.Client;

import java.util.Objects;

public record OffloadHost(String host, int port) {
  public OffloadHost {
    Objects.requireNonNull(host);
  }
}
