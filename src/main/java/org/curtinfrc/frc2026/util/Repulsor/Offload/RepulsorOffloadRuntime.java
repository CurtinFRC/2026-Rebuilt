package org.curtinfrc.frc2026.util.Repulsor.Offload;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import org.curtinfrc.frc2026.util.Repulsor.Offload.Client.OffloadClientConfig;
import org.curtinfrc.frc2026.util.Repulsor.Offload.Client.OffloadHost;
import org.curtinfrc.frc2026.util.Repulsor.Offload.Client.TcpOffloadClient;

public final class RepulsorOffloadRuntime {
  static {
    OffloadBootstrap.registerInitializer(RepulsorOffloadRuntime::ensureInitialized);
  }

  private static final AtomicBoolean INITIALIZED = new AtomicBoolean(false);
  private static TcpOffloadClient client;

  private RepulsorOffloadRuntime() {}

  public static void ensureInitialized() {
    if (OffloadRpc.isGatewayConfigured()) {
      INITIALIZED.set(true);
      return;
    }

    if (!INITIALIZED.compareAndSet(false, true)) {
      return;
    }

    OffloadClientConfig config =
        OffloadClientConfig.builder()
            .hosts(parseHosts())
            .connectTimeoutMs(intProp("repulsor.offload.connectTimeoutMs", 50))
            .readTimeoutMs(intProp("repulsor.offload.readTimeoutMs", 5))
            .queueCapacity(intProp("repulsor.offload.queueCapacity", 128))
            .probeTimeoutMs(intProp("repulsor.offload.probeTimeoutMs", 40))
            .build();

    client = new TcpOffloadClient(config);
    client.start();
    OffloadRpc.setGateway(client);
  }

  public static void shutdown() {
    TcpOffloadClient current = client;
    if (current != null) {
      current.close();
    }
  }

  private static int intProp(String name, int fallback) {
    String value = System.getProperty(name);
    if (value == null || value.isBlank()) {
      return fallback;
    }
    try {
      return Integer.parseInt(value);
    } catch (NumberFormatException ignored) {
      return fallback;
    }
  }

  private static List<OffloadHost> parseHosts() {
    String configured =
        System.getProperty(
            "repulsor.offload.hosts",
            System.getenv().getOrDefault("REPULSOR_OFFLOAD_HOSTS", "127.0.0.1:5808"));

    List<OffloadHost> hosts = new ArrayList<>();
    for (String token : configured.split(",")) {
      String trimmed = token.trim();
      if (trimmed.isEmpty()) {
        continue;
      }

      String[] parts = trimmed.split(":", 2);
      String host = parts[0].trim();
      int port = parts.length == 2 ? Integer.parseInt(parts[1].trim()) : 5808;
      hosts.add(new OffloadHost(host, port));
    }

    if (hosts.isEmpty()) {
      hosts.add(new OffloadHost("127.0.0.1", 5808));
    }
    return hosts;
  }
}
