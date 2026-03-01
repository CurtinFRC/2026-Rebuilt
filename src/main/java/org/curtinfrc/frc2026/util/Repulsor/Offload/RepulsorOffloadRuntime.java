package org.curtinfrc.frc2026.util.Repulsor.Offload;

import java.lang.reflect.Constructor;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

public final class RepulsorOffloadRuntime {
  private static final String OFFLOAD_BOOTSTRAP_CLASS =
      "org.curtinfrc.frc2026.util.Repulsor.Offload.OffloadBootstrap";
  private static final String OFFLOAD_RPC_CLASS =
      "org.curtinfrc.frc2026.util.Repulsor.Offload.OffloadRpc";
  private static final String OFFLOAD_GATEWAY_CLASS =
      "org.curtinfrc.frc2026.util.Repulsor.Offload.OffloadGateway";
  private static final String OFFLOAD_HOST_CLASS =
      "org.curtinfrc.frc2026.util.Repulsor.Offload.Client.OffloadHost";
  private static final String OFFLOAD_CLIENT_CONFIG_CLASS =
      "org.curtinfrc.frc2026.util.Repulsor.Offload.Client.OffloadClientConfig";
  private static final String TCP_OFFLOAD_CLIENT_CLASS =
      "org.curtinfrc.frc2026.util.Repulsor.Offload.Client.TcpOffloadClient";

  static {
    registerBootstrapInitializer();
  }

  private static final AtomicBoolean INITIALIZED = new AtomicBoolean(false);
  private static volatile Object client;
  private static volatile boolean missingDependencyReported;

  private RepulsorOffloadRuntime() {}

  public static void ensureInitialized() {
    if (!INITIALIZED.compareAndSet(false, true)) {
      return;
    }

    try {
      if (isGatewayConfigured()) {
        return;
      }

      Object config = buildClientConfig();
      if (config == null) {
        reportMissingDependencyOnce("Offload disabled: client config class unavailable.");
        return;
      }

      Class<?> clientClass = Class.forName(TCP_OFFLOAD_CLIENT_CLASS);
      Constructor<?> ctor = clientClass.getConstructor(config.getClass());
      Object newClient = ctor.newInstance(config);
      clientClass.getMethod("start").invoke(newClient);
      setGateway(newClient);
      client = newClient;
    } catch (ClassNotFoundException ex) {
      reportMissingDependencyOnce("Offload disabled: missing offload classes.");
    } catch (ReflectiveOperationException | LinkageError ex) {
      throw new IllegalStateException("Failed to initialize offload runtime.", ex);
    }
  }

  public static void shutdown() {
    Object current = client;
    if (current != null) {
      try {
        current.getClass().getMethod("close").invoke(current);
      } catch (ReflectiveOperationException ignored) {
      }
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

  private static void registerBootstrapInitializer() {
    try {
      Class<?> bootstrapClass = Class.forName(OFFLOAD_BOOTSTRAP_CLASS);
      Method register = bootstrapClass.getMethod("registerInitializer", Runnable.class);
      register.invoke(null, (Runnable) RepulsorOffloadRuntime::ensureInitialized);
    } catch (ClassNotFoundException ignored) {
    } catch (ReflectiveOperationException ex) {
      throw new IllegalStateException("Failed to register offload bootstrap initializer.", ex);
    }
  }

  private static boolean isGatewayConfigured() throws ReflectiveOperationException {
    Class<?> rpcClass = Class.forName(OFFLOAD_RPC_CLASS);
    Method isGatewayConfigured = rpcClass.getMethod("isGatewayConfigured");
    return (boolean) isGatewayConfigured.invoke(null);
  }

  private static void setGateway(Object gateway) throws ReflectiveOperationException {
    Class<?> rpcClass = Class.forName(OFFLOAD_RPC_CLASS);
    Class<?> gatewayClass = Class.forName(OFFLOAD_GATEWAY_CLASS);
    Method setGateway = rpcClass.getMethod("setGateway", gatewayClass);
    setGateway.invoke(null, gateway);
  }

  private static Object buildClientConfig() throws ReflectiveOperationException {
    Class<?> configClass = Class.forName(OFFLOAD_CLIENT_CONFIG_CLASS);
    Object builder = configClass.getMethod("builder").invoke(null);

    builder = invokeBuilder(builder, "hosts", List.class, parseHosts());
    builder =
        invokeBuilder(
            builder,
            "connectTimeoutMs",
            int.class,
            intProp("repulsor.offload.connectTimeoutMs", 50));
    builder =
        invokeBuilder(
            builder, "readTimeoutMs", int.class, intProp("repulsor.offload.readTimeoutMs", 5));
    builder =
        invokeBuilder(
            builder, "queueCapacity", int.class, intProp("repulsor.offload.queueCapacity", 128));
    builder =
        invokeBuilder(
            builder, "probeTimeoutMs", int.class, intProp("repulsor.offload.probeTimeoutMs", 40));
    return builder.getClass().getMethod("build").invoke(builder);
  }

  private static Object invokeBuilder(Object target, String method, Class<?> argType, Object arg)
      throws ReflectiveOperationException {
    Method setter = target.getClass().getMethod(method, argType);
    return setter.invoke(target, arg);
  }

  private static List<Object> parseHosts() {
    String configured =
        System.getProperty(
            "repulsor.offload.hosts",
            System.getenv().getOrDefault("REPULSOR_OFFLOAD_HOSTS", "127.0.0.1:5808"));

    List<Object> hosts = new ArrayList<>();
    for (String token : configured.split(",")) {
      String trimmed = token.trim();
      if (trimmed.isEmpty()) {
        continue;
      }

      String[] parts = trimmed.split(":", 2);
      String host = parts[0].trim();
      int port = parts.length == 2 ? intOr(parts[1].trim(), 5808) : 5808;
      Object offloadHost = newHost(host, port);
      if (offloadHost != null) {
        hosts.add(offloadHost);
      }
    }

    if (hosts.isEmpty()) {
      Object defaultHost = newHost("127.0.0.1", 5808);
      if (defaultHost != null) {
        hosts.add(defaultHost);
      }
    }
    return hosts;
  }

  private static Object newHost(String host, int port) {
    try {
      Class<?> hostClass = Class.forName(OFFLOAD_HOST_CLASS);
      Constructor<?> ctor = hostClass.getConstructor(String.class, int.class);
      return ctor.newInstance(host, port);
    } catch (ReflectiveOperationException ex) {
      return null;
    }
  }

  private static int intOr(String value, int fallback) {
    try {
      return Integer.parseInt(value);
    } catch (NumberFormatException ex) {
      return fallback;
    }
  }

  private static void reportMissingDependencyOnce(String message) {
    if (missingDependencyReported) {
      return;
    }
    missingDependencyReported = true;
    System.err.println(message);
  }
}
