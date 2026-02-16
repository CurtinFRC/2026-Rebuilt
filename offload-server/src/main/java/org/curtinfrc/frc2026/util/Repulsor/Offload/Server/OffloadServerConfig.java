package org.curtinfrc.frc2026.util.Repulsor.Offload.Server;

import java.nio.file.Path;

public record OffloadServerConfig(
    int port,
    Path pluginDirectory,
    int workerThreads,
    String serverName,
    String serverVersion,
    boolean includeTaskListInHello) {

  public static OffloadServerConfig fromEnvironment() {
    int port = Integer.parseInt(System.getenv().getOrDefault("OFFLOAD_PORT", "5808"));
    Path pluginDirectory =
        Path.of(System.getenv().getOrDefault("OFFLOAD_PLUGIN_DIR", "/opt/offload/plugins"));
    int workerThreads = Integer.parseInt(System.getenv().getOrDefault("OFFLOAD_WORKERS", "4"));
    String serverName = System.getenv().getOrDefault("OFFLOAD_SERVER_NAME", "repulsor-offload");
    String serverVersion = System.getenv().getOrDefault("OFFLOAD_SERVER_VERSION", "1.0.0");
    boolean includeTaskList =
        Boolean.parseBoolean(System.getenv().getOrDefault("OFFLOAD_HELLO_TASK_LIST", "false"));
    return new OffloadServerConfig(
        port, pluginDirectory, workerThreads, serverName, serverVersion, includeTaskList);
  }
}
