package org.curtinfrc.frc2026.util.Repulsor.Offload;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.net.ServerSocket;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardCopyOption;
import java.util.Comparator;
import java.util.List;
import java.util.concurrent.TimeUnit;
import org.curtinfrc.frc2026.util.Repulsor.Offload.Client.OffloadClientConfig;
import org.curtinfrc.frc2026.util.Repulsor.Offload.Client.OffloadHost;
import org.curtinfrc.frc2026.util.Repulsor.Offload.Client.TcpOffloadClient;
import org.curtinfrc.frc2026.util.Repulsor.Offload.Server.OffloadServer;
import org.curtinfrc.frc2026.util.Repulsor.Offload.Server.OffloadServerConfig;
import org.junit.jupiter.api.Test;

class LocalOffloadSmokeTest {
  @Test
  void shouldOffloadComputeToLocalServerAndFallbackWhenDisconnected() throws Exception {
    OffloadRpc.clearGateway();
    Path pluginDirectory = Files.createTempDirectory("offload-local-smoke");
    int port = findFreePort();

    try {
      Path taskJar =
          requirePath(
              "repulsor.offload.test.taskJar",
              Path.of("offload-tasks/build/libs/offload-tasks.jar"));
      Path manifest =
          requirePath(
              "repulsor.offload.test.manifest", Path.of("build/offload/offload-manifest.json"));

      copy(taskJar, pluginDirectory.resolve("offload-tasks.jar"));
      copy(manifest, pluginDirectory.resolve("offload-manifest.json"));

      OffloadServerConfig config =
          new OffloadServerConfig(port, pluginDirectory, 2, "local-smoke", "1.0", true);

      try (OffloadServer server = new OffloadServer(config);
          TcpOffloadClient client =
              new TcpOffloadClient(
                  OffloadClientConfig.builder()
                      .hosts(List.of(new OffloadHost("127.0.0.1", port)))
                      .connectTimeoutMs(500)
                      .probeTimeoutMs(500)
                      .readTimeoutMs(100)
                      .probeBeforeConnect(false)
                      .queueCapacity(32)
                      .build())) {
        assertTrue(server.taskCount() > 0, "No tasks loaded from local plugin JAR");

        server.start();
        OffloadRpc.setGateway(client);
        client.start();

        waitForHealthy(client, 3000L);
        assertTrue(client.isHealthy(), "Client did not become healthy");

        assertTrue(runRemoteProbe(), "Expected method to execute on offload server worker thread");

        client.close();
        waitForUnhealthy(client, 1000L);

        boolean fallback =
            SampleMathOffloadEntrypoints_Offloaded.runsOnOffloadWorkerThread_offload(1);
        assertFalse(fallback, "Expected local fallback execution after disconnect");
      }
    } finally {
      OffloadRpc.clearGateway();
      deleteTree(pluginDirectory);
    }
  }

  private static Path requirePath(String systemProperty, Path fallback) {
    Path path = Path.of(System.getProperty(systemProperty, fallback.toString()));
    assertTrue(Files.exists(path), "Required file missing: " + path.toAbsolutePath());
    return path;
  }

  private static void copy(Path source, Path target) throws IOException {
    Files.copy(source, target, StandardCopyOption.REPLACE_EXISTING);
  }

  private static int findFreePort() throws IOException {
    try (ServerSocket socket = new ServerSocket(0)) {
      return socket.getLocalPort();
    }
  }

  private static void waitForHealthy(TcpOffloadClient client, long timeoutMs)
      throws InterruptedException {
    long deadline = System.currentTimeMillis() + timeoutMs;
    while (!client.isHealthy() && System.currentTimeMillis() < deadline) {
      Thread.sleep(10L);
    }
  }

  private static void waitForUnhealthy(TcpOffloadClient client, long timeoutMs)
      throws InterruptedException {
    long deadline = System.currentTimeMillis() + timeoutMs;
    while (client.isHealthy() && System.currentTimeMillis() < deadline) {
      Thread.sleep(10L);
    }
  }

  private static boolean runRemoteProbe() throws Exception {
    SampleMathOffloadEntrypoints_runsOnOffloadWorkerThread_OffloadRequest request =
        new SampleMathOffloadEntrypoints_runsOnOffloadWorkerThread_OffloadRequest();
    request.setArg0(OffloadValueCodec.encode("int", 1));
    SampleMathOffloadEntrypoints_runsOnOffloadWorkerThread_OffloadResponse response =
        OffloadRpc.callTyped(
                OffloadTaskIds.SAMPLE_WORKER_THREAD_PROBE,
                request,
                SampleMathOffloadEntrypoints_runsOnOffloadWorkerThread_OffloadResponse.class,
                500)
            .get(1000L, TimeUnit.MILLISECONDS);
    return (boolean) OffloadValueCodec.decode("boolean", response.getResult());
  }

  private static void deleteTree(Path root) throws IOException {
    if (!Files.exists(root)) {
      return;
    }
    try (var walk = Files.walk(root)) {
      walk.sorted(Comparator.reverseOrder()).forEach(LocalOffloadSmokeTest::deleteQuietly);
    }
  }

  private static void deleteQuietly(Path path) {
    try {
      Files.deleteIfExists(path);
    } catch (IOException ignored) {
    }
  }
}
