package org.curtinfrc.frc2026.util.Repulsor.Offload;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import org.curtinfrc.frc2026.util.Repulsor.Offload.Client.OffloadClientConfig;
import org.curtinfrc.frc2026.util.Repulsor.Offload.Client.OffloadHost;
import org.curtinfrc.frc2026.util.Repulsor.Offload.Client.TcpOffloadClient;
import org.junit.jupiter.api.Test;

class DragShotOffloadIntegrationTest {
  @Test
  void shouldUseRemoteThenFallbackWhenServerIsDown() throws Exception {
    try (MockOffloadServer server = new MockOffloadServer()) {
      TcpOffloadClient client =
          new TcpOffloadClient(
              OffloadClientConfig.builder()
                  .hosts(List.of(new OffloadHost("127.0.0.1", server.port())))
                  .connectTimeoutMs(80)
                  .probeTimeoutMs(80)
                  .readTimeoutMs(5)
                  .queueCapacity(16)
                  .build());
      OffloadRpc.setGateway(client);
      client.start();

      long waitUntil = System.currentTimeMillis() + 500L;
      while (!client.isHealthy() && System.currentTimeMillis() < waitUntil) {
        Thread.sleep(10L);
      }

      SampleMathRequestDTO request = new SampleMathRequestDTO(7);

      SampleMathResponseDTO remote = SampleMathOffloadEntrypoints_Offloaded.doubleValue(request);
      assertEquals(100, remote.getOutput());

      client.close();

      SampleMathResponseDTO fallback = SampleMathOffloadEntrypoints_Offloaded.doubleValue(request);
      assertNotNull(fallback);
      assertEquals(14, fallback.getOutput());
    }
  }

  private static final class MockOffloadServer implements AutoCloseable {
    private final ServerSocket serverSocket;
    private final ExecutorService executor = Executors.newCachedThreadPool();
    private volatile boolean running = true;

    MockOffloadServer() throws IOException {
      serverSocket = new ServerSocket(0);
      executor.submit(this::acceptLoop);
    }

    int port() {
      return serverSocket.getLocalPort();
    }

    private void acceptLoop() {
      while (running) {
        try {
          Socket socket = serverSocket.accept();
          socket.setTcpNoDelay(true);
          executor.submit(() -> handle(socket));
        } catch (IOException ex) {
          if (running) {
            throw new RuntimeException(ex);
          }
          return;
        }
      }
    }

    private void handle(Socket socket) {
      try (socket;
          DataInputStream input = new DataInputStream(socket.getInputStream());
          DataOutputStream output = new DataOutputStream(socket.getOutputStream())) {
        while (running && !socket.isClosed()) {
          OffloadProtocol.RequestFrame request = OffloadProtocol.readRequest(input);
          if (OffloadProtocol.TASK_HELLO.equals(request.taskId())) {
            OffloadHelloResponse hello =
                new OffloadHelloResponse("test-server", "1.0", "hash", List.of());
            OffloadProtocol.writeResponse(
                output, request.correlationId(), OffloadProtocol.STATUS_OK, CborSerde.write(hello));
            continue;
          }

          if ("repulsor.sample.math.double.v1".equals(request.taskId())) {
            SampleMathResponseDTO response = new SampleMathResponseDTO(100);
            OffloadProtocol.writeResponse(
                output,
                request.correlationId(),
                OffloadProtocol.STATUS_OK,
                CborSerde.write(response));
            continue;
          }

          OffloadProtocol.writeResponse(
              output,
              request.correlationId(),
              OffloadProtocol.STATUS_ERR,
              CborSerde.write(new OffloadError("UNKNOWN", "task")));
        }
      } catch (IOException ignored) {
      }
    }

    @Override
    public void close() throws Exception {
      running = false;
      serverSocket.close();
      executor.shutdownNow();
    }
  }
}
