package org.curtinfrc.frc2026.util.Repulsor.Offload;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

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
    OffloadRpc.clearGateway();
    try (MockOffloadServer server = new MockOffloadServer()) {
      TcpOffloadClient client =
          new TcpOffloadClient(
              OffloadClientConfig.builder()
                  .hosts(List.of(new OffloadHost("127.0.0.1", server.port())))
                  .connectTimeoutMs(250)
                  .probeTimeoutMs(250)
                  .readTimeoutMs(50)
                  .probeBeforeConnect(false)
                  .queueCapacity(16)
                  .build());
      OffloadRpc.setGateway(client);
      client.start();

      long waitUntil = System.currentTimeMillis() + 3000L;
      while (!client.isHealthy() && System.currentTimeMillis() < waitUntil) {
        Thread.sleep(10L);
      }
      assertTrue(client.isHealthy(), "Client did not become healthy before offload call");

      int remote = SampleMathOffloadEntrypoints_Offloaded.doubleValue_offload(7);
      assertEquals(100, remote);

      client.close();

      int fallback = SampleMathOffloadEntrypoints_Offloaded.doubleValue_offload(7);
      assertEquals(14, fallback);
    } finally {
      OffloadRpc.clearGateway();
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

          if (OffloadTaskIds.SAMPLE_DOUBLE_VALUE.equals(request.taskId())) {
            SampleMathOffloadEntrypoints_doubleValue_OffloadResponse response =
                new SampleMathOffloadEntrypoints_doubleValue_OffloadResponse();
            response.setResult(OffloadValueCodec.encode("int", 100));
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
