package org.curtinfrc.frc2026.util.Repulsor.Offload.Client;

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
import java.util.concurrent.TimeUnit;
import org.curtinfrc.frc2026.util.Repulsor.Offload.CborSerde;
import org.curtinfrc.frc2026.util.Repulsor.Offload.OffloadError;
import org.curtinfrc.frc2026.util.Repulsor.Offload.OffloadHelloResponse;
import org.curtinfrc.frc2026.util.Repulsor.Offload.OffloadProtocol;
import org.junit.jupiter.api.Test;

class TcpOffloadClientRoundTripTest {
  @Test
  void shouldRoundTripRequestAndResponse() throws Exception {
    try (EchoServer server = new EchoServer()) {
      TcpOffloadClient client =
          new TcpOffloadClient(
              OffloadClientConfig.builder()
                  .hosts(List.of(new OffloadHost("127.0.0.1", server.port())))
                  .connectTimeoutMs(1500)
                  .probeTimeoutMs(1500)
                  .readTimeoutMs(1500)
                  .probeBeforeConnect(false)
                  .queueCapacity(32)
                  .build());
      client.start();

      long waitUntil = System.currentTimeMillis() + 1500L;
      while (!client.isHealthy() && System.currentTimeMillis() < waitUntil) {
        Thread.sleep(10L);
      }
      assertTrue(client.isHealthy(), "Client did not become healthy before first request");

      EchoRequest request = new EchoRequest("ping");
      byte[] payload = CborSerde.write(request);
      byte[] responseBytes = client.call("echo.task", payload, 5000).get(6, TimeUnit.SECONDS);
      EchoResponse response = CborSerde.read(responseBytes, EchoResponse.class);

      assertEquals("PING", response.value);
      client.close();
    }
  }

  private static final class EchoServer implements AutoCloseable {
    private final ServerSocket serverSocket;
    private final ExecutorService executor = Executors.newCachedThreadPool();
    private volatile boolean running = true;

    EchoServer() throws IOException {
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
          executor.submit(() -> handle(socket));
        } catch (IOException ex) {
          if (running) {
            throw new RuntimeException(ex);
          }
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
            OffloadProtocol.writeResponse(
                output,
                request.correlationId(),
                OffloadProtocol.STATUS_OK,
                CborSerde.write(new OffloadHelloResponse("test", "1", "hash", List.of())));
            continue;
          }

          if ("echo.task".equals(request.taskId())) {
            EchoRequest value = CborSerde.read(request.payload(), EchoRequest.class);
            OffloadProtocol.writeResponse(
                output,
                request.correlationId(),
                OffloadProtocol.STATUS_OK,
                CborSerde.write(new EchoResponse(value.value.toUpperCase())));
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

  private static final class EchoRequest {
    public String value;

    public EchoRequest() {}

    EchoRequest(String value) {
      this.value = value;
    }
  }

  private static final class EchoResponse {
    public String value;

    public EchoResponse() {}

    EchoResponse(String value) {
      this.value = value;
    }
  }
}
