package org.curtinfrc.frc2026.util.Repulsor.Offload.Client;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.net.InetSocketAddress;
import java.net.Socket;
import java.net.SocketTimeoutException;
import java.util.Comparator;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.PriorityQueue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.ThreadFactory;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicLong;
import org.curtinfrc.frc2026.util.Repulsor.Offload.CborSerde;
import org.curtinfrc.frc2026.util.Repulsor.Offload.OffloadCallException;
import org.curtinfrc.frc2026.util.Repulsor.Offload.OffloadError;
import org.curtinfrc.frc2026.util.Repulsor.Offload.OffloadGateway;
import org.curtinfrc.frc2026.util.Repulsor.Offload.OffloadHelloResponse;
import org.curtinfrc.frc2026.util.Repulsor.Offload.OffloadProtocol;

public final class TcpOffloadClient implements OffloadGateway, AutoCloseable {
  private final OffloadClientConfig config;
  private final ArrayBlockingQueue<OutboundRequest> outboundQueue;
  private final Map<Long, PendingRequest> pendingByCorrelation = new ConcurrentHashMap<>();
  private final AtomicLong nextCorrelation = new AtomicLong(1L);

  private final ExecutorService ioExecutor;
  private volatile boolean running;

  private volatile Socket socket;
  private volatile DataInputStream input;
  private volatile DataOutputStream output;
  private volatile OffloadHost connectedHost;

  private volatile boolean healthy;
  private volatile String manifestHash = "";

  public TcpOffloadClient(OffloadClientConfig config) {
    this.config = Objects.requireNonNull(config);
    this.outboundQueue = new ArrayBlockingQueue<>(config.queueCapacity());
    this.ioExecutor =
        Executors.newSingleThreadExecutor(
            new ThreadFactory() {
              @Override
              public Thread newThread(Runnable runnable) {
                Thread thread = new Thread(runnable, "offload-client-io");
                thread.setDaemon(true);
                return thread;
              }
            });
  }

  public synchronized void start() {
    if (running) {
      return;
    }
    running = true;
    ioExecutor.submit(this::ioLoop);
  }

  @Override
  public CompletableFuture<byte[]> call(String taskId, byte[] payloadBytes, int timeoutMs) {
    Objects.requireNonNull(taskId);
    Objects.requireNonNull(payloadBytes);

    start();

    long correlationId = nextCorrelation.getAndIncrement();
    CompletableFuture<byte[]> future = new CompletableFuture<>();
    PendingRequest pending =
        new PendingRequest(
            correlationId, future, System.nanoTime() + TimeUnit.MILLISECONDS.toNanos(timeoutMs));

    pendingByCorrelation.put(correlationId, pending);
    boolean queued = outboundQueue.offer(new OutboundRequest(correlationId, taskId, payloadBytes));
    if (!queued) {
      pendingByCorrelation.remove(correlationId);
      future.completeExceptionally(new OffloadCallException("Offload queue is full"));
      return future;
    }

    return future;
  }

  @Override
  public boolean isHealthy() {
    return healthy;
  }

  public Optional<OffloadHost> connectedHost() {
    return Optional.ofNullable(connectedHost);
  }

  public String manifestHash() {
    return manifestHash;
  }

  private void ioLoop() {
    while (running) {
      try {
        ensureConnected();
        drainOutbound();
        readResponses();
        expireTimedOutRequests();
      } catch (Exception ex) {
        disconnect(ex);
        sleepQuietly(40);
      }
    }
    disconnect(new OffloadCallException("Offload client stopped"));
  }

  private void ensureConnected() throws IOException {
    if (socket != null && socket.isConnected() && !socket.isClosed()) {
      return;
    }

    ProbeResult best = selectBestHost();
    Socket newSocket = new Socket();
    newSocket.connect(
        new InetSocketAddress(best.host().host(), best.host().port()), config.connectTimeoutMs());
    newSocket.setTcpNoDelay(true);
    int handshakeTimeoutMs = Math.max(config.readTimeoutMs(), config.connectTimeoutMs());
    newSocket.setSoTimeout(handshakeTimeoutMs);

    DataInputStream newInput = new DataInputStream(newSocket.getInputStream());
    DataOutputStream newOutput = new DataOutputStream(newSocket.getOutputStream());

    OffloadHelloResponse hello = hello(newInput, newOutput);
    newSocket.setSoTimeout(config.readTimeoutMs());

    socket = newSocket;
    input = newInput;
    output = newOutput;
    connectedHost = best.host();
    healthy = true;
    manifestHash = hello.getManifestHash() == null ? "" : hello.getManifestHash();
  }

  private ProbeResult selectBestHost() {
    PriorityQueue<ProbeResult> successful =
        new PriorityQueue<>(Comparator.comparingLong(ProbeResult::latencyNanos));

    for (OffloadHost host : config.hosts()) {
      try {
        long start = System.nanoTime();
        try (Socket probeSocket = new Socket()) {
          probeSocket.connect(
              new InetSocketAddress(host.host(), host.port()), config.probeTimeoutMs());
          probeSocket.setTcpNoDelay(true);
          probeSocket.setSoTimeout(config.probeTimeoutMs());

          DataInputStream probeInput = new DataInputStream(probeSocket.getInputStream());
          DataOutputStream probeOutput = new DataOutputStream(probeSocket.getOutputStream());
          hello(probeInput, probeOutput);
        }
        successful.add(new ProbeResult(host, System.nanoTime() - start));
      } catch (Exception ignored) {
      }
    }

    if (!successful.isEmpty()) {
      return successful.poll();
    }

    OffloadHost first = config.hosts().get(0);
    return new ProbeResult(first, Long.MAX_VALUE);
  }

  private OffloadHelloResponse hello(DataInputStream sourceInput, DataOutputStream sourceOutput)
      throws IOException {
    long correlation = nextCorrelation.getAndIncrement();
    OffloadProtocol.writeRequest(
        sourceOutput, correlation, OffloadProtocol.TASK_HELLO, new byte[0]);
    OffloadProtocol.ResponseFrame response = OffloadProtocol.readResponse(sourceInput);
    if (response.correlationId() != correlation) {
      throw new IOException("HELLO correlation mismatch");
    }
    if (response.status() != OffloadProtocol.STATUS_OK) {
      throw new IOException("HELLO failed");
    }
    return CborSerde.read(response.payload(), OffloadHelloResponse.class);
  }

  private void drainOutbound() throws IOException {
    for (int i = 0; i < 8; i++) {
      OutboundRequest request = outboundQueue.poll();
      if (request == null) {
        return;
      }
      OffloadProtocol.writeRequest(
          output, request.correlationId(), request.taskId(), request.payload());
    }
  }

  private void readResponses() throws IOException {
    try {
      OffloadProtocol.ResponseFrame response = OffloadProtocol.readResponse(input);
      PendingRequest pending = pendingByCorrelation.remove(response.correlationId());
      if (pending == null) {
        return;
      }
      if (response.status() == OffloadProtocol.STATUS_OK) {
        pending.future().complete(response.payload());
      } else {
        OffloadError err = CborSerde.read(response.payload(), OffloadError.class);
        pending
            .future()
            .completeExceptionally(
                new OffloadCallException(
                    "Remote execution failed ["
                        + safe(err.getCode())
                        + "]: "
                        + safe(err.getMessage())));
      }
    } catch (SocketTimeoutException ignored) {
    }
  }

  private void expireTimedOutRequests() {
    long now = System.nanoTime();
    for (PendingRequest pending : pendingByCorrelation.values()) {
      if (pending.deadlineNanos() < now
          && pendingByCorrelation.remove(pending.correlationId()) != null) {
        pending
            .future()
            .completeExceptionally(
                new OffloadCallException(
                    "Offload request timed out for correlation=" + pending.correlationId()));
      }
    }
  }

  private static String safe(String value) {
    return value == null ? "" : value;
  }

  private void disconnect(Exception cause) {
    healthy = false;

    Socket existingSocket = socket;
    socket = null;
    input = null;
    output = null;
    connectedHost = null;

    if (existingSocket != null) {
      try {
        existingSocket.close();
      } catch (IOException ignored) {
      }
    }

    for (PendingRequest pending : pendingByCorrelation.values()) {
      if (pendingByCorrelation.remove(pending.correlationId()) != null) {
        pending
            .future()
            .completeExceptionally(
                new OffloadCallException("Disconnected from offload server", cause));
      }
    }
  }

  private static void sleepQuietly(long millis) {
    try {
      Thread.sleep(millis);
    } catch (InterruptedException ex) {
      Thread.currentThread().interrupt();
    }
  }

  @Override
  public synchronized void close() {
    running = false;
    ioExecutor.shutdownNow();
    disconnect(new OffloadCallException("Offload client closed"));
  }

  private record OutboundRequest(long correlationId, String taskId, byte[] payload) {}

  private record PendingRequest(
      long correlationId, CompletableFuture<byte[]> future, long deadlineNanos) {}

  private record ProbeResult(OffloadHost host, long latencyNanos) {}
}
