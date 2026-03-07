package org.curtinfrc.frc2026.util.Repulsor.Offload.Client;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetSocketAddress;
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
import org.curtinfrc.frc2026.util.Repulsor.Offload.OffloadResponseEnvelope;

public final class UdpOffloadClient implements OffloadGateway, AutoCloseable {
  private static final int MAX_UDP_PACKET_BYTES = 65_507;

  private final OffloadClientConfig config;
  private final ArrayBlockingQueue<OutboundRequest> outboundQueue;
  private final Map<Long, PendingRequest> pendingByCorrelation = new ConcurrentHashMap<>();
  private final Map<String, OffloadServerTiming> latestServerTimingByTask =
      new ConcurrentHashMap<>();
  private final AtomicLong nextCorrelation = new AtomicLong(1L);

  private final ExecutorService ioExecutor;
  private volatile boolean running;

  private volatile DatagramSocket socket;
  private volatile OffloadHost connectedHost;
  private volatile boolean healthy;
  private volatile String manifestHash = "";

  public UdpOffloadClient(OffloadClientConfig config) {
    this.config = Objects.requireNonNull(config);
    this.outboundQueue = new ArrayBlockingQueue<>(config.queueCapacity());
    this.ioExecutor =
        Executors.newSingleThreadExecutor(
            new ThreadFactory() {
              @Override
              public Thread newThread(Runnable runnable) {
                Thread thread = new Thread(runnable, "offload-client-udp-io");
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
            correlationId,
            taskId,
            future,
            System.nanoTime() + TimeUnit.MILLISECONDS.toNanos(timeoutMs));

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

  public Optional<OffloadServerTiming> latestServerTiming(String taskId) {
    if (taskId == null) {
      return Optional.empty();
    }
    return Optional.ofNullable(latestServerTimingByTask.get(taskId));
  }

  private void ioLoop() {
    while (running) {
      try {
        ensureConnected();
        DatagramSocket activeSocket = requireActiveSocket();
        int sentThisLoop = drainOutbound(activeSocket, 16);
        if (sentThisLoop == 0 && pendingByCorrelation.isEmpty()) {
          waitForOutboundAndSend(activeSocket);
          expireTimedOutRequests();
          continue;
        }

        if (!pendingByCorrelation.isEmpty()) {
          readResponses(activeSocket);
        }
        expireTimedOutRequests();
      } catch (InterruptedException ex) {
        Thread.currentThread().interrupt();
        break;
      } catch (Exception ex) {
        disconnect(ex);
        sleepQuietly(40);
      }
    }
    disconnect(new OffloadCallException("Offload client stopped"));
  }

  private void ensureConnected() throws IOException {
    if (socket != null && !socket.isClosed()) {
      return;
    }

    ProbeResult best =
        config.probeBeforeConnect() ? selectBestHost() : new ProbeResult(config.hosts().get(0), 0L);

    DatagramSocket newSocket = new DatagramSocket();
    newSocket.connect(new InetSocketAddress(best.host().host(), best.host().port()));
    int handshakeTimeoutMs = Math.max(config.readTimeoutMs(), config.connectTimeoutMs());
    newSocket.setSoTimeout(handshakeTimeoutMs);
    OffloadHelloResponse hello = hello(newSocket);
    newSocket.setSoTimeout(config.readTimeoutMs());

    socket = newSocket;
    connectedHost = best.host();
    healthy = true;
    manifestHash = hello.getManifestHash() == null ? "" : hello.getManifestHash();
  }

  private ProbeResult selectBestHost() {
    PriorityQueue<ProbeResult> successful =
        new PriorityQueue<>(Comparator.comparingLong(ProbeResult::latencyNanos));

    for (OffloadHost host : config.hosts()) {
      try (DatagramSocket probeSocket = new DatagramSocket()) {
        probeSocket.connect(new InetSocketAddress(host.host(), host.port()));
        probeSocket.setSoTimeout(config.probeTimeoutMs());
        long start = System.nanoTime();
        hello(probeSocket);
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

  private OffloadHelloResponse hello(DatagramSocket sourceSocket) throws IOException {
    long correlation = nextCorrelation.getAndIncrement();
    byte[] helloBytes = OffloadProtocol.serializeRequest(correlation, OffloadProtocol.TASK_HELLO, new byte[0]);
    sendPacket(sourceSocket, helloBytes);

    while (true) {
      OffloadProtocol.ResponseFrame response = receivePacket(sourceSocket);
      if (response.correlationId() != correlation) {
        continue;
      }
      if (response.status() != OffloadProtocol.STATUS_OK) {
        throw new IOException("HELLO failed");
      }
      return CborSerde.read(response.payload(), OffloadHelloResponse.class);
    }
  }

  private DatagramSocket requireActiveSocket() throws IOException {
    DatagramSocket activeSocket = socket;
    if (activeSocket == null || activeSocket.isClosed()) {
      throw new IOException("UDP socket is unavailable");
    }
    return activeSocket;
  }

  private int drainOutbound(DatagramSocket activeSocket, int burstLimit) throws IOException {
    int sent = 0;
    for (int i = 0; i < Math.max(1, burstLimit); i++) {
      OutboundRequest request = outboundQueue.poll();
      if (request == null) {
        break;
      }
      sendRequest(activeSocket, request);
      sent++;
    }
    return sent;
  }

  private void waitForOutboundAndSend(DatagramSocket activeSocket)
      throws IOException, InterruptedException {
    OutboundRequest request =
        outboundQueue.poll(Math.max(1, config.readTimeoutMs()), TimeUnit.MILLISECONDS);
    if (request == null) {
      return;
    }
    sendRequest(activeSocket, request);
    drainOutbound(activeSocket, 15);
  }

  private void sendRequest(DatagramSocket activeSocket, OutboundRequest request) throws IOException {
    byte[] frame =
        OffloadProtocol.serializeRequest(request.correlationId(), request.taskId(), request.payload());
    if (frame.length > MAX_UDP_PACKET_BYTES) {
      PendingRequest pending = pendingByCorrelation.remove(request.correlationId());
      if (pending != null) {
        pending
            .future()
            .completeExceptionally(
                new OffloadCallException(
                    "Offload request too large for UDP packet: " + frame.length + " bytes"));
      }
      return;
    }
    sendPacket(activeSocket, frame);
  }

  private void readResponses(DatagramSocket activeSocket) throws IOException {
    try {
      OffloadProtocol.ResponseFrame response = receivePacket(activeSocket);
      PendingRequest pending = pendingByCorrelation.remove(response.correlationId());
      if (pending == null) {
        return;
      }
      if (response.status() == OffloadProtocol.STATUS_OK) {
        byte[] payload = response.payload();
        OffloadResponseEnvelope envelope = tryDecodeEnvelope(payload);
        if (envelope != null
            && envelope.getPayload() != null
            && envelope.getQueueNs() >= 0
            && envelope.getExecuteNs() >= 0
            && envelope.getServerNs() > 0) {
          latestServerTimingByTask.put(
              pending.taskId(),
              new OffloadServerTiming(
                  envelope.getQueueNs(), envelope.getExecuteNs(), envelope.getServerNs()));
          pending.future().complete(envelope.getPayload());
          return;
        }
        pending.future().complete(payload);
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

  private static OffloadProtocol.ResponseFrame receivePacket(DatagramSocket sourceSocket)
      throws IOException {
    byte[] receiveBuffer = new byte[MAX_UDP_PACKET_BYTES];
    DatagramPacket packet = new DatagramPacket(receiveBuffer, receiveBuffer.length);
    sourceSocket.receive(packet);
    byte[] frame = new byte[packet.getLength()];
    System.arraycopy(packet.getData(), packet.getOffset(), frame, 0, packet.getLength());
    return OffloadProtocol.parseResponse(frame);
  }

  private static void sendPacket(DatagramSocket targetSocket, byte[] frame) throws IOException {
    DatagramPacket packet = new DatagramPacket(frame, frame.length);
    targetSocket.send(packet);
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

  private static OffloadResponseEnvelope tryDecodeEnvelope(byte[] payload) {
    try {
      return CborSerde.read(payload, OffloadResponseEnvelope.class);
    } catch (RuntimeException ignored) {
      return null;
    }
  }

  private void disconnect(Exception cause) {
    healthy = false;
    DatagramSocket existingSocket = socket;
    socket = null;
    connectedHost = null;
    if (existingSocket != null) {
      existingSocket.close();
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
      long correlationId, String taskId, CompletableFuture<byte[]> future, long deadlineNanos) {}

  private record ProbeResult(OffloadHost host, long latencyNanos) {}
}
