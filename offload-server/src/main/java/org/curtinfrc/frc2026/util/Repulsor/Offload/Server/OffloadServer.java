package org.curtinfrc.frc2026.util.Repulsor.Offload.Server;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.net.Inet4Address;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.SocketException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.ThreadFactory;
import java.util.concurrent.TimeUnit;
import org.curtinfrc.frc2026.util.Repulsor.Offload.CborSerde;
import org.curtinfrc.frc2026.util.Repulsor.Offload.OffloadError;
import org.curtinfrc.frc2026.util.Repulsor.Offload.OffloadFunction;
import org.curtinfrc.frc2026.util.Repulsor.Offload.OffloadHelloResponse;
import org.curtinfrc.frc2026.util.Repulsor.Offload.OffloadProtocol;

public final class OffloadServer implements AutoCloseable {
  private final OffloadServerConfig config;
  private final boolean timingLogsEnabled;
  private final Map<String, OffloadFunction<?, ?>> functionsById;
  private final List<ClassLoader> pluginClassLoaders;
  private final ExecutorService connectionPool;
  private final ExecutorService workerPool;
  private final String manifestHash;
  private final List<String> manifestTaskIds;

  private volatile boolean running;
  private volatile ServerSocket serverSocket;

  public OffloadServer(OffloadServerConfig config) throws IOException {
    this.config = Objects.requireNonNull(config);
    this.timingLogsEnabled = config.timingLogsEnabled();

    OffloadPluginLoader.LoadedPlugins loaded = OffloadPluginLoader.load(config.pluginDirectory());
    this.functionsById = loaded.functionsById();
    this.pluginClassLoaders = loaded.classLoaders();

    OffloadManifestLoader.ManifestInfo manifestInfo =
        OffloadManifestLoader.load(config.pluginDirectory());
    this.manifestHash = manifestInfo.hash();
    this.manifestTaskIds = List.copyOf(manifestInfo.taskIds());

    this.connectionPool =
        Executors.newCachedThreadPool(
            new ThreadFactory() {
              @Override
              public Thread newThread(Runnable runnable) {
                Thread thread = new Thread(runnable, "offload-server-connection");
                thread.setDaemon(true);
                return thread;
              }
            });
    this.workerPool =
        Executors.newFixedThreadPool(
            Math.max(1, config.workerThreads()),
            new ThreadFactory() {
              @Override
              public Thread newThread(Runnable runnable) {
                Thread thread = new Thread(runnable, "offload-server-worker");
                thread.setDaemon(true);
                return thread;
              }
            });
  }

  public void start() throws IOException {
    if (running) {
      return;
    }
    running = true;
    serverSocket = new ServerSocket(config.port(), 50, Inet4Address.getByName("0.0.0.0"));

    connectionPool.submit(
        () -> {
          while (running) {
            try {
              Socket socket = serverSocket.accept();
              socket.setTcpNoDelay(true);
              connectionPool.submit(() -> handleConnection(socket));
            } catch (SocketException ex) {
              if (running) {
                ex.printStackTrace();
              }
              break;
            } catch (IOException ex) {
              if (running) {
                ex.printStackTrace();
              }
            }
          }
        });
  }

  private void handleConnection(Socket socket) {
    Object writeLock = new Object();
    try (socket;
        DataInputStream input = new DataInputStream(socket.getInputStream());
        DataOutputStream output = new DataOutputStream(socket.getOutputStream())) {
      while (running && !socket.isClosed()) {
        OffloadProtocol.RequestFrame request = OffloadProtocol.readRequest(input);
        dispatchRequest(request, output, writeLock);
      }
    } catch (IOException ignored) {
    }
  }

  private void dispatchRequest(
      OffloadProtocol.RequestFrame request, DataOutputStream output, Object writeLock) {
    String taskId = request.taskId();

    if (OffloadProtocol.TASK_PING.equals(taskId)) {
      writeResponse(
          output, writeLock, request.correlationId(), OffloadProtocol.STATUS_OK, new byte[0]);
      return;
    }

    if (OffloadProtocol.TASK_HELLO.equals(taskId)) {
      List<String> tasks =
          config.includeTaskListInHello()
              ? new ArrayList<>(sortedTaskIds())
              : Collections.emptyList();
      OffloadHelloResponse hello =
          new OffloadHelloResponse(
              config.serverName(), config.serverVersion(), manifestHash, tasks);
      writeResponse(
          output,
          writeLock,
          request.correlationId(),
          OffloadProtocol.STATUS_OK,
          CborSerde.write(hello));
      return;
    }

    OffloadFunction<?, ?> function = functionsById.get(taskId);
    if (function == null) {
      writeResponse(
          output,
          writeLock,
          request.correlationId(),
          OffloadProtocol.STATUS_ERR,
          CborSerde.write(new OffloadError("UNKNOWN_TASK", "No task registered for id=" + taskId)));
      return;
    }

    RequestTiming timing = new RequestTiming(System.nanoTime());
    CompletableFuture.supplyAsync(
            () -> {
              long executeStartNs = System.nanoTime();
              timing.executeStartNs = executeStartNs;
              if (timingLogsEnabled) {
                logTaskStart(
                    taskId, request.correlationId(), timing.receivedNs, executeStartNs, function.timeoutMs());
              }
              try {
                return executeFunction(function, request.payload());
              } finally {
                timing.executeEndNs = System.nanoTime();
              }
            },
            workerPool)
        .orTimeout(function.timeoutMs(), TimeUnit.MILLISECONDS)
        .whenComplete(
            (payload, error) -> {
              long writeStartNs = System.nanoTime();
              if (error == null) {
                writeResponse(
                    output, writeLock, request.correlationId(), OffloadProtocol.STATUS_OK, payload);
                if (timingLogsEnabled) {
                  long writeEndNs = System.nanoTime();
                  logTaskStop(
                      taskId,
                      request.correlationId(),
                      timing,
                      writeEndNs,
                      writeEndNs - writeStartNs,
                      "OK",
                      "");
                }
              } else {
                Throwable resolved = unwrapCompletionError(error);
                resolved.printStackTrace(System.err);
                writeResponse(
                    output,
                    writeLock,
                    request.correlationId(),
                    OffloadProtocol.STATUS_ERR,
                    CborSerde.write(new OffloadError("EXEC_ERROR", summarizeRootCause(resolved))));
                if (timingLogsEnabled) {
                  long writeEndNs = System.nanoTime();
                  logTaskStop(
                      taskId,
                      request.correlationId(),
                      timing,
                      writeEndNs,
                      writeEndNs - writeStartNs,
                      "ERR",
                      summarizeRootCause(resolved));
                }
              }
            });
  }

  @SuppressWarnings("unchecked")
  private static <RequestT, ResponseT> byte[] executeFunction(
      OffloadFunction<RequestT, ResponseT> function, byte[] payloadBytes) {
    Thread thread = Thread.currentThread();
    ClassLoader previousClassLoader = thread.getContextClassLoader();
    ClassLoader pluginClassLoader = function.getClass().getClassLoader();
    try {
      thread.setContextClassLoader(pluginClassLoader);
      RequestT request = CborSerde.read(payloadBytes, function.requestType());
      ResponseT response = function.execute(request);
      return CborSerde.write(response);
    } catch (Exception ex) {
      throw new IllegalStateException(
          "Offload task failed: " + function.taskId() + " (" + summarizeRootCause(ex) + ")", ex);
    } finally {
      thread.setContextClassLoader(previousClassLoader);
    }
  }

  private static String summarizeRootCause(Throwable throwable) {
    Throwable root = throwable;
    while (root.getCause() != null && root.getCause() != root) {
      root = root.getCause();
    }
    String message = root.getMessage();
    if (message == null || message.isBlank()) {
      return root.getClass().getName();
    }
    return root.getClass().getName() + ": " + message;
  }

  private static Throwable unwrapCompletionError(Throwable throwable) {
    Throwable current = throwable;
    while ((current instanceof java.util.concurrent.CompletionException
            || current instanceof java.util.concurrent.ExecutionException)
        && current.getCause() != null
        && current.getCause() != current) {
      current = current.getCause();
    }
    return current;
  }

  private void writeResponse(
      DataOutputStream output, Object writeLock, long correlationId, byte status, byte[] payload) {
    synchronized (writeLock) {
      try {
        OffloadProtocol.writeResponse(output, correlationId, status, payload);
      } catch (IOException ignored) {
      }
    }
  }

  private void logTaskStart(
      String taskId, long correlationId, long receivedNs, long executeStartNs, int timeoutMs) {
    System.out.printf(
        "offload-server task-start corr=%d task=%s queueMs=%.3f timeoutMs=%d thread=%s%n",
        correlationId,
        taskId,
        nanosToMillis(executeStartNs - receivedNs),
        timeoutMs,
        Thread.currentThread().getName());
  }

  private void logTaskStop(
      String taskId,
      long correlationId,
      RequestTiming timing,
      long responseWrittenNs,
      long responseWriteNs,
      String status,
      String errorSummary) {
    double queueMs =
        timing.executeStartNs > 0 ? nanosToMillis(timing.executeStartNs - timing.receivedNs) : -1.0;
    double execMs =
        (timing.executeStartNs > 0 && timing.executeEndNs > 0)
            ? nanosToMillis(timing.executeEndNs - timing.executeStartNs)
            : -1.0;
    double serverMs = nanosToMillis(responseWrittenNs - timing.receivedNs);

    String error = (errorSummary == null || errorSummary.isBlank()) ? "-" : errorSummary;
    System.out.printf(
        "offload-server task-stop corr=%d task=%s status=%s queueMs=%.3f execMs=%.3f serverMs=%.3f writeMs=%.3f error=%s%n",
        correlationId, taskId, status, queueMs, execMs, serverMs, nanosToMillis(responseWriteNs), error);
  }

  private static double nanosToMillis(long nanos) {
    return nanos / 1_000_000.0;
  }

  private List<String> sortedTaskIds() {
    if (!manifestTaskIds.isEmpty()) {
      return manifestTaskIds;
    }
    List<String> ids = new ArrayList<>(functionsById.keySet());
    Collections.sort(ids);
    return ids;
  }

  public String manifestHash() {
    return manifestHash;
  }

  public int taskCount() {
    return functionsById.size();
  }

  public boolean timingLogsEnabled() {
    return timingLogsEnabled;
  }

  @Override
  public void close() {
    running = false;

    if (serverSocket != null) {
      try {
        serverSocket.close();
      } catch (IOException ignored) {
      }
    }

    connectionPool.shutdownNow();
    workerPool.shutdownNow();

    for (ClassLoader classLoader : pluginClassLoaders) {
      if (classLoader instanceof AutoCloseable autoCloseable) {
        try {
          autoCloseable.close();
        } catch (Exception ignored) {
        }
      }
    }
  }

  private static final class RequestTiming {
    private final long receivedNs;
    private volatile long executeStartNs = -1L;
    private volatile long executeEndNs = -1L;

    private RequestTiming(long receivedNs) {
      this.receivedNs = receivedNs;
    }
  }
}
