package org.curtinfrc.frc2026.util.Repulsor.Offload;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetSocketAddress;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardOpenOption;
import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.concurrent.TimeUnit;
import org.curtinfrc.frc2026.util.Repulsor.Offload.Client.OffloadClientConfig;
import org.curtinfrc.frc2026.util.Repulsor.Offload.Client.OffloadHost;
import org.curtinfrc.frc2026.util.Repulsor.Offload.Client.OffloadServerTiming;
import org.curtinfrc.frc2026.util.Repulsor.Offload.Client.UdpOffloadClient;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.PredictiveFieldStateLocalAccess;
import org.curtinfrc.frc2026.util.Repulsor.Shooting.DragShotPlanner;
import org.curtinfrc.frc2026.util.Repulsor.Shooting.Constraints;
import org.curtinfrc.frc2026.util.Repulsor.Shooting.GamePiecePhysics;
import org.curtinfrc.frc2026.util.Repulsor.Shooting.ShotSolution;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.FieldTrackerLocalAccess;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.condition.EnabledIfSystemProperty;

class RealOffloadServerDragShotTest {
  private static final String ENABLED_PROPERTY = "repulsor.offload.real.enabled";
  private static final String HOST_PROPERTY = "repulsor.offload.real.host";
  private static final String DEFAULT_HOST = "10.47.88.11";
  private static final int DEFAULT_PORT = 5808;
  private static final int RPC_TIMEOUT_MS = 1500;
  private static final int CLIENT_READ_TIMEOUT_MS =
      Integer.getInteger("repulsor.offload.real.readTimeoutMs", 2);
  private static final int LATENCY_WARMUP_SAMPLES =
      Integer.getInteger("repulsor.offload.real.warmupSamples", 5);
  private static final int LATENCY_MEASURED_SAMPLES =
      Integer.getInteger("repulsor.offload.real.samplesPerTask", 80);
  private static final String METRICS_FILE_PROPERTY = "repulsor.offload.real.metricsFile";
  private static final String DEFAULT_METRICS_FILE = "build/offload/real-offload-latency.json";
  private static final String OBSTACLE_LIST_TYPE =
      "java.util.List<? extends org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacle>";
  private static final String DYNAMIC_OBJECT_LIST_TYPE =
      "java.util.List<org.curtinfrc.frc2026.util.Repulsor.Offload.ShuttleRecoveryDynamicObjectDTO>";

  @Test
  @EnabledIfSystemProperty(named = ENABLED_PROPERTY, matches = "(?i)true|1|yes|on")
  void shouldRunAllCurrentOffloadedFunctionsAgainstRealOffloadServer() throws Exception {
    OffloadRpc.clearGateway();
    OffloadHost host = parseHost();
    Path metricsFile = resolveMetricsFile();
    String udpProbe = probeUdp(host, 2000);
    assertTrue(
        "reachable".equals(udpProbe),
        "UDP probe failed for "
            + host.host()
            + ":"
            + host.port()
            + " ("
              + udpProbe
              + "). Check routing/firewall/server bind address.");

    LatencyRecorder latencyRecorder =
        new LatencyRecorder(
            host.host(),
            host.port(),
            CLIENT_READ_TIMEOUT_MS,
            LATENCY_WARMUP_SAMPLES,
            LATENCY_MEASURED_SAMPLES,
            RPC_TIMEOUT_MS,
            metricsFile);
    System.out.printf(
        "offload-rtt config host=%s:%d readTimeoutMs=%d warmup=%d samples=%d rpcTimeoutMs=%d metricsFile=%s%n",
        host.host(),
        host.port(),
        CLIENT_READ_TIMEOUT_MS,
        LATENCY_WARMUP_SAMPLES,
        LATENCY_MEASURED_SAMPLES,
        RPC_TIMEOUT_MS,
        metricsFile.toAbsolutePath());

    try (UdpOffloadClient client =
        new UdpOffloadClient(
            OffloadClientConfig.builder()
                .hosts(List.of(host))
                .connectTimeoutMs(2500)
                .probeTimeoutMs(2500)
                .readTimeoutMs(CLIENT_READ_TIMEOUT_MS)
                .probeBeforeConnect(true)
                .queueCapacity(64)
                .build())) {
      OffloadRpc.setGateway(client);
      client.start();
      waitForHealthy(client, 12000L);

      assertTrue(
          client.isHealthy(),
          "Offload client did not become healthy for "
              + host.host()
              + ":"
              + host.port()
              + " (UDP probe: "
              + udpProbe
              + "). Possible protocol mismatch or handshake failure.");

      runSampleDoubleValueRpc(latencyRecorder, client);
      runSampleWorkerProbeRpc(latencyRecorder, client);
      runDragShotStaticRpc(latencyRecorder, client);
      runDragShotFindBestAutoRpc(latencyRecorder, client);
      runPredictiveShuttleRecoveryRpc(latencyRecorder, client);
      runFieldTrackerRecoveryGoalRpc(latencyRecorder, client);
    } finally {
      OffloadRpc.clearGateway();
      latencyRecorder.printSummary();
    }
  }

  private static void runSampleDoubleValueRpc(LatencyRecorder latencyRecorder, UdpOffloadClient client)
      throws Exception {
    SampleMathOffloadEntrypoints_doubleValue_OffloadRequest request =
        new SampleMathOffloadEntrypoints_doubleValue_OffloadRequest();
    request.setArg0(OffloadValueCodec.encode("int", 7));

    SampleMathOffloadEntrypoints_doubleValue_OffloadResponse response =
        warmupThenMeasure(
            OffloadTaskIds.SAMPLE_DOUBLE_VALUE,
            latencyRecorder,
            client,
            () ->
                OffloadRpc.callTyped(
                        OffloadTaskIds.SAMPLE_DOUBLE_VALUE,
                        request,
                        SampleMathOffloadEntrypoints_doubleValue_OffloadResponse.class,
                        RPC_TIMEOUT_MS)
                    .get(4, TimeUnit.SECONDS));

    int remote = (int) OffloadValueCodec.decode("int", response.getResult());
    assertEquals(14, remote, "sample doubleValue remote result");
  }

  private static void runSampleWorkerProbeRpc(
      LatencyRecorder latencyRecorder, UdpOffloadClient client) throws Exception {
    SampleMathOffloadEntrypoints_runsOnOffloadWorkerThread_OffloadRequest request =
        new SampleMathOffloadEntrypoints_runsOnOffloadWorkerThread_OffloadRequest();
    request.setArg0(OffloadValueCodec.encode("int", 1));

    SampleMathOffloadEntrypoints_runsOnOffloadWorkerThread_OffloadResponse response =
        warmupThenMeasure(
            OffloadTaskIds.SAMPLE_WORKER_THREAD_PROBE,
            latencyRecorder,
            client,
            () ->
                OffloadRpc.callTyped(
                        OffloadTaskIds.SAMPLE_WORKER_THREAD_PROBE,
                        request,
                        SampleMathOffloadEntrypoints_runsOnOffloadWorkerThread_OffloadResponse
                            .class,
                        RPC_TIMEOUT_MS)
                    .get(4, TimeUnit.SECONDS));

    boolean remote = (boolean) OffloadValueCodec.decode("boolean", response.getResult());
    assertTrue(remote, "Expected worker-thread probe to execute remotely");
  }

  @SuppressWarnings("unchecked")
  private static void runDragShotStaticRpc(LatencyRecorder latencyRecorder, UdpOffloadClient client)
      throws Exception {
    GamePiecePhysics gamePiece = new TestGamePiecePhysics(0.25, 0.018, 0.47, 1.225);
    Translation2d shooterFieldPosition = new Translation2d(4.0, 3.5);
    Translation2d targetFieldPosition = new Translation2d(8.0, 4.0);
    double targetHeightMeters = 2.2;
    double shooterReleaseHeightMeters = 0.95;
    Constraints constraints = new Constraints(8.0, 42.0, 15.0, 80.0);

    DragShotPlannerOffloadEntrypoints_calculateStaticShotAngleAndSpeed_OffloadRequest request =
        new DragShotPlannerOffloadEntrypoints_calculateStaticShotAngleAndSpeed_OffloadRequest();
    request.setArg0(
        OffloadValueCodec.encode(
            "org.curtinfrc.frc2026.util.Repulsor.Shooting.GamePiecePhysics", gamePiece));
    request.setArg1(
        OffloadValueCodec.encode("edu.wpi.first.math.geometry.Translation2d", shooterFieldPosition));
    request.setArg2(
        OffloadValueCodec.encode("edu.wpi.first.math.geometry.Translation2d", targetFieldPosition));
    request.setArg3(OffloadValueCodec.encode("double", targetHeightMeters));
    request.setArg4(OffloadValueCodec.encode("double", shooterReleaseHeightMeters));
    request.setArg5(
        OffloadValueCodec.encode("org.curtinfrc.frc2026.util.Repulsor.Shooting.Constraints", constraints));

    DragShotPlannerOffloadEntrypoints_calculateStaticShotAngleAndSpeed_OffloadResponse response =
        warmupThenMeasure(
            OffloadTaskIds.DRAG_SHOT_CALC_STATIC_SHOT_ANGLE_SPEED,
            latencyRecorder,
            client,
            () ->
                OffloadRpc.callTyped(
                        OffloadTaskIds.DRAG_SHOT_CALC_STATIC_SHOT_ANGLE_SPEED,
                        request,
                        DragShotPlannerOffloadEntrypoints_calculateStaticShotAngleAndSpeed_OffloadResponse
                            .class,
                        RPC_TIMEOUT_MS)
                    .get(4, TimeUnit.SECONDS));

    Optional<ShotSolution> remote =
        (Optional<ShotSolution>)
            OffloadValueCodec.decode(
                "java.util.Optional<org.curtinfrc.frc2026.util.Repulsor.Shooting.ShotSolution>",
                response.getResult());

    Optional<ShotSolution> local =
        DragShotPlanner.calculateStaticShotAngleAndSpeedLocal(
            gamePiece,
            shooterFieldPosition,
            targetFieldPosition,
            targetHeightMeters,
            shooterReleaseHeightMeters,
            constraints);

    assertShotEquals(remote, local, "dragshot static RPC vs local");

    Optional<ShotSolution> offloadedApi =
        DragShotPlanner.calculateStaticShotAngleAndSpeed(
            gamePiece,
            shooterFieldPosition,
            targetFieldPosition,
            targetHeightMeters,
            shooterReleaseHeightMeters,
            constraints);
    assertShotEquals(remote, offloadedApi, "dragshot static offloaded API vs direct RPC");
  }

  @SuppressWarnings("unchecked")
  private static void runDragShotFindBestAutoRpc(
      LatencyRecorder latencyRecorder, UdpOffloadClient client) throws Exception {
    GamePiecePhysics gamePiece = new TestGamePiecePhysics(0.25, 0.018, 0.47, 1.225);
    Translation2d targetFieldPosition = new Translation2d(8.0, 4.0);
    double targetHeightMeters = 2.2;
    Pose2d robotPose = new Pose2d(4.2, 3.3, Rotation2d.fromDegrees(18.0));
    double shooterReleaseHeightMeters = 0.95;
    double robotHalfLengthMeters = 0.45;
    double robotHalfWidthMeters = 0.40;
    List<org.curtinfrc.frc2026.util.Repulsor.FieldPlanner.Obstacle> dynamicObstacles = List.of();
    Constraints constraints = new Constraints(8.0, 42.0, 15.0, 80.0);

    DragShotPlannerOffloadEntrypoints_findBestShotAuto_OffloadRequest request =
        new DragShotPlannerOffloadEntrypoints_findBestShotAuto_OffloadRequest();
    request.setArg0(
        OffloadValueCodec.encode(
            "org.curtinfrc.frc2026.util.Repulsor.Shooting.GamePiecePhysics", gamePiece));
    request.setArg1(
        OffloadValueCodec.encode("edu.wpi.first.math.geometry.Translation2d", targetFieldPosition));
    request.setArg2(OffloadValueCodec.encode("double", targetHeightMeters));
    request.setArg3(OffloadValueCodec.encode("edu.wpi.first.math.geometry.Pose2d", robotPose));
    request.setArg4(OffloadValueCodec.encode("double", shooterReleaseHeightMeters));
    request.setArg5(OffloadValueCodec.encode("double", robotHalfLengthMeters));
    request.setArg6(OffloadValueCodec.encode("double", robotHalfWidthMeters));
    request.setArg7(OffloadValueCodec.encode(OBSTACLE_LIST_TYPE, dynamicObstacles));
    request.setArg8(
        OffloadValueCodec.encode("org.curtinfrc.frc2026.util.Repulsor.Shooting.Constraints", constraints));

    DragShotPlannerOffloadEntrypoints_findBestShotAuto_OffloadResponse response =
        warmupThenMeasure(
            OffloadTaskIds.DRAG_SHOT_FIND_BEST_SHOT_AUTO,
            latencyRecorder,
            client,
            () ->
                OffloadRpc.callTyped(
                        OffloadTaskIds.DRAG_SHOT_FIND_BEST_SHOT_AUTO,
                        request,
                        DragShotPlannerOffloadEntrypoints_findBestShotAuto_OffloadResponse.class,
                        RPC_TIMEOUT_MS)
                    .get(4, TimeUnit.SECONDS));

    Optional<ShotSolution> remote =
        (Optional<ShotSolution>)
            OffloadValueCodec.decode(
                "java.util.Optional<org.curtinfrc.frc2026.util.Repulsor.Shooting.ShotSolution>",
                response.getResult());

    Optional<ShotSolution> local =
        DragShotPlanner.findBestShotAutoLocal(
            gamePiece,
            targetFieldPosition,
            targetHeightMeters,
            robotPose,
            shooterReleaseHeightMeters,
            robotHalfLengthMeters,
            robotHalfWidthMeters,
            dynamicObstacles,
            constraints);
    assertShotEquals(remote, local, "dragshot auto RPC vs local");
  }

  private static void runPredictiveShuttleRecoveryRpc(
      LatencyRecorder latencyRecorder, UdpOffloadClient client)
      throws Exception {
    Pose2d robotPose = new Pose2d(2.2, 3.4, Rotation2d.fromDegrees(25.0));
    double ourSpeedCap = 2.8;
    int goalUnits = 1;
    boolean flipRedToBlue = false;
    List<ShuttleRecoveryDynamicObjectDTO> dynamicObjects = sampleDynamicObjects();

    PredictiveFieldStateOffloadEntrypoints_selectShuttleRecoveryPoint_OffloadRequest request =
        new PredictiveFieldStateOffloadEntrypoints_selectShuttleRecoveryPoint_OffloadRequest();
    request.setArg0(OffloadValueCodec.encode("edu.wpi.first.math.geometry.Pose2d", robotPose));
    request.setArg1(OffloadValueCodec.encode("double", ourSpeedCap));
    request.setArg2(OffloadValueCodec.encode("int", goalUnits));
    request.setArg3(OffloadValueCodec.encode("boolean", flipRedToBlue));
    request.setArg4(OffloadValueCodec.encode(DYNAMIC_OBJECT_LIST_TYPE, dynamicObjects));

    PredictiveFieldStateOffloadEntrypoints_selectShuttleRecoveryPoint_OffloadResponse response =
        warmupThenMeasure(
            OffloadTaskIds.PREDICTIVE_SELECT_SHUTTLE_RECOVERY_POINT,
            latencyRecorder,
            client,
            () ->
                OffloadRpc.callTyped(
                        OffloadTaskIds.PREDICTIVE_SELECT_SHUTTLE_RECOVERY_POINT,
                        request,
                        PredictiveFieldStateOffloadEntrypoints_selectShuttleRecoveryPoint_OffloadResponse
                            .class,
                        RPC_TIMEOUT_MS)
                    .get(4, TimeUnit.SECONDS));

    ShuttleRecoveryPointDTO remote =
        (ShuttleRecoveryPointDTO)
            OffloadValueCodec.decode(
                "org.curtinfrc.frc2026.util.Repulsor.Offload.ShuttleRecoveryPointDTO",
                response.getResult());
    ShuttleRecoveryPointDTO local =
        PredictiveFieldStateLocalAccess.selectShuttleRecoveryPointLocal(
            robotPose, ourSpeedCap, goalUnits, flipRedToBlue, dynamicObjects);
    assertRecoveryPointEquals(remote, local, "predictive shuttle recovery RPC vs local");
  }

  private static void runFieldTrackerRecoveryGoalRpc(
      LatencyRecorder latencyRecorder, UdpOffloadClient client)
      throws Exception {
    Pose2d robotPose = new Pose2d(2.2, 3.4, Rotation2d.fromDegrees(25.0));
    double ourSpeedCap = 2.8;
    int goalUnits = 1;
    boolean flipRedToBlue = false;
    List<ShuttleRecoveryDynamicObjectDTO> dynamicObjects = sampleDynamicObjects();

    FieldTrackerOffloadEntrypoints_nextShuttleRecoveryGoalBlue_OffloadRequest request =
        new FieldTrackerOffloadEntrypoints_nextShuttleRecoveryGoalBlue_OffloadRequest();
    request.setArg0(OffloadValueCodec.encode("edu.wpi.first.math.geometry.Pose2d", robotPose));
    request.setArg1(OffloadValueCodec.encode("double", ourSpeedCap));
    request.setArg2(OffloadValueCodec.encode("int", goalUnits));
    request.setArg3(OffloadValueCodec.encode("boolean", flipRedToBlue));
    request.setArg4(OffloadValueCodec.encode(DYNAMIC_OBJECT_LIST_TYPE, dynamicObjects));

    FieldTrackerOffloadEntrypoints_nextShuttleRecoveryGoalBlue_OffloadResponse response =
        warmupThenMeasure(
            OffloadTaskIds.FIELD_TRACKER_NEXT_SHUTTLE_RECOVERY_GOAL_BLUE,
            latencyRecorder,
            client,
            () ->
                OffloadRpc.callTyped(
                        OffloadTaskIds.FIELD_TRACKER_NEXT_SHUTTLE_RECOVERY_GOAL_BLUE,
                        request,
                        FieldTrackerOffloadEntrypoints_nextShuttleRecoveryGoalBlue_OffloadResponse
                            .class,
                        RPC_TIMEOUT_MS)
                    .get(4, TimeUnit.SECONDS));

    Pose2d remote =
        (Pose2d)
            OffloadValueCodec.decode(
                "edu.wpi.first.math.geometry.Pose2d",
                response.getResult());
    Pose2d local =
        FieldTrackerLocalAccess.nextAllianceShuttleRecoveryGoalBlueLocal(
            robotPose, ourSpeedCap, goalUnits, flipRedToBlue, dynamicObjects);
    assertPoseEquals(remote, local, "field tracker recovery RPC vs local");
  }

  private static List<ShuttleRecoveryDynamicObjectDTO> sampleDynamicObjects() {
    ShuttleRecoveryDynamicObjectDTO fuel = new ShuttleRecoveryDynamicObjectDTO();
    fuel.setId("fuel-1");
    fuel.setType("fuel");
    fuel.setX(2.0);
    fuel.setY(3.6);
    fuel.setVx(0.0);
    fuel.setVy(0.0);
    fuel.setAgeS(0.02);

    ShuttleRecoveryDynamicObjectDTO blocker = new ShuttleRecoveryDynamicObjectDTO();
    blocker.setId("bot-1");
    blocker.setType("robot");
    blocker.setX(4.5);
    blocker.setY(2.7);
    blocker.setVx(-0.1);
    blocker.setVy(0.0);
    blocker.setAgeS(0.03);

    return List.of(fuel, blocker);
  }

  private static void assertRecoveryPointEquals(
      ShuttleRecoveryPointDTO expected, ShuttleRecoveryPointDTO actual, String context) {
    assertEquals(expected.isFound(), actual.isFound(), context + " found flag");
    if (!expected.isFound()) {
      return;
    }
    assertEquals(expected.getX(), actual.getX(), 1e-6, context + " x");
    assertEquals(expected.getY(), actual.getY(), 1e-6, context + " y");
    assertEquals(expected.getYawDeg(), actual.getYawDeg(), 1e-5, context + " yaw");
    assertEquals(expected.getScore(), actual.getScore(), 1e-4, context + " score");
  }

  private static void assertPoseEquals(Pose2d expected, Pose2d actual, String context) {
    assertEquals(expected.getX(), actual.getX(), 1e-6, context + " x");
    assertEquals(expected.getY(), actual.getY(), 1e-6, context + " y");
    assertEquals(
        expected.getRotation().getRadians(),
        actual.getRotation().getRadians(),
        1e-6,
        context + " yaw");
  }

  @FunctionalInterface
  private interface ThrowingSupplier<T> {
    T get() throws Exception;
  }

  private static <T> T warmupThenMeasure(
      String taskId,
      LatencyRecorder latencyRecorder,
      UdpOffloadClient client,
      ThrowingSupplier<T> call)
      throws Exception {
    for (int i = 0; i < Math.max(0, LATENCY_WARMUP_SAMPLES); i++) {
      call.get();
    }
    T last = null;
    for (int i = 0; i < Math.max(1, LATENCY_MEASURED_SAMPLES); i++) {
      last = latencyRecorder.measure(taskId, call, client);
    }
    return last;
  }

  private static final class LatencyRecorder {
    private static final ObjectMapper JSON = new ObjectMapper();

    private final String host;
    private final int port;
    private final int readTimeoutMs;
    private final int warmupSamples;
    private final int measuredSamples;
    private final int rpcTimeoutMs;
    private final Path metricsFile;

    private final Map<String, List<Long>> byTaskNs = new LinkedHashMap<>();
    private final Map<String, List<Long>> byTaskServerNs = new LinkedHashMap<>();
    private final Map<String, List<Long>> byTaskExecNs = new LinkedHashMap<>();
    private final Map<String, List<Long>> byTaskOverheadNs = new LinkedHashMap<>();
    private final Map<String, List<SamplePoint>> byTaskSamples = new LinkedHashMap<>();

    private int globalSampleIndex = 0;

    private LatencyRecorder(
        String host,
        int port,
        int readTimeoutMs,
        int warmupSamples,
        int measuredSamples,
        int rpcTimeoutMs,
        Path metricsFile) {
      this.host = host;
      this.port = port;
      this.readTimeoutMs = readTimeoutMs;
      this.warmupSamples = warmupSamples;
      this.measuredSamples = measuredSamples;
      this.rpcTimeoutMs = rpcTimeoutMs;
      this.metricsFile = metricsFile;
    }

    <T> T measure(String taskId, ThrowingSupplier<T> call, UdpOffloadClient client) throws Exception {
      long startNs = System.nanoTime();
      T value = call.get();
      long elapsedNs = System.nanoTime() - startNs;
      globalSampleIndex++;
      byTaskNs.computeIfAbsent(taskId, ignored -> new ArrayList<>()).add(elapsedNs);
      Optional<OffloadServerTiming> timingOpt = client.latestServerTiming(taskId);
      if (timingOpt.isPresent()) {
        OffloadServerTiming timing = timingOpt.get();
        long serverNs = timing.serverNs();
        long execNs = timing.executeNs();
        long overheadNs = Math.max(0L, elapsedNs - serverNs);
        byTaskServerNs.computeIfAbsent(taskId, ignored -> new ArrayList<>()).add(serverNs);
        byTaskExecNs.computeIfAbsent(taskId, ignored -> new ArrayList<>()).add(execNs);
        byTaskOverheadNs.computeIfAbsent(taskId, ignored -> new ArrayList<>()).add(overheadNs);
        byTaskSamples
            .computeIfAbsent(taskId, ignored -> new ArrayList<>())
            .add(
                new SamplePoint(
                    globalSampleIndex, elapsedNs, serverNs, execNs, overheadNs, timing.queueNs()));
        System.out.printf(
            "offload-rtt task=%s rttMs=%.3f serverMs=%.3f execMs=%.3f overheadMs=%.3f%n",
            taskId,
            elapsedNs / 1_000_000.0,
            serverNs / 1_000_000.0,
            execNs / 1_000_000.0,
            overheadNs / 1_000_000.0);
      } else {
        byTaskSamples
            .computeIfAbsent(taskId, ignored -> new ArrayList<>())
            .add(new SamplePoint(globalSampleIndex, elapsedNs, -1L, -1L, -1L, -1L));
        System.out.printf("offload-rtt task=%s rttMs=%.3f%n", taskId, elapsedNs / 1_000_000.0);
      }
      return value;
    }

    void printSummary() {
      if (byTaskNs.isEmpty()) {
        return;
      }
      System.out.println("offload-rtt summary:");
      byTaskNs.forEach(
          (taskId, samples) -> {
            Stats rtt = statsMs(samples);
            List<Long> server = byTaskServerNs.get(taskId);
            List<Long> exec = byTaskExecNs.get(taskId);
            List<Long> overhead = byTaskOverheadNs.get(taskId);
            if (server == null || exec == null || overhead == null) {
              System.out.printf(
                  "  task=%s samples=%d minMs=%.3f p50Ms=%.3f avgMs=%.3f p95Ms=%.3f maxMs=%.3f%n",
                  taskId,
                  samples.size(),
                  rtt.minMs,
                  rtt.p50Ms,
                  rtt.avgMs,
                  rtt.p95Ms,
                  rtt.maxMs);
              return;
            }

            Stats serverStats = statsMs(server);
            Stats execStats = statsMs(exec);
            Stats overheadStats = statsMs(overhead);
            System.out.printf(
                "  task=%s samples=%d rtt[min/p50/avg/p95/max]=%.3f/%.3f/%.3f/%.3f/%.3f server[min/p50/avg/p95/max]=%.3f/%.3f/%.3f/%.3f/%.3f exec[min/p50/avg/p95/max]=%.3f/%.3f/%.3f/%.3f/%.3f overhead[min/p50/avg/p95/max]=%.3f/%.3f/%.3f/%.3f/%.3f%n",
                taskId,
                samples.size(),
                rtt.minMs,
                rtt.p50Ms,
                rtt.avgMs,
                rtt.p95Ms,
                rtt.maxMs,
                serverStats.minMs,
                serverStats.p50Ms,
                serverStats.avgMs,
                serverStats.p95Ms,
                serverStats.maxMs,
                execStats.minMs,
                execStats.p50Ms,
                execStats.avgMs,
                execStats.p95Ms,
                execStats.maxMs,
                overheadStats.minMs,
                overheadStats.p50Ms,
                overheadStats.avgMs,
                overheadStats.p95Ms,
                overheadStats.maxMs);
          });
      writeMetricsFile();
    }

    private static Stats statsMs(List<Long> samplesNs) {
      double minMs = Double.POSITIVE_INFINITY;
      double maxMs = 0.0;
      double sumMs = 0.0;
      List<Double> sortedMs = new ArrayList<>(samplesNs.size());
      for (long ns : samplesNs) {
        double ms = ns / 1_000_000.0;
        minMs = Math.min(minMs, ms);
        maxMs = Math.max(maxMs, ms);
        sumMs += ms;
        sortedMs.add(ms);
      }
      Collections.sort(sortedMs);
      double avgMs = sumMs / samplesNs.size();
      double p50Ms = percentile(sortedMs, 0.50);
      double p95Ms = percentile(sortedMs, 0.95);
      return new Stats(minMs, p50Ms, avgMs, p95Ms, maxMs);
    }

    private static double percentile(List<Double> sorted, double q) {
      if (sorted.isEmpty()) {
        return 0.0;
      }
      if (sorted.size() == 1) {
        return sorted.get(0);
      }
      double clamped = Math.max(0.0, Math.min(1.0, q));
      int index = (int) Math.ceil(clamped * sorted.size()) - 1;
      index = Math.max(0, Math.min(sorted.size() - 1, index));
      return sorted.get(index);
    }

    private record Stats(double minMs, double p50Ms, double avgMs, double p95Ms, double maxMs) {}

    private record SamplePoint(
        int sampleIndex, long rttNs, long serverNs, long execNs, long overheadNs, long queueNs) {}

    private void writeMetricsFile() {
      try {
        if (metricsFile.getParent() != null) {
          Files.createDirectories(metricsFile.getParent());
        }

        Map<String, Object> root = new LinkedHashMap<>();
        root.put("host", host);
        root.put("port", port);
        root.put("readTimeoutMs", readTimeoutMs);
        root.put("warmupSamples", warmupSamples);
        root.put("measuredSamples", measuredSamples);
        root.put("rpcTimeoutMs", rpcTimeoutMs);
        root.put("metricsFile", metricsFile.toAbsolutePath().toString());

        Map<String, Object> tasksOut = new LinkedHashMap<>();
        for (Map.Entry<String, List<SamplePoint>> entry : byTaskSamples.entrySet()) {
          String taskId = entry.getKey();
          List<SamplePoint> samples = entry.getValue();
          Map<String, Object> taskOut = new LinkedHashMap<>();
          taskOut.put("samples", samplesToMaps(samples));
          taskOut.put("summary", summaryForTask(taskId));
          tasksOut.put(taskId, taskOut);
        }
        root.put("tasks", tasksOut);

        String json = JSON.writerWithDefaultPrettyPrinter().writeValueAsString(root);
        Files.writeString(
            metricsFile,
            json,
            StandardOpenOption.CREATE,
            StandardOpenOption.TRUNCATE_EXISTING,
            StandardOpenOption.WRITE);
        System.out.printf("offload-rtt wrote metrics file: %s%n", metricsFile.toAbsolutePath());
      } catch (Exception ex) {
        System.err.printf("offload-rtt failed to write metrics file: %s (%s)%n", metricsFile, ex);
      }
    }

    private static List<Map<String, Object>> samplesToMaps(List<SamplePoint> samples) {
      List<Map<String, Object>> out = new ArrayList<>(samples.size());
      for (SamplePoint sample : samples) {
        Map<String, Object> row = new LinkedHashMap<>();
        row.put("sampleIndex", sample.sampleIndex());
        row.put("rttMs", sample.rttNs() / 1_000_000.0);
        row.put("serverMs", nsToMsOrNull(sample.serverNs()));
        row.put("execMs", nsToMsOrNull(sample.execNs()));
        row.put("overheadMs", nsToMsOrNull(sample.overheadNs()));
        row.put("queueMs", nsToMsOrNull(sample.queueNs()));
        out.add(row);
      }
      return out;
    }

    private Map<String, Object> summaryForTask(String taskId) {
      Map<String, Object> out = new LinkedHashMap<>();
      out.put("rtt", statsMap(statsMs(byTaskNs.get(taskId))));
      List<Long> server = byTaskServerNs.get(taskId);
      List<Long> exec = byTaskExecNs.get(taskId);
      List<Long> overhead = byTaskOverheadNs.get(taskId);
      if (server != null) {
        out.put("server", statsMap(statsMs(server)));
      }
      if (exec != null) {
        out.put("exec", statsMap(statsMs(exec)));
      }
      if (overhead != null) {
        out.put("overhead", statsMap(statsMs(overhead)));
      }
      return out;
    }

    private static Map<String, Object> statsMap(Stats stats) {
      Map<String, Object> out = new LinkedHashMap<>();
      out.put("minMs", stats.minMs());
      out.put("p50Ms", stats.p50Ms());
      out.put("avgMs", stats.avgMs());
      out.put("p95Ms", stats.p95Ms());
      out.put("maxMs", stats.maxMs());
      return out;
    }

    private static Double nsToMsOrNull(long ns) {
      return ns >= 0 ? ns / 1_000_000.0 : null;
    }
  }

  private static OffloadHost parseHost() {
    String configured =
        System.getProperty(HOST_PROPERTY, DEFAULT_HOST + ":" + DEFAULT_PORT).trim();
    if (configured.isEmpty()) {
      throw new IllegalArgumentException(HOST_PROPERTY + " must be non-empty.");
    }

    String host = configured;
    int port = DEFAULT_PORT;
    int separator = configured.lastIndexOf(':');
    if (separator >= 0) {
      host = configured.substring(0, separator).trim();
      String rawPort = configured.substring(separator + 1).trim();
      if (host.isEmpty() || rawPort.isEmpty()) {
        throw new IllegalArgumentException(
            "Expected host:port format for " + HOST_PROPERTY + ", got: " + configured);
      }
      port = Integer.parseInt(rawPort);
    }

    if (port < 1 || port > 65535) {
      throw new IllegalArgumentException("Port out of range in " + HOST_PROPERTY + ": " + port);
    }
    return new OffloadHost(host, port);
  }

  private static Path resolveMetricsFile() {
    String configured = System.getProperty(METRICS_FILE_PROPERTY, DEFAULT_METRICS_FILE).trim();
    Path path = Path.of(configured);
    if (!path.isAbsolute()) {
      path = Path.of(System.getProperty("user.dir")).resolve(path);
    }
    return path.normalize();
  }

  private static void waitForHealthy(UdpOffloadClient client, long timeoutMs)
      throws InterruptedException {
    long deadline = System.currentTimeMillis() + timeoutMs;
    while (!client.isHealthy() && System.currentTimeMillis() < deadline) {
      Thread.sleep(20L);
    }
  }

  private static String probeUdp(OffloadHost host, int timeoutMs) {
    try (DatagramSocket socket = new DatagramSocket()) {
      socket.connect(new InetSocketAddress(host.host(), host.port()));
      socket.setSoTimeout(timeoutMs);

      long correlationId = System.nanoTime();
      byte[] request =
          OffloadProtocol.serializeRequest(correlationId, OffloadProtocol.TASK_PING, new byte[0]);
      socket.send(new DatagramPacket(request, request.length));

      byte[] receive = new byte[4096];
      DatagramPacket responsePacket = new DatagramPacket(receive, receive.length);
      socket.receive(responsePacket);

      byte[] frame = new byte[responsePacket.getLength()];
      System.arraycopy(
          responsePacket.getData(), responsePacket.getOffset(), frame, 0, responsePacket.getLength());
      OffloadProtocol.ResponseFrame response = OffloadProtocol.parseResponse(frame);
      if (response.correlationId() != correlationId) {
        return "unexpected-correlation:" + response.correlationId();
      }
      if (response.status() != OffloadProtocol.STATUS_OK) {
        return "unexpected-status:" + response.status();
      }
      return "reachable";
    } catch (Exception ex) {
      return ex.getClass().getSimpleName() + ": " + ex.getMessage();
    }
  }

  private static void assertShotEquals(
      Optional<ShotSolution> expected, Optional<ShotSolution> actual, String context) {
    assertEquals(expected.isPresent(), actual.isPresent(), context + " present flag mismatch");
    if (expected.isEmpty() || actual.isEmpty()) {
      return;
    }

    ShotSolution expectedShot = expected.get();
    ShotSolution actualShot = actual.get();
    assertEquals(
        expectedShot.shooterPosition().getX(),
        actualShot.shooterPosition().getX(),
        1e-4,
        context + " shooter x");
    assertEquals(
        expectedShot.shooterPosition().getY(),
        actualShot.shooterPosition().getY(),
        1e-4,
        context + " shooter y");
    assertEquals(
        expectedShot.shooterYaw().getRadians(),
        actualShot.shooterYaw().getRadians(),
        1e-4,
        context + " shooter yaw");
    assertEquals(
        expectedShot.launchSpeedMetersPerSecond(),
        actualShot.launchSpeedMetersPerSecond(),
        1e-3,
        context + " launch speed");
    assertEquals(
        expectedShot.launchAngle().getRadians(),
        actualShot.launchAngle().getRadians(),
        1e-4,
        context + " launch angle");
    assertEquals(
        expectedShot.timeToPlaneSeconds(),
        actualShot.timeToPlaneSeconds(),
        1e-3,
        context + " time to plane");
    assertEquals(
        expectedShot.impactFieldPosition().getX(),
        actualShot.impactFieldPosition().getX(),
        1e-4,
        context + " impact x");
    assertEquals(
        expectedShot.impactFieldPosition().getY(),
        actualShot.impactFieldPosition().getY(),
        1e-4,
        context + " impact y");
    assertEquals(
        expectedShot.verticalErrorMeters(),
        actualShot.verticalErrorMeters(),
        1e-4,
        context + " vertical error");
  }

  private static final class TestGamePiecePhysics extends GamePiecePhysics {
    private final double massKg;
    private final double crossSectionAreaM2;
    private final double dragCoefficient;
    private final double airDensityKgPerM3;

    TestGamePiecePhysics(
        double massKg, double crossSectionAreaM2, double dragCoefficient, double airDensityKgPerM3) {
      this.massKg = massKg;
      this.crossSectionAreaM2 = crossSectionAreaM2;
      this.dragCoefficient = dragCoefficient;
      this.airDensityKgPerM3 = airDensityKgPerM3;
    }

    @Override
    public double massKg() {
      return massKg;
    }

    @Override
    public double crossSectionAreaM2() {
      return crossSectionAreaM2;
    }

    @Override
    public double dragCoefficient() {
      return dragCoefficient;
    }

    @Override
    public double airDensityKgPerM3() {
      return airDensityKgPerM3;
    }
  }
}
