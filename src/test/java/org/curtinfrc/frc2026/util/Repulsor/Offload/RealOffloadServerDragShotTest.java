package org.curtinfrc.frc2026.util.Repulsor.Offload;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Translation2d;
import java.io.IOException;
import java.net.InetSocketAddress;
import java.net.Socket;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.TimeUnit;
import org.curtinfrc.frc2026.util.Repulsor.Offload.Client.OffloadClientConfig;
import org.curtinfrc.frc2026.util.Repulsor.Offload.Client.OffloadHost;
import org.curtinfrc.frc2026.util.Repulsor.Offload.Client.TcpOffloadClient;
import org.curtinfrc.frc2026.util.Repulsor.Shooting.Constraints;
import org.curtinfrc.frc2026.util.Repulsor.Shooting.DragShotPlanner;
import org.curtinfrc.frc2026.util.Repulsor.Shooting.GamePiecePhysics;
import org.curtinfrc.frc2026.util.Repulsor.Shooting.ShotSolution;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.condition.EnabledIfSystemProperty;

class RealOffloadServerDragShotTest {
  private static final String ENABLED_PROPERTY = "repulsor.offload.real.enabled";
  private static final String HOST_PROPERTY = "repulsor.offload.real.host";
  private static final String DEFAULT_HOST = "10.47.88.11";
  private static final int DEFAULT_PORT = 5808;

  @Test
  @EnabledIfSystemProperty(named = ENABLED_PROPERTY, matches = "(?i)true|1|yes|on")
  void shouldRunDragShotStaticSolveAgainstRealOffloadServer() throws Exception {
    OffloadRpc.clearGateway();
    OffloadHost host = parseHost();
    String tcpProbe = probeTcp(host, 2000);
    assertTrue(
        "reachable".equals(tcpProbe),
        "TCP probe failed for "
            + host.host()
            + ":"
            + host.port()
            + " ("
            + tcpProbe
            + "). Check routing/firewall/server bind address.");

    try (TcpOffloadClient client =
        new TcpOffloadClient(
            OffloadClientConfig.builder()
                .hosts(List.of(host))
                .connectTimeoutMs(2500)
                .probeTimeoutMs(2500)
                .readTimeoutMs(250)
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
              + " (TCP probe: "
              + tcpProbe
              + "). Possible protocol mismatch or handshake failure.");

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
          OffloadValueCodec.encode(
              "edu.wpi.first.math.geometry.Translation2d", shooterFieldPosition));
      request.setArg2(
          OffloadValueCodec.encode("edu.wpi.first.math.geometry.Translation2d", targetFieldPosition));
      request.setArg3(OffloadValueCodec.encode("double", targetHeightMeters));
      request.setArg4(OffloadValueCodec.encode("double", shooterReleaseHeightMeters));
      request.setArg5(
          OffloadValueCodec.encode(
              "org.curtinfrc.frc2026.util.Repulsor.Shooting.Constraints", constraints));

      DragShotPlannerOffloadEntrypoints_calculateStaticShotAngleAndSpeed_OffloadResponse response =
          OffloadRpc.callTyped(
                  OffloadTaskIds.DRAG_SHOT_CALC_STATIC_SHOT_ANGLE_SPEED,
                  request,
                  DragShotPlannerOffloadEntrypoints_calculateStaticShotAngleAndSpeed_OffloadResponse
                      .class,
                  500)
              .get(3, TimeUnit.SECONDS);

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

      assertShotEquals(remote, local, "direct offload RPC vs local");

      Optional<ShotSolution> offloadedApi =
          DragShotPlanner.calculateStaticShotAngleAndSpeed(
              gamePiece,
              shooterFieldPosition,
              targetFieldPosition,
              targetHeightMeters,
              shooterReleaseHeightMeters,
              constraints);
      assertShotEquals(remote, offloadedApi, "offloaded API vs direct offload RPC");
    } finally {
      OffloadRpc.clearGateway();
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

  private static void waitForHealthy(TcpOffloadClient client, long timeoutMs)
      throws InterruptedException {
    long deadline = System.currentTimeMillis() + timeoutMs;
    while (!client.isHealthy() && System.currentTimeMillis() < deadline) {
      Thread.sleep(20L);
    }
  }

  private static String probeTcp(OffloadHost host, int timeoutMs) {
    try (Socket socket = new Socket()) {
      socket.connect(new InetSocketAddress(host.host(), host.port()), timeoutMs);
      return "reachable";
    } catch (IOException ex) {
      return ex.getClass().getSimpleName() + ": " + ex.getMessage();
    }
  }

  private static void assertShotEquals(
      Optional<ShotSolution> expected, Optional<ShotSolution> actual, String context) {
    assertEquals(expected.isPresent(), actual.isPresent(), context + " present flag mismatch");
    if (expected.isEmpty()) {
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
