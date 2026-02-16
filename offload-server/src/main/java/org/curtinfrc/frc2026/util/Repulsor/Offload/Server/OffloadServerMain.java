package org.curtinfrc.frc2026.util.Repulsor.Offload.Server;

public final class OffloadServerMain {
  private OffloadServerMain() {}

  public static void main(String[] args) throws Exception {
    OffloadServerConfig config = OffloadServerConfig.fromEnvironment();
    OffloadServer server = new OffloadServer(config);
    server.start();

    System.out.println(
        "Offload server listening on port "
            + config.port()
            + " with "
            + server.taskCount()
            + " loaded tasks");

    Runtime.getRuntime().addShutdownHook(new Thread(server::close, "offload-server-shutdown"));

    synchronized (OffloadServerMain.class) {
      OffloadServerMain.class.wait();
    }
  }
}
