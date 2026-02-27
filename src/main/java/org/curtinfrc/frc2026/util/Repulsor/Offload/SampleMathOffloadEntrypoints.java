package org.curtinfrc.frc2026.util.Repulsor.Offload;

@SuppressWarnings("unused")
public final class SampleMathOffloadEntrypoints {
  private SampleMathOffloadEntrypoints() {}

  @Offloadable(
      id = OffloadTaskIds.SAMPLE_DOUBLE_VALUE,
      version = 1,
      timeoutMs = 250,
      fallback = true)
  public static int doubleValue(int input) {
    return input * 2;
  }

  @Offloadable(
      id = OffloadTaskIds.SAMPLE_WORKER_THREAD_PROBE,
      version = 1,
      timeoutMs = 250,
      fallback = true)
  public static boolean runsOnOffloadWorkerThread(int marker) {
    return marker == 1 && Thread.currentThread().getName().startsWith("offload-server-worker");
  }
}
