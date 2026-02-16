package org.curtinfrc.frc2026.util.Repulsor.Offload;

@SuppressWarnings("unused")
public final class SampleMathOffloadEntrypoints {
  private SampleMathOffloadEntrypoints() {}

  @Offloadable(id = "repulsor.sample.math.double.v1", version = 1, timeoutMs = 20, fallback = true)
  public static SampleMathResponseDTO doubleValue(SampleMathRequestDTO request) {
    int input = request == null ? 0 : request.getInput();
    return new SampleMathResponseDTO(input * 2);
  }
}
