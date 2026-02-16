package org.curtinfrc.frc2026.util.Repulsor.Offload;

public interface OffloadFunction<RequestT, ResponseT> {
  String taskId();

  Class<RequestT> requestType();

  Class<ResponseT> responseType();

  default int timeoutMs() {
    return 30;
  }

  ResponseT execute(RequestT request) throws Exception;
}
