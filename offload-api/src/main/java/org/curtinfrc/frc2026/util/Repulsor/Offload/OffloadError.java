package org.curtinfrc.frc2026.util.Repulsor.Offload;

public class OffloadError {
  private String code;
  private String message;

  public OffloadError() {}

  public OffloadError(String code, String message) {
    this.code = code;
    this.message = message;
  }

  public String getCode() {
    return code;
  }

  public String getMessage() {
    return message;
  }

  public void setCode(String code) {
    this.code = code;
  }

  public void setMessage(String message) {
    this.message = message;
  }
}
