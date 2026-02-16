package org.curtinfrc.frc2026.util.Repulsor.Offload;

import java.util.ArrayList;
import java.util.List;

public class OffloadManifest {
  private int schemaVersion = 1;
  private List<FunctionEntry> functions = new ArrayList<>();

  public int getSchemaVersion() {
    return schemaVersion;
  }

  public List<FunctionEntry> getFunctions() {
    return functions;
  }

  public void setSchemaVersion(int schemaVersion) {
    this.schemaVersion = schemaVersion;
  }

  public void setFunctions(List<FunctionEntry> functions) {
    this.functions = functions;
  }

  public static class FunctionEntry {
    private String id;
    private String declaringClass;
    private String methodName;
    private String requestType;
    private String responseType;
    private int version;
    private int timeoutMs;
    private boolean fallback;

    public String getId() {
      return id;
    }

    public void setId(String id) {
      this.id = id;
    }

    public String getDeclaringClass() {
      return declaringClass;
    }

    public void setDeclaringClass(String declaringClass) {
      this.declaringClass = declaringClass;
    }

    public String getMethodName() {
      return methodName;
    }

    public void setMethodName(String methodName) {
      this.methodName = methodName;
    }

    public String getRequestType() {
      return requestType;
    }

    public void setRequestType(String requestType) {
      this.requestType = requestType;
    }

    public String getResponseType() {
      return responseType;
    }

    public void setResponseType(String responseType) {
      this.responseType = responseType;
    }

    public int getVersion() {
      return version;
    }

    public void setVersion(int version) {
      this.version = version;
    }

    public int getTimeoutMs() {
      return timeoutMs;
    }

    public void setTimeoutMs(int timeoutMs) {
      this.timeoutMs = timeoutMs;
    }

    public boolean isFallback() {
      return fallback;
    }

    public void setFallback(boolean fallback) {
      this.fallback = fallback;
    }
  }
}
