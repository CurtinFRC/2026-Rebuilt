package org.curtinfrc.frc2026.util.Repulsor.Offload;

import java.util.List;

public class OffloadHelloResponse {
  private String serverName;
  private String serverVersion;
  private String manifestHash;
  private List<String> supportedTaskIds;

  public OffloadHelloResponse() {}

  public OffloadHelloResponse(
      String serverName, String serverVersion, String manifestHash, List<String> supportedTaskIds) {
    this.serverName = serverName;
    this.serverVersion = serverVersion;
    this.manifestHash = manifestHash;
    this.supportedTaskIds = supportedTaskIds;
  }

  public String getServerName() {
    return serverName;
  }

  public String getServerVersion() {
    return serverVersion;
  }

  public String getManifestHash() {
    return manifestHash;
  }

  public List<String> getSupportedTaskIds() {
    return supportedTaskIds;
  }

  public void setServerName(String serverName) {
    this.serverName = serverName;
  }

  public void setServerVersion(String serverVersion) {
    this.serverVersion = serverVersion;
  }

  public void setManifestHash(String manifestHash) {
    this.manifestHash = manifestHash;
  }

  public void setSupportedTaskIds(List<String> supportedTaskIds) {
    this.supportedTaskIds = supportedTaskIds;
  }
}
