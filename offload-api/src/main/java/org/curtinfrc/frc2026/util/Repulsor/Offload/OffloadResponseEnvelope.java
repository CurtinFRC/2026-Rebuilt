package org.curtinfrc.frc2026.util.Repulsor.Offload;

public final class OffloadResponseEnvelope {
  private byte[] payload;
  private long queueNs;
  private long executeNs;
  private long serverNs;

  public OffloadResponseEnvelope() {}

  public OffloadResponseEnvelope(byte[] payload, long queueNs, long executeNs, long serverNs) {
    this.payload = payload;
    this.queueNs = queueNs;
    this.executeNs = executeNs;
    this.serverNs = serverNs;
  }

  public byte[] getPayload() {
    return payload;
  }

  public void setPayload(byte[] payload) {
    this.payload = payload;
  }

  public long getQueueNs() {
    return queueNs;
  }

  public void setQueueNs(long queueNs) {
    this.queueNs = queueNs;
  }

  public long getExecuteNs() {
    return executeNs;
  }

  public void setExecuteNs(long executeNs) {
    this.executeNs = executeNs;
  }

  public long getServerNs() {
    return serverNs;
  }

  public void setServerNs(long serverNs) {
    this.serverNs = serverNs;
  }
}
