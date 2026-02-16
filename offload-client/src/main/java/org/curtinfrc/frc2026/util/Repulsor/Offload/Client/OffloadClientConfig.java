package org.curtinfrc.frc2026.util.Repulsor.Offload.Client;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Objects;

public final class OffloadClientConfig {
  private final List<OffloadHost> hosts;
  private final int connectTimeoutMs;
  private final int readTimeoutMs;
  private final int queueCapacity;
  private final int probeTimeoutMs;
  private final boolean probeBeforeConnect;

  private OffloadClientConfig(
      List<OffloadHost> hosts,
      int connectTimeoutMs,
      int readTimeoutMs,
      int queueCapacity,
      int probeTimeoutMs,
      boolean probeBeforeConnect) {
    this.hosts = List.copyOf(hosts);
    this.connectTimeoutMs = connectTimeoutMs;
    this.readTimeoutMs = readTimeoutMs;
    this.queueCapacity = queueCapacity;
    this.probeTimeoutMs = probeTimeoutMs;
    this.probeBeforeConnect = probeBeforeConnect;
  }

  public List<OffloadHost> hosts() {
    return hosts;
  }

  public int connectTimeoutMs() {
    return connectTimeoutMs;
  }

  public int readTimeoutMs() {
    return readTimeoutMs;
  }

  public int queueCapacity() {
    return queueCapacity;
  }

  public int probeTimeoutMs() {
    return probeTimeoutMs;
  }

  public boolean probeBeforeConnect() {
    return probeBeforeConnect;
  }

  public static Builder builder() {
    return new Builder();
  }

  public static final class Builder {
    private final List<OffloadHost> hosts = new ArrayList<>();
    private int connectTimeoutMs = 60;
    private int readTimeoutMs = 5;
    private int queueCapacity = 128;
    private int probeTimeoutMs = 50;
    private boolean probeBeforeConnect = true;

    public Builder hosts(List<OffloadHost> values) {
      hosts.clear();
      hosts.addAll(Objects.requireNonNull(values));
      return this;
    }

    public Builder addHost(OffloadHost host) {
      hosts.add(Objects.requireNonNull(host));
      return this;
    }

    public Builder connectTimeoutMs(int value) {
      connectTimeoutMs = value;
      return this;
    }

    public Builder readTimeoutMs(int value) {
      readTimeoutMs = value;
      return this;
    }

    public Builder queueCapacity(int value) {
      queueCapacity = value;
      return this;
    }

    public Builder probeTimeoutMs(int value) {
      probeTimeoutMs = value;
      return this;
    }

    public Builder probeBeforeConnect(boolean value) {
      probeBeforeConnect = value;
      return this;
    }

    public OffloadClientConfig build() {
      if (hosts.isEmpty()) {
        return new OffloadClientConfig(
            Collections.singletonList(new OffloadHost("127.0.0.1", 5808)),
            connectTimeoutMs,
            readTimeoutMs,
            queueCapacity,
            probeTimeoutMs,
            probeBeforeConnect);
      }
      return new OffloadClientConfig(
          hosts,
          connectTimeoutMs,
          readTimeoutMs,
          queueCapacity,
          probeTimeoutMs,
          probeBeforeConnect);
    }
  }
}
