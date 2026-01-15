package org.curtinfrc.frc2026.util.Repulsor.Metrics;

import com.google.gson.Gson;
import java.lang.reflect.Type;
import java.util.Optional;

public final class MetricCodecs {
  private MetricCodecs() {}

  public static final class GsonCodec<T> implements MetricCodec<T> {
    private final Gson gson = new Gson();
    private final Type type;

    public GsonCodec(Type type) {
      this.type = type;
    }

    @Override
    public String encode(T value) {
      return value == null ? "" : gson.toJson(value, type);
    }

    @Override
    public Optional<T> decode(String raw) {
      if (raw == null || raw.isEmpty()) return Optional.empty();
      return Optional.ofNullable(gson.fromJson(raw, type));
    }
  }

  public static final class StringCodec implements MetricCodec<String> {
    @Override
    public String encode(String value) {
      return value == null ? "" : value;
    }
  }

  public static final class DoubleCodec implements MetricCodec<Double> {
    @Override
    public String encode(Double value) {
      return value == null ? "" : Double.toString(value);
    }
  }
}
