package org.curtinfrc.frc2026.util.Repulsor.Offload;

import com.fasterxml.jackson.databind.JavaType;
import java.util.Map;
import java.util.Objects;
import java.util.ServiceLoader;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Function;

public final class OffloadValueCodec {
  private static final AtomicBoolean INITIALIZED = new AtomicBoolean(false);
  private static final Map<String, Adapter<?, ?>> ADAPTERS = new ConcurrentHashMap<>();

  private OffloadValueCodec() {}

  public static <T, W> void registerAdapter(
      String typeName, Class<W> wireType, Function<T, W> toWire, Function<W, T> fromWire) {
    Objects.requireNonNull(typeName);
    Objects.requireNonNull(wireType);
    Objects.requireNonNull(toWire);
    Objects.requireNonNull(fromWire);
    ADAPTERS.put(typeName, new Adapter<>(wireType, toWire, fromWire));
  }

  public static byte[] encode(String typeName, Object value) {
    ensureInitialized();
    Adapter<?, ?> adapter = ADAPTERS.get(typeName);
    if (adapter != null) {
      return CborSerde.write(adapter.toWire(value));
    }
    return CborSerde.write(value);
  }

  public static Object decode(String typeName, byte[] payload) {
    ensureInitialized();
    Adapter<?, ?> adapter = ADAPTERS.get(typeName);
    if (adapter != null) {
      Object wireValue = CborSerde.read(payload, adapter.wireType());
      return adapter.fromWire(wireValue);
    }

    JavaType javaType = resolveType(typeName);
    try {
      return CborSerde.mapper().readValue(payload, javaType);
    } catch (Exception ex) {
      throw new IllegalStateException("Failed to decode type " + typeName, ex);
    }
  }

  private static void ensureInitialized() {
    if (!INITIALIZED.compareAndSet(false, true)) {
      return;
    }

    for (OffloadAdapterRegistrar registrar : ServiceLoader.load(OffloadAdapterRegistrar.class)) {
      registrar.registerAdapters();
    }
  }

  private static JavaType resolveType(String typeName) {
    try {
      return CborSerde.mapper().getTypeFactory().constructFromCanonical(typeName);
    } catch (IllegalArgumentException ex) {
      return CborSerde.mapper().getTypeFactory().constructType(rawClassFor(typeName));
    }
  }

  private static Class<?> rawClassFor(String typeName) {
    return switch (typeName) {
      case "byte" -> Byte.class;
      case "short" -> Short.class;
      case "int" -> Integer.class;
      case "long" -> Long.class;
      case "float" -> Float.class;
      case "double" -> Double.class;
      case "boolean" -> Boolean.class;
      case "char" -> Character.class;
      default -> {
        try {
          yield Class.forName(typeName);
        } catch (ClassNotFoundException ex) {
          throw new IllegalStateException("Cannot resolve type " + typeName, ex);
        }
      }
    };
  }

  private record Adapter<T, W>(Class<W> wireType, Function<T, W> toWire, Function<W, T> fromWire) {
    Object toWire(Object value) {
      return toWire.apply((T) value);
    }

    Object fromWire(Object wireValue) {
      return fromWire.apply((W) wireValue);
    }
  }
}
