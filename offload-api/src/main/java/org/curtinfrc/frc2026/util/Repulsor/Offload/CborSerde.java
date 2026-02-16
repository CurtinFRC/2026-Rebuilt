package org.curtinfrc.frc2026.util.Repulsor.Offload;

import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.dataformat.cbor.CBORFactory;
import com.fasterxml.jackson.datatype.jdk8.Jdk8Module;

public final class CborSerde {
  private static final ObjectMapper OBJECT_MAPPER =
      new ObjectMapper(new CBORFactory())
          .registerModule(new Jdk8Module())
          .configure(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES, false);

  private CborSerde() {}

  public static byte[] write(Object value) {
    try {
      return OBJECT_MAPPER.writeValueAsBytes(value);
    } catch (Exception ex) {
      throw new IllegalStateException("Failed to encode CBOR payload", ex);
    }
  }

  public static <T> T read(byte[] bytes, Class<T> type) {
    try {
      return OBJECT_MAPPER.readValue(bytes, type);
    } catch (Exception ex) {
      throw new IllegalStateException("Failed to decode CBOR payload to " + type.getName(), ex);
    }
  }

  public static ObjectMapper mapper() {
    return OBJECT_MAPPER;
  }
}
