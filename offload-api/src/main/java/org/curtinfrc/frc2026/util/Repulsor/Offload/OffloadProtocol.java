package org.curtinfrc.frc2026.util.Repulsor.Offload;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.EOFException;
import java.io.IOException;
import java.nio.charset.StandardCharsets;

public final class OffloadProtocol {
  public static final int MAGIC = 0x4F46464C;
  public static final short VERSION = 1;
  private static final int TIMED_PAYLOAD_MAGIC = 0x54494D45;
  private static final int TIMED_PAYLOAD_HEADER_BYTES = Integer.BYTES + (Long.BYTES * 3);

  public static final String TASK_PING = "__PING__";
  public static final String TASK_HELLO = "__HELLO__";

  public static final byte STATUS_OK = 0;
  public static final byte STATUS_ERR = 1;
  public static final byte STATUS_OK_TIMED = 2;

  private OffloadProtocol() {}

  public static void writeRequest(
      DataOutputStream output, long correlationId, String taskId, byte[] payload)
      throws IOException {
    output.write(serializeRequest(correlationId, taskId, payload));
    output.flush();
  }

  public static byte[] serializeRequest(long correlationId, String taskId, byte[] payload)
      throws IOException {
    byte[] taskBytes = taskId.getBytes(StandardCharsets.UTF_8);
    ByteArrayOutputStream byteStream =
        new ByteArrayOutputStream(32 + taskBytes.length + payload.length);
    try (DataOutputStream output = new DataOutputStream(byteStream)) {
      output.writeInt(MAGIC);
      output.writeShort(VERSION);
      output.writeLong(correlationId);
      output.writeInt(taskBytes.length);
      output.write(taskBytes);
      output.writeInt(payload.length);
      output.write(payload);
      output.flush();
    }
    return byteStream.toByteArray();
  }

  public static RequestFrame readRequest(DataInputStream input) throws IOException {
    return parseRequestFrame(input);
  }

  public static RequestFrame parseRequest(byte[] frameBytes) throws IOException {
    return parseRequest(frameBytes, 0, frameBytes.length);
  }

  public static RequestFrame parseRequest(byte[] frameBytes, int offset, int length)
      throws IOException {
    try (DataInputStream input =
        new DataInputStream(new ByteArrayInputStream(frameBytes, offset, length))) {
      RequestFrame frame = parseRequestFrame(input);
      if (input.available() != 0) {
        throw new IOException("Trailing bytes in request frame");
      }
      return frame;
    }
  }

  public static void writeResponse(
      DataOutputStream output, long correlationId, byte status, byte[] payload) throws IOException {
    output.write(serializeResponse(correlationId, status, payload));
    output.flush();
  }

  public static byte[] serializeResponse(long correlationId, byte status, byte[] payload)
      throws IOException {
    ByteArrayOutputStream byteStream = new ByteArrayOutputStream(24 + payload.length);
    try (DataOutputStream output = new DataOutputStream(byteStream)) {
      output.writeInt(MAGIC);
      output.writeShort(VERSION);
      output.writeLong(correlationId);
      output.writeByte(status);
      output.writeInt(payload.length);
      output.write(payload);
      output.flush();
    }
    return byteStream.toByteArray();
  }

  public static ResponseFrame readResponse(DataInputStream input) throws IOException {
    return parseResponseFrame(input);
  }

  public static ResponseFrame parseResponse(byte[] frameBytes) throws IOException {
    return parseResponse(frameBytes, 0, frameBytes.length);
  }

  public static ResponseFrame parseResponse(byte[] frameBytes, int offset, int length)
      throws IOException {
    try (DataInputStream input =
        new DataInputStream(new ByteArrayInputStream(frameBytes, offset, length))) {
      ResponseFrame frame = parseResponseFrame(input);
      if (input.available() != 0) {
        throw new IOException("Trailing bytes in response frame");
      }
      return frame;
    }
  }

  public static byte[] serializeTimedPayload(
      byte[] payload, long queueNs, long executeNs, long serverNs) throws IOException {
    ByteArrayOutputStream byteStream =
        new ByteArrayOutputStream(TIMED_PAYLOAD_HEADER_BYTES + payload.length);
    try (DataOutputStream output = new DataOutputStream(byteStream)) {
      output.writeInt(TIMED_PAYLOAD_MAGIC);
      output.writeLong(queueNs);
      output.writeLong(executeNs);
      output.writeLong(serverNs);
      output.write(payload);
      output.flush();
    }
    return byteStream.toByteArray();
  }

  public static TimedPayload parseTimedPayload(byte[] payloadBytes) throws IOException {
    return parseTimedPayload(payloadBytes, 0, payloadBytes.length);
  }

  public static TimedPayload parseTimedPayload(byte[] payloadBytes, int offset, int length)
      throws IOException {
    if (length < TIMED_PAYLOAD_HEADER_BYTES) {
      throw new IOException("Invalid timed payload length: " + length);
    }

    try (DataInputStream input =
        new DataInputStream(new ByteArrayInputStream(payloadBytes, offset, length))) {
      int magic = input.readInt();
      if (magic != TIMED_PAYLOAD_MAGIC) {
        throw new IOException("Invalid timed payload magic: " + Integer.toHexString(magic));
      }
      long queueNs = input.readLong();
      long executeNs = input.readLong();
      long serverNs = input.readLong();
      byte[] payload = input.readAllBytes();
      return new TimedPayload(payload, queueNs, executeNs, serverNs);
    }
  }

  private static RequestFrame parseRequestFrame(DataInputStream input) throws IOException {
    int magic = input.readInt();
    if (magic != MAGIC) {
      throw new IOException("Invalid request magic: " + Integer.toHexString(magic));
    }
    short version = input.readShort();
    if (version != VERSION) {
      throw new IOException("Unsupported protocol version: " + version);
    }
    long correlationId = input.readLong();
    int taskLength = input.readInt();
    if (taskLength < 0 || taskLength > 64_000) {
      throw new IOException("Invalid task id length: " + taskLength);
    }
    byte[] taskBytes = readExact(input, taskLength);
    String taskId = new String(taskBytes, StandardCharsets.UTF_8);
    int payloadLength = input.readInt();
    if (payloadLength < 0 || payloadLength > 16_000_000) {
      throw new IOException("Invalid payload length: " + payloadLength);
    }
    byte[] payload = readExact(input, payloadLength);
    return new RequestFrame(correlationId, taskId, payload);
  }

  private static ResponseFrame parseResponseFrame(DataInputStream input) throws IOException {
    int magic = input.readInt();
    if (magic != MAGIC) {
      throw new IOException("Invalid response magic: " + Integer.toHexString(magic));
    }
    short version = input.readShort();
    if (version != VERSION) {
      throw new IOException("Unsupported protocol version: " + version);
    }
    long correlationId = input.readLong();
    byte status = input.readByte();
    int payloadLength = input.readInt();
    if (payloadLength < 0 || payloadLength > 16_000_000) {
      throw new IOException("Invalid payload length: " + payloadLength);
    }
    byte[] payload = readExact(input, payloadLength);
    return new ResponseFrame(correlationId, status, payload);
  }

  private static byte[] readExact(DataInputStream input, int length) throws IOException {
    byte[] output = input.readNBytes(length);
    if (output.length != length) {
      throw new EOFException("Unexpected EOF while reading frame payload");
    }
    return output;
  }

  public record RequestFrame(long correlationId, String taskId, byte[] payload) {}

  public record ResponseFrame(long correlationId, byte status, byte[] payload) {}

  public record TimedPayload(byte[] payload, long queueNs, long executeNs, long serverNs) {}
}
