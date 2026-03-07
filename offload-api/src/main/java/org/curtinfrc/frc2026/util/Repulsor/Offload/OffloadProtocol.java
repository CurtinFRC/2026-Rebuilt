package org.curtinfrc.frc2026.util.Repulsor.Offload;

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
    byte[] frame = new byte[22 + taskBytes.length + payload.length];
    int position = 0;
    position = putInt(frame, position, MAGIC);
    position = putShort(frame, position, VERSION);
    position = putLong(frame, position, correlationId);
    position = putInt(frame, position, taskBytes.length);
    System.arraycopy(taskBytes, 0, frame, position, taskBytes.length);
    position += taskBytes.length;
    position = putInt(frame, position, payload.length);
    System.arraycopy(payload, 0, frame, position, payload.length);
    return frame;
  }

  public static RequestFrame readRequest(DataInputStream input) throws IOException {
    return parseRequestFrame(input);
  }

  public static RequestFrame parseRequest(byte[] frameBytes) throws IOException {
    return parseRequest(frameBytes, 0, frameBytes.length);
  }

  public static RequestFrame parseRequest(byte[] frameBytes, int offset, int length)
      throws IOException {
    if (length < 22) {
      throw new IOException("Invalid request frame length: " + length);
    }
    int end = offset + length;
    int position = offset;

    int magic = getInt(frameBytes, position);
    position += Integer.BYTES;
    if (magic != MAGIC) {
      throw new IOException("Invalid request magic: " + Integer.toHexString(magic));
    }

    short version = getShort(frameBytes, position);
    position += Short.BYTES;
    if (version != VERSION) {
      throw new IOException("Unsupported protocol version: " + version);
    }

    long correlationId = getLong(frameBytes, position);
    position += Long.BYTES;

    int taskLength = getInt(frameBytes, position);
    position += Integer.BYTES;
    if (taskLength < 0 || taskLength > 64_000 || position + taskLength + Integer.BYTES > end) {
      throw new IOException("Invalid task id length: " + taskLength);
    }

    String taskId = new String(frameBytes, position, taskLength, StandardCharsets.UTF_8);
    position += taskLength;

    int payloadLength = getInt(frameBytes, position);
    position += Integer.BYTES;
    if (payloadLength < 0 || payloadLength > 16_000_000 || position + payloadLength != end) {
      throw new IOException("Invalid payload length: " + payloadLength);
    }

    byte[] payload = new byte[payloadLength];
    System.arraycopy(frameBytes, position, payload, 0, payloadLength);
    return new RequestFrame(correlationId, taskId, payload);
  }

  public static void writeResponse(
      DataOutputStream output, long correlationId, byte status, byte[] payload) throws IOException {
    output.write(serializeResponse(correlationId, status, payload));
    output.flush();
  }

  public static byte[] serializeResponse(long correlationId, byte status, byte[] payload)
      throws IOException {
    byte[] frame = new byte[19 + payload.length];
    int position = 0;
    position = putInt(frame, position, MAGIC);
    position = putShort(frame, position, VERSION);
    position = putLong(frame, position, correlationId);
    frame[position++] = status;
    position = putInt(frame, position, payload.length);
    System.arraycopy(payload, 0, frame, position, payload.length);
    return frame;
  }

  public static ResponseFrame readResponse(DataInputStream input) throws IOException {
    return parseResponseFrame(input);
  }

  public static ResponseFrame parseResponse(byte[] frameBytes) throws IOException {
    return parseResponse(frameBytes, 0, frameBytes.length);
  }

  public static ResponseFrame parseResponse(byte[] frameBytes, int offset, int length)
      throws IOException {
    if (length < 19) {
      throw new IOException("Invalid response frame length: " + length);
    }
    int end = offset + length;
    int position = offset;

    int magic = getInt(frameBytes, position);
    position += Integer.BYTES;
    if (magic != MAGIC) {
      throw new IOException("Invalid response magic: " + Integer.toHexString(magic));
    }

    short version = getShort(frameBytes, position);
    position += Short.BYTES;
    if (version != VERSION) {
      throw new IOException("Unsupported protocol version: " + version);
    }

    long correlationId = getLong(frameBytes, position);
    position += Long.BYTES;
    byte status = frameBytes[position++];

    int payloadLength = getInt(frameBytes, position);
    position += Integer.BYTES;
    if (payloadLength < 0 || payloadLength > 16_000_000 || position + payloadLength != end) {
      throw new IOException("Invalid payload length: " + payloadLength);
    }

    byte[] payload = new byte[payloadLength];
    System.arraycopy(frameBytes, position, payload, 0, payloadLength);
    return new ResponseFrame(correlationId, status, payload);
  }

  public static byte[] serializeTimedPayload(
      byte[] payload, long queueNs, long executeNs, long serverNs) throws IOException {
    byte[] output = new byte[TIMED_PAYLOAD_HEADER_BYTES + payload.length];
    int position = 0;
    position = putInt(output, position, TIMED_PAYLOAD_MAGIC);
    position = putLong(output, position, queueNs);
    position = putLong(output, position, executeNs);
    position = putLong(output, position, serverNs);
    System.arraycopy(payload, 0, output, position, payload.length);
    return output;
  }

  public static TimedPayload parseTimedPayload(byte[] payloadBytes) throws IOException {
    return parseTimedPayload(payloadBytes, 0, payloadBytes.length);
  }

  public static TimedPayload parseTimedPayload(byte[] payloadBytes, int offset, int length)
      throws IOException {
    if (length < TIMED_PAYLOAD_HEADER_BYTES) {
      throw new IOException("Invalid timed payload length: " + length);
    }
    int end = offset + length;
    int position = offset;
    int magic = getInt(payloadBytes, position);
    position += Integer.BYTES;
    if (magic != TIMED_PAYLOAD_MAGIC) {
      throw new IOException("Invalid timed payload magic: " + Integer.toHexString(magic));
    }
    long queueNs = getLong(payloadBytes, position);
    position += Long.BYTES;
    long executeNs = getLong(payloadBytes, position);
    position += Long.BYTES;
    long serverNs = getLong(payloadBytes, position);
    position += Long.BYTES;
    byte[] payload = new byte[end - position];
    System.arraycopy(payloadBytes, position, payload, 0, payload.length);
    return new TimedPayload(payload, queueNs, executeNs, serverNs);
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

  private static int putInt(byte[] output, int offset, int value) {
    output[offset] = (byte) (value >>> 24);
    output[offset + 1] = (byte) (value >>> 16);
    output[offset + 2] = (byte) (value >>> 8);
    output[offset + 3] = (byte) value;
    return offset + Integer.BYTES;
  }

  private static int putShort(byte[] output, int offset, short value) {
    output[offset] = (byte) (value >>> 8);
    output[offset + 1] = (byte) value;
    return offset + Short.BYTES;
  }

  private static int putLong(byte[] output, int offset, long value) {
    output[offset] = (byte) (value >>> 56);
    output[offset + 1] = (byte) (value >>> 48);
    output[offset + 2] = (byte) (value >>> 40);
    output[offset + 3] = (byte) (value >>> 32);
    output[offset + 4] = (byte) (value >>> 24);
    output[offset + 5] = (byte) (value >>> 16);
    output[offset + 6] = (byte) (value >>> 8);
    output[offset + 7] = (byte) value;
    return offset + Long.BYTES;
  }

  private static short getShort(byte[] input, int offset) {
    return (short) (((input[offset] & 0xFF) << 8) | (input[offset + 1] & 0xFF));
  }

  private static int getInt(byte[] input, int offset) {
    return ((input[offset] & 0xFF) << 24)
        | ((input[offset + 1] & 0xFF) << 16)
        | ((input[offset + 2] & 0xFF) << 8)
        | (input[offset + 3] & 0xFF);
  }

  private static long getLong(byte[] input, int offset) {
    return ((long) (input[offset] & 0xFF) << 56)
        | ((long) (input[offset + 1] & 0xFF) << 48)
        | ((long) (input[offset + 2] & 0xFF) << 40)
        | ((long) (input[offset + 3] & 0xFF) << 32)
        | ((long) (input[offset + 4] & 0xFF) << 24)
        | ((long) (input[offset + 5] & 0xFF) << 16)
        | ((long) (input[offset + 6] & 0xFF) << 8)
        | ((long) (input[offset + 7] & 0xFF));
  }

  public record RequestFrame(long correlationId, String taskId, byte[] payload) {}

  public record ResponseFrame(long correlationId, byte status, byte[] payload) {}

  public record TimedPayload(byte[] payload, long queueNs, long executeNs, long serverNs) {}
}
