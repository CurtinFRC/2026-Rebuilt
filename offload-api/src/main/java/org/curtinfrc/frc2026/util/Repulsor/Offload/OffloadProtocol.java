package org.curtinfrc.frc2026.util.Repulsor.Offload;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.EOFException;
import java.io.IOException;
import java.nio.charset.StandardCharsets;

public final class OffloadProtocol {
  public static final int MAGIC = 0x4F46464C;
  public static final short VERSION = 1;

  public static final String TASK_PING = "__PING__";
  public static final String TASK_HELLO = "__HELLO__";

  public static final byte STATUS_OK = 0;
  public static final byte STATUS_ERR = 1;

  private OffloadProtocol() {}

  public static void writeRequest(
      DataOutputStream output, long correlationId, String taskId, byte[] payload)
      throws IOException {
    byte[] taskBytes = taskId.getBytes(StandardCharsets.UTF_8);
    output.writeInt(MAGIC);
    output.writeShort(VERSION);
    output.writeLong(correlationId);
    output.writeInt(taskBytes.length);
    output.write(taskBytes);
    output.writeInt(payload.length);
    output.write(payload);
    output.flush();
  }

  public static RequestFrame readRequest(DataInputStream input) throws IOException {
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

  public static void writeResponse(
      DataOutputStream output, long correlationId, byte status, byte[] payload) throws IOException {
    output.writeInt(MAGIC);
    output.writeShort(VERSION);
    output.writeLong(correlationId);
    output.writeByte(status);
    output.writeInt(payload.length);
    output.write(payload);
    output.flush();
  }

  public static ResponseFrame readResponse(DataInputStream input) throws IOException {
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
}
