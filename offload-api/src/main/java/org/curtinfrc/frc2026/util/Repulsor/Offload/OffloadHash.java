package org.curtinfrc.frc2026.util.Repulsor.Offload;

import java.security.MessageDigest;

public final class OffloadHash {
  private OffloadHash() {}

  public static String sha256Hex(byte[] bytes) {
    try {
      MessageDigest digest = MessageDigest.getInstance("SHA-256");
      byte[] hash = digest.digest(bytes);
      StringBuilder output = new StringBuilder(hash.length * 2);
      for (byte b : hash) {
        output.append(String.format("%02x", b));
      }
      return output.toString();
    } catch (Exception ex) {
      throw new IllegalStateException("Failed to compute SHA-256 hash", ex);
    }
  }
}
