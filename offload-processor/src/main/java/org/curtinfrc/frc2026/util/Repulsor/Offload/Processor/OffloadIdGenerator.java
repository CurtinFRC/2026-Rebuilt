package org.curtinfrc.frc2026.util.Repulsor.Offload.Processor;

import java.nio.charset.StandardCharsets;
import java.security.MessageDigest;

public final class OffloadIdGenerator {
  private OffloadIdGenerator() {}

  public static String autoId(
      String declaringClass, String methodName, String signature, int version) {
    String basis = declaringClass + "#" + methodName + "(" + signature + ")@v" + version;
    String hex = sha256Hex(basis).substring(0, 16);
    return "offload." + hex + ".v" + version;
  }

  private static String sha256Hex(String value) {
    try {
      MessageDigest digest = MessageDigest.getInstance("SHA-256");
      byte[] hash = digest.digest(value.getBytes(StandardCharsets.UTF_8));
      StringBuilder output = new StringBuilder(hash.length * 2);
      for (byte b : hash) {
        output.append(String.format("%02x", b));
      }
      return output.toString();
    } catch (Exception ex) {
      throw new IllegalStateException("Failed to generate offload id", ex);
    }
  }
}
