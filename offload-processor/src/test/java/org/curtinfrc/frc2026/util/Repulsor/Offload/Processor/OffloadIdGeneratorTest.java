package org.curtinfrc.frc2026.util.Repulsor.Offload.Processor;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class OffloadIdGeneratorTest {
  @Test
  void autoIdShouldBeStable() {
    String first =
        OffloadIdGenerator.autoId(
            "org.curtinfrc.TestClass", "run", "com.example.Request->com.example.Response", 2);
    String second =
        OffloadIdGenerator.autoId(
            "org.curtinfrc.TestClass", "run", "com.example.Request->com.example.Response", 2);

    assertEquals(first, second);
  }
}
