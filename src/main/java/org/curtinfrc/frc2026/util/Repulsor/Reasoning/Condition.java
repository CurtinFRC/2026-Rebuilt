// File: src/main/java/org/curtinfrc/frc2026/util/Repulsor/Reasoning/Condition.java
package org.curtinfrc.frc2026.util.Repulsor.Reasoning;

@FunctionalInterface
public interface Condition<C> {
  boolean test(C ctx, Signals signals);
}
