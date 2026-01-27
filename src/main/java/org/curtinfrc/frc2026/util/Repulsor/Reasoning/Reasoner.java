// File: src/main/java/org/curtinfrc/frc2026/util/Repulsor/Reasoning/Reasoner.java
package org.curtinfrc.frc2026.util.Repulsor.Reasoning;

import java.util.EnumSet;

public interface Reasoner<F extends Enum<F>, C> {
  EnumSet<F> update(C ctx);

  default void reset() {}
}
