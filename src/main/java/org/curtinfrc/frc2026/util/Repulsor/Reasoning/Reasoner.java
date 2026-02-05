/*
 * =============================================================
 *  Repulsor Library
 *  Copyright (c) 2026 Paul Hodges
 *
 *  Licensed under the Repulsor Non-Commercial License (RNC-1.0).
 * =============================================================
 */
package org.curtinfrc.frc2026.util.Repulsor.Reasoning;

import java.util.EnumSet;

public interface Reasoner<F extends Enum<F>, C> {
  EnumSet<F> update(C ctx);

  default void reset() {}
}

