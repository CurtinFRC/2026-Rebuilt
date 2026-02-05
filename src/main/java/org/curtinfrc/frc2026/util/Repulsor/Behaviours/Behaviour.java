/*
 * =============================================================
 *  Repulsor Library
 *  Copyright (c) 2026 Paul Hodges
 *
 *  Licensed under the Repulsor Non-Commercial License (RNC-1.0).
 * =============================================================
 */
package org.curtinfrc.frc2026.util.Repulsor.Behaviours;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.EnumSet;

public abstract class Behaviour {
  public abstract String name();

  public abstract int priority();

  public abstract boolean shouldRun(EnumSet<BehaviourFlag> flags, BehaviourContext ctx);

  public abstract Command build(BehaviourContext ctx);
}

