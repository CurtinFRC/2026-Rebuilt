/*
 * Copyright (C) 2026 Paul Hodges
 *
 * This file is part of Repulsor.
 *
 * Repulsor is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Repulsor is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Repulsor. If not, see https://www.gnu.org/licenses/.
 */


package org.curtinfrc.frc2026.util.Repulsor.Behaviours;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.ArrayList;
import java.util.EnumSet;
import java.util.List;
import org.curtinfrc.frc2026.util.Repulsor.Reasoning.Reasoner;

public class BehaviourManager {
  private final List<Behaviour> behaviours = new ArrayList<>();
  private Behaviour active = null;
  private Command running = null;

  private Reasoner<BehaviourFlag, BehaviourContext> reasoner = null;

  public BehaviourManager add(Behaviour b) {
    behaviours.add(b);
    return this;
  }

  public Behaviour active() {
    return active;
  }

  public BehaviourManager setReasoner(Reasoner<BehaviourFlag, BehaviourContext> reasoner) {
    this.reasoner = reasoner;
    return this;
  }

  public void update(BehaviourContext ctx) {
    EnumSet<BehaviourFlag> flags =
        reasoner != null ? reasoner.update(ctx) : EnumSet.noneOf(BehaviourFlag.class);

    update(flags, ctx);
  }

  public void update(EnumSet<BehaviourFlag> flags, BehaviourContext ctx) {
    Behaviour best = null;
    int bestP = Integer.MIN_VALUE;
    for (Behaviour b : behaviours) {
      if (b.shouldRun(flags, ctx) && b.priority() > bestP) {
        best = b;
        bestP = b.priority();
      }
    }
    if (best == active) return;

    if (running != null) {
      running.cancel();
      running = null;
    }
    active = best;
    if (active != null) {
      running = active.build(ctx);
      CommandScheduler.getInstance().schedule(running);
    }
  }

  public void stop() {
    if (running != null) running.cancel();
    running = null;
    active = null;
    if (reasoner != null) reasoner.reset();
  }
}

