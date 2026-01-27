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
