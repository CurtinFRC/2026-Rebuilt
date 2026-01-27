package org.curtinfrc.frc2026.util.Repulsor.Reasoning;

import java.util.ArrayList;
import java.util.EnumSet;
import java.util.List;
import java.util.Objects;

public final class RuleReasoner<F extends Enum<F>, C> implements Reasoner<F, C> {
  public static final class Rule<F extends Enum<F>, C> {
    public final String name;
    public final int priority;
    public final Condition<C> condition;
    public final EnumSet<F> flags;

    public Rule(String name, int priority, Condition<C> condition, EnumSet<F> flags) {
      this.name = Objects.requireNonNull(name);
      this.priority = priority;
      this.condition = Objects.requireNonNull(condition);
      this.flags = Objects.requireNonNull(flags);
    }
  }

  private final Class<F> enumClass;
  private final Signals signals;
  private final List<Rule<F, C>> rules = new ArrayList<>();
  private EnumSet<F> fallback;

  public RuleReasoner(Class<F> enumClass) {
    this(enumClass, new Blackboard());
  }

  public RuleReasoner(Class<F> enumClass, Signals signals) {
    this.enumClass = Objects.requireNonNull(enumClass);
    this.signals = Objects.requireNonNull(signals);
    this.fallback = EnumSet.noneOf(enumClass);
  }

  public Signals signals() {
    return signals;
  }

  public RuleReasoner<F, C> setFallback(EnumSet<F> flags) {
    this.fallback = Objects.requireNonNull(flags);
    return this;
  }

  public RuleReasoner<F, C> addRule(
      String name, int priority, Condition<C> condition, EnumSet<F> flags) {
    rules.add(new Rule<>(name, priority, condition, flags));
    return this;
  }

  @Override
  public EnumSet<F> update(C ctx) {
    Rule<F, C> best = null;
    int bestP = Integer.MIN_VALUE;
    for (Rule<F, C> r : rules) {
      if (r.condition.test(ctx, signals) && r.priority > bestP) {
        best = r;
        bestP = r.priority;
      }
    }
    return best != null ? EnumSet.copyOf(best.flags) : EnumSet.copyOf(fallback);
  }

  @Override
  public void reset() {
    signals.clear();
  }
}
