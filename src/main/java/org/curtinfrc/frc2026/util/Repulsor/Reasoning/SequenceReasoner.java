/*
 * =============================================================
 *  Repulsor Library
 *  Copyright (c) 2026 Paul Hodges
 *
 *  Licensed under the MIT License.
 *  SPDX-License-Identifier: MIT
 * =============================================================
 */


package org.curtinfrc.frc2026.util.Repulsor.Reasoning;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.EnumSet;
import java.util.List;
import java.util.Objects;

public final class SequenceReasoner<F extends Enum<F>, C> implements Reasoner<F, C> {
  public static final class Transition<F extends Enum<F>, C> {
    public final String name;
    public final int priority;
    public final double minPhaseAgeSec;
    public final Condition<C> condition;
    public final int nextPhaseIndex;

    public Transition(
        String name,
        int priority,
        double minPhaseAgeSec,
        Condition<C> condition,
        int nextPhaseIndex) {
      this.name = Objects.requireNonNull(name);
      this.priority = priority;
      this.minPhaseAgeSec = Math.max(0.0, minPhaseAgeSec);
      this.condition = Objects.requireNonNull(condition);
      this.nextPhaseIndex = nextPhaseIndex;
    }
  }

  public static final class Phase<F extends Enum<F>, C> {
    public final String name;
    public final EnumSet<F> flags;
    public final double minDurationSec;
    public final double maxDurationSec;
    public final int autoNextIndex;
    public final List<Transition<F, C>> transitions;

    public Phase(
        String name,
        EnumSet<F> flags,
        double minDurationSec,
        double maxDurationSec,
        int autoNextIndex,
        List<Transition<F, C>> transitions) {
      this.name = Objects.requireNonNull(name);
      this.flags = EnumSet.copyOf(Objects.requireNonNull(flags));
      this.minDurationSec = Math.max(0.0, minDurationSec);
      this.maxDurationSec = maxDurationSec <= 0.0 ? Double.POSITIVE_INFINITY : maxDurationSec;
      this.autoNextIndex = autoNextIndex;
      List<Transition<F, C>> copy = new ArrayList<>(Objects.requireNonNull(transitions));
      copy.sort(Comparator.comparingInt((Transition<F, C> t) -> t.priority).reversed());
      this.transitions = Collections.unmodifiableList(copy);
    }
  }

  public static final class Builder<F extends Enum<F>, C> {
    private final Class<F> enumClass;
    private final Clock clock;
    private final Signals signals;
    private final List<Phase<F, C>> phases = new ArrayList<>();
    private int startIndex = 0;

    public Builder(Class<F> enumClass, Clock clock) {
      this(enumClass, clock, new Blackboard());
    }

    public Builder(Class<F> enumClass, Clock clock, Signals signals) {
      this.enumClass = Objects.requireNonNull(enumClass);
      this.clock = Objects.requireNonNull(clock);
      this.signals = Objects.requireNonNull(signals);
    }

    public Signals signals() {
      return signals;
    }

    public Builder<F, C> startAt(int index) {
      this.startIndex = Math.max(0, index);
      return this;
    }

    public Builder<F, C> addPhase(
        String name,
        EnumSet<F> flags,
        double minDurationSec,
        double maxDurationSec,
        int autoNextIndex) {
      phases.add(
          new Phase<>(
              name, flags, minDurationSec, maxDurationSec, autoNextIndex, new ArrayList<>()));
      return this;
    }

    public Builder<F, C> addPhaseFor(String name, EnumSet<F> flags, double durationSec) {
      int next = phases.size() + 1;
      phases.add(new Phase<>(name, flags, durationSec, durationSec, next, new ArrayList<>()));
      return this;
    }

    public Builder<F, C> addTransition(
        int fromPhaseIndex,
        String name,
        int priority,
        double minPhaseAgeSec,
        Condition<C> condition,
        int nextPhaseIndex) {
      if (fromPhaseIndex < 0 || fromPhaseIndex >= phases.size()) {
        throw new IndexOutOfBoundsException(
            "fromPhaseIndex=" + fromPhaseIndex + " size=" + phases.size());
      }
      Phase<F, C> p = phases.get(fromPhaseIndex);
      List<Transition<F, C>> ts = new ArrayList<>(p.transitions);
      ts.add(new Transition<>(name, priority, minPhaseAgeSec, condition, nextPhaseIndex));
      phases.set(
          fromPhaseIndex,
          new Phase<>(p.name, p.flags, p.minDurationSec, p.maxDurationSec, p.autoNextIndex, ts));
      return this;
    }

    public SequenceReasoner<F, C> build() {
      if (phases.isEmpty()) {
        phases.add(
            new Phase<>(
                "idle",
                EnumSet.noneOf(enumClass),
                0.0,
                Double.POSITIVE_INFINITY,
                0,
                new ArrayList<>()));
      }
      int s = Math.min(startIndex, phases.size() - 1);
      return new SequenceReasoner<>(enumClass, clock, signals, phases, s);
    }
  }

  private final Class<F> enumClass;
  private final Clock clock;
  private final Signals signals;
  private final List<Phase<F, C>> phases;

  private int phaseIndex;
  private double phaseStartSec;

  public SequenceReasoner(
      Class<F> enumClass, Clock clock, Signals signals, List<Phase<F, C>> phases, int startIndex) {
    this.enumClass = Objects.requireNonNull(enumClass);
    this.clock = Objects.requireNonNull(clock);
    this.signals = Objects.requireNonNull(signals);
    this.phases = List.copyOf(Objects.requireNonNull(phases));
    this.phaseIndex = Math.max(0, Math.min(startIndex, this.phases.size() - 1));
    this.phaseStartSec = Double.NaN;
  }

  public Signals signals() {
    return signals;
  }

  public int phaseIndex() {
    return phaseIndex;
  }

  public String phaseName() {
    return phases.get(phaseIndex).name;
  }

  public void forcePhase(int index) {
    int clamped = Math.max(0, Math.min(index, phases.size() - 1));
    if (clamped != phaseIndex) {
      phaseIndex = clamped;
      phaseStartSec = clock.nowSec();
    }
  }

  @Override
  public EnumSet<F> update(C ctx) {
    double now = clock.nowSec();
    if (!Double.isFinite(phaseStartSec)) phaseStartSec = now;

    Phase<F, C> p = phases.get(phaseIndex);
    double age = Math.max(0.0, now - phaseStartSec);

    int chosenNext = -1;
    int chosenPrio = Integer.MIN_VALUE;

    if (age >= p.minDurationSec) {
      for (Transition<F, C> t : p.transitions) {
        if (age < t.minPhaseAgeSec) continue;
        if (!t.condition.test(ctx, signals)) continue;
        if (t.priority > chosenPrio) {
          chosenPrio = t.priority;
          chosenNext = t.nextPhaseIndex;
        }
      }
    }

    if (chosenNext < 0
        && age >= p.maxDurationSec
        && p.autoNextIndex >= 0
        && p.autoNextIndex < phases.size()) {
      chosenNext = p.autoNextIndex;
    }

    if (chosenNext >= 0 && chosenNext < phases.size() && chosenNext != phaseIndex) {
      phaseIndex = chosenNext;
      phaseStartSec = now;
      p = phases.get(phaseIndex);
    }

    return EnumSet.copyOf(p.flags);
  }

  @Override
  public void reset() {
    signals.clear();
    phaseIndex = 0;
    phaseStartSec = Double.NaN;
  }
}
