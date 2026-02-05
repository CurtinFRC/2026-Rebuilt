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

package org.curtinfrc.frc2026.util.Repulsor.Commands;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.*;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

public final class GateTelemetry implements AutoCloseable {
  public enum Mode {
    ON_CHANGE,
    PERIODIC
  }

  private final NetworkTable root;
  private final List<Runnable> updaters = new ArrayList<>();
  private final Map<String, StringPublisher> infoStr = new HashMap<>();
  private final Map<String, IntegerPublisher> infoInt = new HashMap<>();
  private final Map<String, DoublePublisher> infoDbl = new HashMap<>();
  private Notifier notifier;
  private Mode mode = Mode.ON_CHANGE;
  private double minPeriodSec = 0.0;
  private double lastPush = 0.0;
  private final List<Notifier> heartbeatNotifiers = new ArrayList<>();

  public GateTelemetry(String rootPath) {
    this.root = NetworkTableInstance.getDefault().getTable(rootPath);
    setInfo("schema", "gate-telemetry");
    setInfo("version", 2);
    setInfo("started_at", Timer.getFPGATimestamp());
  }

  public void setMode(Mode m) {
    this.mode = m;
  }

  public void setRateLimit(double minPeriodSec) {
    this.minPeriodSec = Math.max(0.0, minPeriodSec);
  }

  public void poll() {
    if (mode == Mode.PERIODIC) {
      double now = Timer.getFPGATimestamp();
      if (now - lastPush < minPeriodSec) return;
      lastPush = now;
      for (var r : updaters) r.run();
      return;
    }
    double now = Timer.getFPGATimestamp();
    if (now - lastPush < minPeriodSec) return;
    lastPush = now;
    for (var r : updaters) r.run();
  }

  public void startPeriodic(double periodSec) {
    if (notifier != null) return;
    notifier = new Notifier(this::poll);
    notifier.startPeriodic(Math.max(0.02, periodSec));
  }

  public void stop() {
    if (notifier != null) {
      notifier.close();
      notifier = null;
    }

    for (var n : heartbeatNotifiers) {
      try {
        n.close();
      } catch (Exception ignored) {
      }
    }
    heartbeatNotifiers.clear();
  }

  @Override
  public void close() {
    stop();
  }

  private void setInfo(String key, String value) {
    infoStr.computeIfAbsent(key, k -> root.getStringTopic(k).publish()).set(value);
  }

  private void setInfo(String key, int value) {
    infoInt.computeIfAbsent(key, k -> root.getIntegerTopic(k).publish()).set(value);
  }

  private void setInfo(String key, double value) {
    infoDbl.computeIfAbsent(key, k -> root.getDoubleTopic(k).publish()).set(value);
  }

  public <T> void registerPhase(
      String name, Triggers.PhaseGate<T> gate, Function<T, String> encoder) {
    NetworkTable tbl = root.getSubTable(name);
    StringPublisher curStr = tbl.getStringTopic("current").publish();
    IntegerPublisher curInt = tbl.getIntegerTopic("current_int").publish();
    DoublePublisher lastChange = tbl.getDoubleTopic("last_change").publish();
    DoublePublisher uptime = tbl.getDoubleTopic("uptime").publish();
    IntegerPublisher changes = tbl.getIntegerTopic("change_count").publish();
    Map<String, BooleanPublisher> perState = new HashMap<>();
    Map<String, DoublePublisher> perStateUptime = new HashMap<>();
    Map<String, IntegerPublisher> perStateCount = new HashMap<>();
    final Object[] lastObj = {null};
    final String[] lastStr = {""};
    final long[] lastOrdinal = {Long.MIN_VALUE};
    final long[] changeCount = {0};
    final Map<String, Double> enterTimes = new HashMap<>();

    updaters.add(
        () -> {
          T cur = gate.phase();
          String s = encoder.apply(cur);
          long vInt =
              (cur instanceof Enum)
                  ? ((Enum<?>) cur).ordinal()
                  : (cur != null ? cur.hashCode() : 0);
          boolean changed = !Objects.equals(lastObj[0], cur);
          double now = Timer.getFPGATimestamp();
          if (changed) {
            lastObj[0] = cur;
            if (!Objects.equals(s, lastStr[0])) {
              curStr.set(s);
              lastStr[0] = s;
            }
            if (vInt != lastOrdinal[0]) {
              curInt.set(vInt);
              lastOrdinal[0] = vInt;
            }
            lastChange.set(now);
            changeCount[0]++;
            changes.set(changeCount[0]);
            enterTimes.put(s, now);
            BooleanPublisher pb =
                perState.computeIfAbsent(s, k -> tbl.getBooleanTopic("is/" + k).publish());
            pb.set(true);
          }
          Double et = enterTimes.get(s);
          if (et != null) uptime.set(now - et);
          for (Map.Entry<String, Double> e : enterTimes.entrySet()) {
            String st = e.getKey();
            double ut = now - e.getValue();
            perStateUptime
                .computeIfAbsent(st, k -> tbl.getDoubleTopic("uptime/" + k).publish())
                .set(ut);
          }
          for (String st : perState.keySet()) {
            if (!st.equals(s)) perState.get(st).set(false);
          }
          perStateCount
              .computeIfAbsent(s, k -> tbl.getIntegerTopic("changes/" + k).publish())
              .set(
                  (int)
                      perStateCount
                          .getOrDefault(s, tbl.getIntegerTopic("null").publish())
                          .getTopic()
                          .getHandle());
        });
  }

  public <T> void registerPhaseEnum(
      String name, Triggers.PhaseGate<T> gate, Class<? extends Enum<?>> enumClass) {
    registerPhase(name, gate, t -> t != null ? ((Enum<?>) t).name() : "null");
    NetworkTable tbl = root.getSubTable(name);
    for (Object c : enumClass.getEnumConstants()) {
      String n = ((Enum<?>) c).name();
      tbl.getBooleanTopic("known/" + n).publish().set(true);
    }
  }

  public <T> void registerPhaseInt(
      String name, Triggers.PhaseGate<T> gate, Function<T, Integer> encoder) {
    NetworkTable tbl = root.getSubTable(name);
    IntegerPublisher cur = tbl.getIntegerTopic("current").publish();
    DoublePublisher lastChange = tbl.getDoubleTopic("last_change").publish();
    DoublePublisher uptime = tbl.getDoubleTopic("uptime").publish();
    IntegerPublisher changes = tbl.getIntegerTopic("change_count").publish();
    final Object[] lastObj = {null};
    final long[] last = {Long.MIN_VALUE};
    final double[] enterAt = {Timer.getFPGATimestamp()};
    final long[] changeCount = {0};
    updaters.add(
        () -> {
          T p = gate.phase();
          if (p != lastObj[0]) {
            lastObj[0] = p;
            int v = (p == null) ? -1 : encoder.apply(p);
            if (v != last[0]) {
              cur.set(v);
              last[0] = v;
            }
            double now = Timer.getFPGATimestamp();
            lastChange.set(now);
            enterAt[0] = now;
            changeCount[0]++;
            changes.set(changeCount[0]);
          }
          uptime.set(Timer.getFPGATimestamp() - enterAt[0]);
        });
  }

  public <E extends Enum<E>> void registerParallel(String name, Triggers.ParallelGate<E> gate) {
    Class<E> cls = gate.cls();
    NetworkTable tbl = root.getSubTable(name);
    IntegerPublisher bitmaskPub = tbl.getIntegerTopic("bitmask").publish();
    StringPublisher csvPub = tbl.getStringTopic("active_csv").publish();
    IntegerPublisher countPub = tbl.getIntegerTopic("active_count").publish();
    DoublePublisher lastChange = tbl.getDoubleTopic("last_change").publish();
    Map<E, BooleanPublisher> tagPubs = new EnumMap<>(cls);
    Map<E, IntegerPublisher> tagCountPubs = new EnumMap<>(cls);
    Map<E, DoublePublisher> tagUptimePubs = new EnumMap<>(cls);
    Map<E, Double> tagEnterTimes = new EnumMap<>(cls);
    Map<E, Long> tagChangeCounts = new EnumMap<>(cls);
    for (E e : cls.getEnumConstants()) {
      tagPubs.put(e, tbl.getBooleanTopic("is/" + e.name()).publish());
      tagCountPubs.put(e, tbl.getIntegerTopic("changes/" + e.name()).publish());
      tagUptimePubs.put(e, tbl.getDoubleTopic("uptime/" + e.name()).publish());
      tagEnterTimes.put(e, Double.NaN);
      tagChangeCounts.put(e, 0L);
    }
    final long[] lastMask = {Long.MIN_VALUE};
    final String[] lastCsv = {""};

    updaters.add(
        () -> {
          Set<E> snap = gate.snapshot();
          double now = Timer.getFPGATimestamp();
          long mask = 0;
          List<String> activeNames = new ArrayList<>();
          for (E e : cls.getEnumConstants()) {
            boolean on = snap.contains(e);
            if (on) {
              mask |= (1L << e.ordinal());
              activeNames.add(e.name());
              if (Double.isNaN(tagEnterTimes.get(e))) tagEnterTimes.put(e, now);
              tagUptimePubs.get(e).set(now - tagEnterTimes.get(e));
            } else {
              tagEnterTimes.put(e, Double.NaN);
              tagUptimePubs.get(e).set(0.0);
            }
            tagPubs.get(e).set(on);
          }
          if (mask != lastMask[0]) {
            bitmaskPub.set(mask);
            lastMask[0] = mask;
            lastChange.set(now);
          }
          String csv = String.join(",", activeNames);
          if (!csv.equals(lastCsv[0])) {
            csvPub.set(csv);
            lastCsv[0] = csv;
          }
          countPub.set(activeNames.size());
          for (E e : cls.getEnumConstants()) {
            boolean on = snap.contains(e);
            long cnt = tagChangeCounts.get(e);
            boolean topicVal = on;
            tagPubs.get(e).set(topicVal);
            if (on && Double.isNaN(tagEnterTimes.get(e))) {
              tagEnterTimes.put(e, now);
              cnt++;
              tagChangeCounts.put(e, cnt);
              tagCountPubs.get(e).set(cnt);
            }
            if (!on && !Double.isNaN(tagEnterTimes.get(e))) {
              tagEnterTimes.put(e, Double.NaN);
              cnt++;
              tagChangeCounts.put(e, cnt);
              tagCountPubs.get(e).set(cnt);
            }
          }
        });
  }

  public void registerTrigger(String name, Trigger t) {
    NetworkTable tbl = root.getSubTable(name);
    BooleanPublisher level = tbl.getBooleanTopic("level").publish();
    BooleanPublisher rising = tbl.getBooleanTopic("rising").publish();
    BooleanPublisher falling = tbl.getBooleanTopic("falling").publish();
    IntegerPublisher rises = tbl.getIntegerTopic("rises").publish();
    IntegerPublisher falls = tbl.getIntegerTopic("falls").publish();
    DoublePublisher lastRise = tbl.getDoubleTopic("last_rise").publish();
    DoublePublisher lastFall = tbl.getDoubleTopic("last_fall").publish();
    final boolean[] last = {false};
    final long[] rc = {0};
    final long[] fc = {0};
    updaters.add(
        () -> {
          boolean now = t.getAsBoolean();
          level.set(now);
          boolean r = now && !last[0];
          boolean f = !now && last[0];
          rising.set(r);
          falling.set(f);
          double ts = Timer.getFPGATimestamp();
          if (r) {
            rc[0]++;
            rises.set(rc[0]);
            lastRise.set(ts);
          }
          if (f) {
            fc[0]++;
            falls.set(fc[0]);
            lastFall.set(ts);
          }
          last[0] = now;
        });
  }

  public void registerDerived(String name, Supplier<Boolean> boolFn) {
    NetworkTable tbl = root.getSubTable(name);
    BooleanPublisher p = tbl.getBooleanTopic("value").publish();
    updaters.add(() -> p.set(boolFn.get()));
  }

  public void registerDerivedNumber(String name, DoubleSupplier dblFn) {
    NetworkTable tbl = root.getSubTable(name);
    DoublePublisher p = tbl.getDoubleTopic("value").publish();
    updaters.add(() -> p.set(dblFn.getAsDouble()));
  }

  public void registerEnumCatalog(String name, Class<? extends Enum<?>> enumClass) {
    NetworkTable tbl = root.getSubTable(name);
    StringPublisher list = tbl.getStringTopic("catalog").publish();
    List<String> all = new ArrayList<>();
    for (Object c : enumClass.getEnumConstants()) all.add(((Enum<?>) c).name());
    list.set(String.join(",", all));
  }

  public void registerHeartbeat(String name, double periodSec) {
    NetworkTable tbl = root.getSubTable(name);
    DoublePublisher hb = tbl.getDoubleTopic("t").publish();
    DoublePublisher dt = tbl.getDoubleTopic("dt").publish();
    final double[] last = {Timer.getFPGATimestamp()};
    Notifier n =
        new Notifier(
            () -> {
              double now = Timer.getFPGATimestamp();
              hb.set(now);
              dt.set(now - last[0]);
              last[0] = now;
            });
    n.startPeriodic(Math.max(0.02, periodSec));
    heartbeatNotifiers.add(n);
  }
}
