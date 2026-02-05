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

package org.curtinfrc.frc2026.util.Repulsor;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;

public final class Heatmap {
  public static interface HeatmapProvider {
    Heatmap getHeatmap();
  }

  public static final class Block {
    public final String uid;
    public final Translation2d position;
    public final Transform2d size;
    public final double heat;

    public Block(String uid, Translation2d position, Transform2d size, double heat) {
      this.uid = requireUid(uid);
      this.position = Objects.requireNonNull(position, "position");
      this.size = Objects.requireNonNull(size, "size");
      this.heat = heat;
    }

    public double x0() {
      return position.getX();
    }

    public double y0() {
      return position.getY();
    }

    public double w() {
      return Math.max(0.0, size.getTranslation().getX());
    }

    public double h() {
      return Math.max(0.0, size.getTranslation().getY());
    }

    public double x1() {
      return x0() + w();
    }

    public double y1() {
      return y0() + h();
    }

    public Translation2d center() {
      return new Translation2d(x0() + 0.5 * w(), y0() + 0.5 * h());
    }

    public boolean contains(Translation2d p) {
      double x = p.getX();
      double y = p.getY();
      return x >= x0() && x <= x1() && y >= y0() && y <= y1();
    }

    public double area() {
      return w() * h();
    }
  }

  public static final class Transition {
    public final String from;
    public final String to;
    public final double scale;

    public Transition(String from, String to, double scale) {
      this.from = requireUid(from);
      this.to = requireUid(to);
      this.scale = scale;
    }
  }

  public enum BlendMode {
    MIN_T,
    MAX_EDGE_INFLUENCE,
    WEIGHTED_SUM
  }

  public static final class Builder {
    private final Map<String, Block> blocks = new LinkedHashMap<>();
    private final List<Transition> transitions = new ArrayList<>();
    private double eps = 1e-9;
    private BlendMode blendMode = BlendMode.MAX_EDGE_INFLUENCE;

    public Builder eps(double eps) {
      this.eps = Math.max(0.0, eps);
      return this;
    }

    public Builder blendMode(BlendMode mode) {
      this.blendMode = Objects.requireNonNull(mode, "mode");
      return this;
    }

    public Builder block(String uid, Translation2d pos, double w, double h, double heat) {
      return block(
          new Block(
              uid,
              pos,
              new Transform2d(
                  new Translation2d(w, h), edu.wpi.first.math.geometry.Rotation2d.kZero),
              heat));
    }

    public Builder block(Block b) {
      Objects.requireNonNull(b, "b");
      if (blocks.containsKey(b.uid))
        throw new IllegalArgumentException("Duplicate block uid: " + b.uid);
      blocks.put(b.uid, b);
      return this;
    }

    public Builder transition(String from, String to, double scale) {
      transitions.add(new Transition(from, to, scale));
      return this;
    }

    public Builder bidirectional(String a, String b, double scaleAB, double scaleBA) {
      transitions.add(new Transition(a, b, scaleAB));
      transitions.add(new Transition(b, a, scaleBA));
      return this;
    }

    public Heatmap build() {
      return new Heatmap(new ArrayList<>(blocks.values()), transitions, eps, blendMode);
    }
  }

  public static Builder builder() {
    return new Builder();
  }

  public enum Side {
    LEFT,
    RIGHT,
    BOTTOM,
    TOP
  }

  public static final class TransitionZone {
    public final String from;
    public final String to;
    public final Side sideInTo;
    public final double thickness;
    public final double scale;

    private TransitionZone(String from, String to, Side sideInTo, double thickness, double scale) {
      this.from = from;
      this.to = to;
      this.sideInTo = sideInTo;
      this.thickness = thickness;
      this.scale = scale;
    }
  }

  private static final class Incoming {
    final Block from;
    final Block to;
    final Side side;
    final double thickness;
    final double scale;

    Incoming(Block from, Block to, Side side, double thickness, double scale) {
      this.from = from;
      this.to = to;
      this.side = side;
      this.thickness = thickness;
      this.scale = scale;
    }

    boolean contains(Translation2d p) {
      if (!to.contains(p)) return false;
      if (thickness <= 1e-12) return false;

      double x = p.getX();
      double y = p.getY();
      switch (side) {
        case LEFT:
          return (x - to.x0()) <= thickness;
        case RIGHT:
          return (to.x1() - x) <= thickness;
        case BOTTOM:
          return (y - to.y0()) <= thickness;
        case TOP:
          return (to.y1() - y) <= thickness;
        default:
          return false;
      }
    }

    double t(Translation2d p) {
      if (thickness <= 1e-12) return 1.0;
      double x = p.getX();
      double y = p.getY();
      double d;
      switch (side) {
        case LEFT:
          d = x - to.x0();
          break;
        case RIGHT:
          d = to.x1() - x;
          break;
        case BOTTOM:
          d = y - to.y0();
          break;
        case TOP:
          d = to.y1() - y;
          break;
        default:
          d = thickness;
          break;
      }
      return clamp(d / thickness, 0.0, 1.0);
    }

    double influence(Translation2d p) {
      if (!contains(p)) return 0.0;
      return 1.0 - t(p);
    }

    double blendedHeatAt(Translation2d p) {
      return lerp(from.heat, to.heat, t(p));
    }
  }

  private final List<Block> blocks;
  private final List<Transition> transitions;
  private final double eps;
  private final BlendMode blendMode;

  private final Map<String, Block> byUid = new HashMap<>();
  private final Map<String, List<Incoming>> incomingByTo = new HashMap<>();

  private Heatmap(
      List<Block> blocks, List<Transition> transitions, double eps, BlendMode blendMode) {
    this.blocks = Collections.unmodifiableList(new ArrayList<>(blocks));
    this.transitions = Collections.unmodifiableList(new ArrayList<>(transitions));
    this.eps = eps;
    this.blendMode = blendMode;

    for (Block b : this.blocks) {
      Block prev = byUid.putIfAbsent(b.uid, b);
      if (prev != null) throw new IllegalArgumentException("Duplicate block uid: " + b.uid);
    }

    buildIncoming();
  }

  public List<Block> blocks() {
    return blocks;
  }

  public List<Transition> transitions() {
    return transitions;
  }

  public Block block(String uid) {
    return byUid.get(uid);
  }

  public double totalHeat() {
    double sum = 0.0;
    for (Block b : blocks) sum += b.heat;
    return sum;
  }

  public Block blockAt(Translation2d p) {
    Objects.requireNonNull(p, "p");
    for (Block b : blocks) {
      if (b.contains(p)) return b;
    }
    return null;
  }

  public double heatAt(Translation2d p) {
    Objects.requireNonNull(p, "p");
    Block b = blockAt(p);
    if (b == null) return 0.0;

    List<Incoming> incoming = incomingByTo.get(b.uid);
    if (incoming == null || incoming.isEmpty()) return b.heat;

    switch (blendMode) {
      case MIN_T:
        return heatAtMinT(p, b, incoming);
      case WEIGHTED_SUM:
        return heatAtWeightedSum(p, b, incoming);
      case MAX_EDGE_INFLUENCE:
      default:
        return heatAtMaxInfluence(p, b, incoming);
    }
  }

  public boolean areTouching(String aUid, String bUid) {
    Block a = byUid.get(aUid);
    Block b = byUid.get(bUid);
    if (a == null || b == null) return false;
    return detectTouch(a, b, eps) != null;
  }

  public List<TransitionZone> zonesForTo(String toUid) {
    List<Incoming> inc = incomingByTo.get(toUid);
    if (inc == null || inc.isEmpty()) return Collections.emptyList();
    List<TransitionZone> out = new ArrayList<>(inc.size());
    for (Incoming i : inc) {
      out.add(new TransitionZone(i.from.uid, i.to.uid, i.side, i.thickness, i.scale));
    }
    return Collections.unmodifiableList(out);
  }

  public List<TransitionZone> zones() {
    List<TransitionZone> out = new ArrayList<>();
    for (Map.Entry<String, List<Incoming>> e : incomingByTo.entrySet()) {
      for (Incoming i : e.getValue()) {
        out.add(new TransitionZone(i.from.uid, i.to.uid, i.side, i.thickness, i.scale));
      }
    }
    return Collections.unmodifiableList(out);
  }

  private double heatAtMinT(Translation2d p, Block to, List<Incoming> incoming) {
    double bestT = Double.POSITIVE_INFINITY;
    double best = to.heat;
    for (Incoming inc : incoming) {
      if (!inc.contains(p)) continue;
      double t = inc.t(p);
      if (t < bestT) {
        bestT = t;
        best = lerp(inc.from.heat, to.heat, t);
      }
    }
    return best;
  }

  private double heatAtMaxInfluence(Translation2d p, Block to, List<Incoming> incoming) {
    double bestInf = 0.0;
    double bestHeat = to.heat;
    for (Incoming inc : incoming) {
      double inf = inc.influence(p);
      if (inf > bestInf) {
        bestInf = inf;
        bestHeat = inc.blendedHeatAt(p);
      }
    }
    return bestHeat;
  }

  private double heatAtWeightedSum(Translation2d p, Block to, List<Incoming> incoming) {
    double base = to.heat;
    double wBase = 1.0;
    double sum = base * wBase;
    double wSum = wBase;

    for (Incoming inc : incoming) {
      double inf = inc.influence(p);
      if (inf <= 0.0) continue;
      double h = inc.blendedHeatAt(p);
      double w = inf;
      sum += h * w;
      wSum += w;
    }
    return (wSum <= 1e-12) ? base : (sum / wSum);
  }

  private void buildIncoming() {
    incomingByTo.clear();
    for (Transition tr : transitions) {
      Block from = byUid.get(tr.from);
      Block to = byUid.get(tr.to);
      if (from == null || to == null) continue;

      Touch touch = detectTouch(from, to, eps);
      if (touch == null) continue;

      double s = clamp(tr.scale, 0.0, 1.0);
      double thickness;
      switch (touch.side) {
        case LEFT:
        case RIGHT:
          thickness = s * to.w();
          break;
        case TOP:
        case BOTTOM:
          thickness = s * to.h();
          break;
        default:
          thickness = 0.0;
          break;
      }

      Incoming inc = new Incoming(from, to, touch.side, Math.max(0.0, thickness), s);
      incomingByTo.computeIfAbsent(to.uid, k -> new ArrayList<>()).add(inc);
    }
  }

  private static final class Touch {
    final Side side;

    Touch(Side side) {
      this.side = side;
    }
  }

  private static Touch detectTouch(Block from, Block to, double eps) {
    double fx0 = from.x0(), fx1 = from.x1();
    double fy0 = from.y0(), fy1 = from.y1();
    double tx0 = to.x0(), tx1 = to.x1();
    double ty0 = to.y0(), ty1 = to.y1();

    boolean yOverlap = overlapPositive(fy0, fy1, ty0, ty1);
    boolean xOverlap = overlapPositive(fx0, fx1, tx0, tx1);

    if (yOverlap && nearlyEqual(fx1, tx0, eps)) return new Touch(Side.LEFT);
    if (yOverlap && nearlyEqual(fx0, tx1, eps)) return new Touch(Side.RIGHT);
    if (xOverlap && nearlyEqual(fy1, ty0, eps)) return new Touch(Side.BOTTOM);
    if (xOverlap && nearlyEqual(fy0, ty1, eps)) return new Touch(Side.TOP);

    return null;
  }

  private static boolean overlapPositive(double a0, double a1, double b0, double b1) {
    double lo = Math.max(Math.min(a0, a1), Math.min(b0, b1));
    double hi = Math.min(Math.max(a0, a1), Math.max(b0, b1));
    return hi - lo > 1e-12;
  }

  private static boolean nearlyEqual(double a, double b, double eps) {
    return Math.abs(a - b) <= eps;
  }

  private static double clamp(double v, double lo, double hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
  }

  private static double lerp(double a, double b, double t) {
    return a + (b - a) * clamp(t, 0.0, 1.0);
  }

  private static String requireUid(String uid) {
    if (uid == null) throw new IllegalArgumentException("uid is null");
    String u = uid.trim();
    if (u.isEmpty()) throw new IllegalArgumentException("uid is empty");
    return u;
  }
}
