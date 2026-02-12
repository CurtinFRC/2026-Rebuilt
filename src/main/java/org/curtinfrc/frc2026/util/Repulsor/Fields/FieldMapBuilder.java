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

package org.curtinfrc.frc2026.util.Repulsor.Fields;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Distance;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Objects;
import java.util.Set;
import java.util.function.Predicate;

import org.curtinfrc.frc2026.util.Repulsor.Setpoints.Setpoints.RepulsorSetpoint;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.FieldTrackerCore;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Model.Alliance;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Model.GameElement;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Model.GameElementModel;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Model.GameObject;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Model.Pipe;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Model.PrimitiveObject;

public final class FieldMapBuilder {
  public enum CategorySpec {
    kScore,
    kCollect,
    kEndgame
  }

  public static final class ElementSpec {
    Alliance alliance = Alliance.kBlue;
    int capacity = 1;
    Pose3d pose = new Pose3d();
    final List<PrimitiveObject> primitives = new ArrayList<>();
    Predicate<GameObject> filter = go -> true;
    RepulsorSetpoint related;
    CategorySpec category = CategorySpec.kScore;
  }

  private final FieldTrackerCore ft;
  private final List<GameElement> elements = new ArrayList<>();
  private ElementSpec spec;

  public FieldMapBuilder(FieldTrackerCore ft) {
    this.ft = Objects.requireNonNull(ft);
  }

  private ElementSpec s() {
    if (spec == null) throw new IllegalStateException("Call begin() first");
    return spec;
  }

  public FieldMapBuilder begin() {
    spec = new ElementSpec();
    return this;
  }

  public FieldMapBuilder alliance(Alliance a) {
    s().alliance = a;
    return this;
  }

  public FieldMapBuilder capacity(int c) {
    s().capacity = c;
    return this;
  }

  public FieldMapBuilder pose(Pose3d p) {
    s().pose = p;
    return this;
  }

  public FieldMapBuilder rotate(double rollRad, double pitchRad, double yawRad) {
    Pose3d p = s().pose;
    s().pose = new Pose3d(p.getX(), p.getY(), p.getZ(), new Rotation3d(rollRad, pitchRad, yawRad));
    return this;
  }

  public FieldMapBuilder translate(double dx, double dy, double dz) {
    Pose3d p = s().pose;
    s().pose = new Pose3d(p.getX() + dx, p.getY() + dy, p.getZ() + dz, p.getRotation());
    return this;
  }

  public FieldMapBuilder category(CategorySpec c) {
    s().category = c != null ? c : CategorySpec.kScore;
    return this;
  }

  public FieldMapBuilder primitivePipe(
      Distance radius, double rollRad, double pitchRad, double yawRad) {
    Pose3d p = s().pose;
    s().primitives
        .add(
            new Pipe(
                new Pose3d(p.getX(), p.getY(), p.getZ(), new Rotation3d(rollRad, pitchRad, yawRad)),
                radius,
                Radians.of(yawRad)));
    return this;
  }

  public FieldMapBuilder primitiveFloorSquare(
      double sideMeters, double zMinMeters, double zMaxMeters) {
    if (sideMeters <= 0) throw new IllegalArgumentException("sideMeters must be > 0");
    if (zMaxMeters < zMinMeters)
      throw new IllegalArgumentException("zMaxMeters must be >= zMinMeters");
    Pose3d p = s().pose;
    double cx = p.getX();
    double cy = p.getY();
    double half = sideMeters * 0.5;
    s().primitives
        .add(
            new PrimitiveObject() {
              @Override
              public boolean intersects(Pose3d pos) {
                double x = pos.getX();
                double y = pos.getY();
                double z = pos.getZ();
                if (z < zMinMeters || z > zMaxMeters) return false;
                return Math.abs(x - cx) <= half && Math.abs(y - cy) <= half;
              }
            });
    return this;
  }

  public FieldMapBuilder filter(Predicate<GameObject> f) {
    s().filter = f != null ? f : (go -> true);
    return this;
  }

  private static String canonType(Object raw) {
    if (raw == null) return "";
    String t = String.valueOf(raw).trim();
    if (t.isEmpty()) return "";
    t = t.replace(' ', '_').replace('-', '_');
    if (t.length() >= 2
        && (t.charAt(0) == 'k' || t.charAt(0) == 'K')
        && Character.isUpperCase(t.charAt(1))) {
      t = t.substring(1);
    }
    t = t.toLowerCase();
    if (t.startsWith("k_")) t = t.substring(2);
    if (t.startsWith("k")) t = t.substring(1);
    return t;
  }

  public FieldMapBuilder filterType(String... allowed) {
    Set<String> set = new HashSet<>();
    if (allowed != null) {
      for (String a : allowed) {
        if (a == null) continue;
        String c = canonType(a);
        if (!c.isEmpty()) set.add(c);
      }
    }
    if (set.isEmpty()) {
      s().filter = go -> true;
      return this;
    }
    s().filter =
        go -> {
          if (go == null) return false;
          String c = canonType(go.getType());
          return set.contains(c);
        };
    return this;
  }

  public FieldMapBuilder related(RepulsorSetpoint sp) {
    s().related = sp;
    return this;
  }

  public FieldMapBuilder add() {
    ElementSpec es = s();
    PrimitiveObject[] prim = es.primitives.toArray(new PrimitiveObject[0]);
    GameElementModel model = new GameElementModel(es.pose, prim);
    elements.add(
        new GameElement(es.alliance, es.capacity, model, es.filter, es.related, es.category));
    spec = null;
    return this;
  }

  public FieldMapBuilder bulk(
      List<Pose3d> poses,
      Alliance alliance,
      int capacity,
      Distance radius,
      double yawRad,
      Predicate<GameObject> filter,
      List<RepulsorSetpoint> related,
      CategorySpec category) {
    int n = poses != null ? poses.size() : 0;
    int r = related != null ? related.size() : 0;
    int m = r > 0 ? Math.min(n, r) : n;
    for (int i = 0; i < m; i++) {
      begin()
          .alliance(alliance)
          .capacity(capacity)
          .pose(poses.get(i))
          .primitivePipe(radius, 0, 0, yawRad)
          .filter(filter)
          .related(r > 0 ? related.get(i) : null)
          .category(category)
          .add();
    }
    return this;
  }

  public FieldMapBuilder mirrorX(double xAxis) {
    List<GameElement> mirrored = new ArrayList<>();
    for (GameElement e : elements) {
      Pose3d p = e.getModel().getPosition();
      Pose3d mp = new Pose3d(2 * xAxis - p.getX(), p.getY(), p.getZ(), p.getRotation());
      PrimitiveObject[] prims = e.getModel().getComposition();
      List<PrimitiveObject> nprims = new ArrayList<>();
      for (PrimitiveObject pr : prims) {
        if (pr instanceof Pipe) {
          Pipe pipe = (Pipe) pr;
          Pose3d pp = pipe.getPosition();
          nprims.add(
              new Pipe(
                  new Pose3d(2 * xAxis - pp.getX(), pp.getY(), pp.getZ(), pp.getRotation()),
                  pipe.getRadius(),
                  pipe.getAngle()));
        }
      }
      GameElementModel model = new GameElementModel(mp, nprims.toArray(new PrimitiveObject[0]));
      GameElement m =
          new GameElement(
              e.getAlliance(),
              e.getMaxContained(),
              model,
              e::filter,
              e.getRelatedPoint().orElse(null),
              e.getCategory());
      mirrored.add(m);
    }
    elements.addAll(mirrored);
    return this;
  }

  public GameElement[] build() {
    return elements.toArray(new GameElement[0]);
  }

  public static Distance small() {
    return Meters.of(0.20);
  }

  public static Distance medium() {
    return Meters.of(0.30);
  }

  public static Distance large() {
    return Meters.of(0.40);
  }

  public static Distance tiny() {
    return Meters.of(0.10);
  }
}
