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

package org.curtinfrc.frc2026.util.Repulsor.Predictive;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.function.Predicate;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.DynamicObject;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.ResourceSpec;

public final class SpatialDyn {
  static long key(int cx, int cy) {
    return (((long) cx) << 32) ^ (cy & 0xffffffffL);
  }

  final List<DynamicObject> all;
  public final List<DynamicObject> resources;
  final List<DynamicObject> others;
  final HashMap<String, ResourceSpec> specs;
  final HashMap<String, Double> otherWeights;
  final HashSet<String> collectTypes;
  final Predicate<Translation2d> collectFilter;

  final double cellM;
  final HashMap<Long, ArrayList<DynamicObject>> resCells;
  final HashMap<Long, ArrayList<DynamicObject>> othCells;

  SpatialDyn(
      List<DynamicObject> dyn,
      HashMap<String, ResourceSpec> specsIn,
      HashMap<String, Double> otherWeightsIn,
      Set<String> collectTypesIn,
      Predicate<Translation2d> collectFilterIn) {
    this.all = dyn != null ? dyn : List.of();
    this.specs = new HashMap<>();
    if (specsIn != null) {
      for (var e : specsIn.entrySet()) {
        if (e.getKey() == null || e.getValue() == null) continue;
        this.specs.put(e.getKey().toLowerCase(), e.getValue());
      }
    }
    this.otherWeights = new HashMap<>();
    if (otherWeightsIn != null) {
      for (var e : otherWeightsIn.entrySet()) {
        if (e.getKey() == null || e.getKey().isEmpty()) continue;
        this.otherWeights.put(e.getKey().toLowerCase(), Math.max(0.0, e.getValue()));
      }
    }
    this.collectTypes = new HashSet<>();
    if (collectTypesIn != null) {
      for (String type : collectTypesIn) {
        if (type == null || type.isEmpty()) continue;
        this.collectTypes.add(type.toLowerCase());
      }
    }
    if (this.collectTypes.isEmpty()) {
      this.collectTypes.add("fuel");
    }
    this.collectFilter = collectFilterIn != null ? collectFilterIn : (p -> true);

    this.cellM = 0.50;
    this.resCells = new HashMap<>(128);
    this.othCells = new HashMap<>(128);

    ArrayList<DynamicObject> res = new ArrayList<>();
    ArrayList<DynamicObject> oth = new ArrayList<>();

    for (DynamicObject o : this.all) {
      if (o == null || o.pos == null) continue;
      if (!PredictiveFieldStateCore.error()
          && o.ageS > PredictiveFieldStateCore.RESOURCE_HARD_MAX_AGE_S) continue;

      String ty = o.type != null ? o.type.toLowerCase() : "unknown";
      if (this.collectTypes.contains(ty) && !this.collectFilter.test(o.pos)) continue;
      if (this.specs.containsKey(ty)) {
        res.add(o);
        addTo(resCells, o.pos, o);
      } else {
        oth.add(o);
        addTo(othCells, o.pos, o);
      }
    }

    this.resources = res;
    this.others = oth;
  }

  private void addTo(
      HashMap<Long, ArrayList<DynamicObject>> map, Translation2d p, DynamicObject o) {
    int cx = (int) Math.floor(p.getX() / Math.max(1e-6, cellM));
    int cy = (int) Math.floor(p.getY() / Math.max(1e-6, cellM));
    long k = key(cx, cy);
    map.computeIfAbsent(k, kk -> new ArrayList<>()).add(o);
  }

  private Iterable<ArrayList<DynamicObject>> cellsInRadius(
      Translation2d p, double r, HashMap<Long, ArrayList<DynamicObject>> map) {
    int cx0 = (int) Math.floor(p.getX() / Math.max(1e-6, cellM));
    int cy0 = (int) Math.floor(p.getY() / Math.max(1e-6, cellM));
    int rad = (int) Math.ceil(r / Math.max(1e-6, cellM));
    ArrayList<ArrayList<DynamicObject>> out = new ArrayList<>((2 * rad + 1) * (2 * rad + 1));
    for (int dx = -rad; dx <= rad; dx++) {
      for (int dy = -rad; dy <= rad; dy++) {
        ArrayList<DynamicObject> c = map.get(key(cx0 + dx, cy0 + dy));
        if (c != null) out.add(c);
      }
    }
    return out;
  }

  public int countResourcesWithin(Translation2d p, double r) {
    if (p == null) return 0;
    double rr2 = r * r;
    int c = 0;
    for (ArrayList<DynamicObject> cell : cellsInRadius(p, r, resCells)) {
      for (int i = 0; i < cell.size(); i++) {
        DynamicObject o = cell.get(i);
        if (o == null || o.pos == null) continue;
        double dx = o.pos.getX() - p.getX();
        double dy = o.pos.getY() - p.getY();
        if (dx * dx + dy * dy <= rr2) c++;
      }
    }
    return c;
  }

  public double totalEvidence() {
    if (resources.isEmpty() || specs.isEmpty()) return 0.0;
    double sum = 0.0;
    for (DynamicObject o : resources) {
      if (o == null || o.pos == null || o.type == null) continue;
      ResourceSpec s = specs.get(o.type.toLowerCase());
      if (s == null) continue;
      double ageW =
          PredictiveFieldStateCore.error()
              ? 1.0
              : Math.exp(-PredictiveFieldStateCore.COLLECT_AGE_DECAY * Math.max(0.0, o.ageS));
      sum += Math.max(0.0, s.unitValue) * ageW;
    }
    return Math.max(0.0, sum);
  }

  public double evidenceMassWithin(Translation2d p, double r) {
    if (p == null || resources.isEmpty() || specs.isEmpty()) return 0.0;
    double rr2 = r * r;
    double sum = 0.0;
    for (ArrayList<DynamicObject> cell : cellsInRadius(p, r, resCells)) {
      for (int i = 0; i < cell.size(); i++) {
        DynamicObject o = cell.get(i);
        if (o == null || o.pos == null || o.type == null) continue;
        double dx = o.pos.getX() - p.getX();
        double dy = o.pos.getY() - p.getY();
        double d2 = dx * dx + dy * dy;
        if (d2 > rr2) continue;

        ResourceSpec s = specs.get(o.type.toLowerCase());
        if (s == null) continue;

        double ageW =
            PredictiveFieldStateCore.error()
                ? 1.0
                : Math.exp(-PredictiveFieldStateCore.COLLECT_AGE_DECAY * Math.max(0.0, o.ageS));
        sum += Math.max(0.0, s.unitValue) * ageW;
      }
    }
    return Math.max(0.0, sum);
  }

  public double valueAt(Translation2d p) {
    if (p == null || resources.isEmpty() || specs.isEmpty()) return 0.0;

    double r = 2.0;
    double sum = 0.0;

    for (ArrayList<DynamicObject> cell : cellsInRadius(p, r, resCells)) {
      for (int i = 0; i < cell.size(); i++) {
        DynamicObject o = cell.get(i);
        if (o == null || o.pos == null || o.type == null) continue;

        ResourceSpec s = specs.get(o.type.toLowerCase());
        if (s == null) continue;

        double dx = o.pos.getX() - p.getX();
        double dy = o.pos.getY() - p.getY();
        double d2 = dx * dx + dy * dy;

        double age = Math.max(0.0, o.ageS);
        double ageW =
            PredictiveFieldStateCore.error()
                ? 1.0
                : Math.exp(-PredictiveFieldStateCore.COLLECT_AGE_DECAY * age);

        double sigmaBase =
            Math.max(
                PredictiveFieldStateCore.RESOURCE_SIGMA_MIN,
                Math.min(
                    PredictiveFieldStateCore.RESOURCE_SIGMA_ABS_MAX,
                    Math.min(s.sigmaM * PredictiveFieldStateCore.RESOURCE_SIGMA_REL_MAX, 2.0)));

        double sigma = sigmaBase * (0.70 + 0.35 * Math.exp(-0.85 * age));
        sigma =
            Math.max(
                PredictiveFieldStateCore.RESOURCE_SIGMA_MIN,
                Math.min(PredictiveFieldStateCore.RESOURCE_SIGMA_ABS_MAX, sigma));

        double sig2 = sigma * sigma;

        double w = Math.max(0.0, s.unitValue) * ageW;
        double k = Math.exp(-0.5 * d2 / Math.max(1e-6, sig2));

        sum += w * k;
      }
    }

    return Math.max(0.0, sum);
  }

  public double valueInSquare(Translation2d center, double half) {
    if (center == null || resources.isEmpty() || specs.isEmpty()) return 0.0;
    double x0 = center.getX() - half;
    double x1 = center.getX() + half;
    double y0 = center.getY() - half;
    double y1 = center.getY() + half;

    double r = Math.sqrt(2.0) * half + 0.20;
    double sum = 0.0;

    for (ArrayList<DynamicObject> cell : cellsInRadius(center, r, resCells)) {
      for (int i = 0; i < cell.size(); i++) {
        DynamicObject o = cell.get(i);
        if (o == null || o.pos == null || o.type == null) continue;
        double x = o.pos.getX();
        double y = o.pos.getY();
        if (x < x0 || x > x1 || y < y0 || y > y1) continue;

        ResourceSpec s = specs.get(o.type.toLowerCase());
        if (s == null) continue;

        double ageW =
            PredictiveFieldStateCore.error()
                ? 1.0
                : Math.exp(-PredictiveFieldStateCore.COLLECT_AGE_DECAY * Math.max(0.0, o.ageS));
        sum += Math.max(0.0, s.unitValue) * ageW;
      }
    }

    return Math.max(0.0, sum);
  }

  public Translation2d centroidResourcesWithin(Translation2d seed, double r, double minMass) {
    if (seed == null || resources.isEmpty() || specs.isEmpty()) return null;

    double rr2 = r * r;
    double sx = 0.0;
    double sy = 0.0;
    double sw = 0.0;

    for (ArrayList<DynamicObject> cell : cellsInRadius(seed, r, resCells)) {
      for (int i = 0; i < cell.size(); i++) {
        DynamicObject o = cell.get(i);
        if (o == null || o.pos == null || o.type == null) continue;

        ResourceSpec s = specs.get(o.type.toLowerCase());
        if (s == null) continue;

        double dx = o.pos.getX() - seed.getX();
        double dy = o.pos.getY() - seed.getY();
        double d2 = dx * dx + dy * dy;
        if (d2 > rr2) continue;

        double age = Math.max(0.0, o.ageS);
        double ageW =
            PredictiveFieldStateCore.error()
                ? 1.0
                : Math.exp(-PredictiveFieldStateCore.COLLECT_AGE_DECAY * age);

        double sigmaBase =
            Math.max(
                PredictiveFieldStateCore.RESOURCE_SIGMA_MIN,
                Math.min(PredictiveFieldStateCore.RESOURCE_SIGMA_ABS_MAX, s.sigmaM));
        double sigma = sigmaBase * (0.70 + 0.35 * Math.exp(-0.85 * age));
        sigma =
            Math.max(
                PredictiveFieldStateCore.RESOURCE_SIGMA_MIN,
                Math.min(PredictiveFieldStateCore.RESOURCE_SIGMA_ABS_MAX, sigma));

        double sig2 = sigma * sigma;

        double w = Math.max(0.0, s.unitValue) * ageW * Math.exp(-0.5 * d2 / Math.max(1e-6, sig2));

        sx += o.pos.getX() * w;
        sy += o.pos.getY() * w;
        sw += w;
      }
    }

    if (sw < Math.max(0.0, minMass)) return null;
    return new Translation2d(sx / sw, sy / sw);
  }

  public Translation2d nearestResourceTo(Translation2d p, double maxDist) {
    if (p == null || resources.isEmpty()) return null;
    double bestD2 = maxDist * maxDist;
    Translation2d best = null;

    for (ArrayList<DynamicObject> cell : cellsInRadius(p, maxDist, resCells)) {
      for (int i = 0; i < cell.size(); i++) {
        DynamicObject o = cell.get(i);
        if (o == null || o.pos == null) continue;
        double dx = o.pos.getX() - p.getX();
        double dy = o.pos.getY() - p.getY();
        double d2 = dx * dx + dy * dy;
        if (d2 <= bestD2) {
          bestD2 = d2;
          best = o.pos;
        }
      }
    }

    return best;
  }

  public double otherDensity(Translation2d p, double sigma) {
    if (p == null || others.isEmpty()) return 0.0;
    double s = Math.max(0.05, sigma);
    double r = 3.0 * s;
    double agg = 0.0;
    double wSum = 0.0;

    for (ArrayList<DynamicObject> cell : cellsInRadius(p, r, othCells)) {
      for (int i = 0; i < cell.size(); i++) {
        DynamicObject o = cell.get(i);
        if (o == null || o.pos == null) continue;
        double d = o.pos.getDistance(p);

        double w = 1.0;
        if (o.type != null) {
          Double ow = otherWeights.get(o.type.toLowerCase());
          if (ow != null) w = Math.max(0.0, ow);
        }

        agg += w * densityKernel(d, s);
        wSum += Math.max(0.05, w);
      }
    }

    if (wSum <= 1e-9) return 0.0;
    return Math.max(0.0, agg / wSum);
  }

  public double localAvoidPenalty(Translation2d p, double r) {
    if (p == null || others.isEmpty()) return 0.0;
    double rr = Math.max(0.05, r);
    double rr2 = rr * rr;

    double pen = 0.0;
    double wSum = 0.0;

    for (ArrayList<DynamicObject> cell : cellsInRadius(p, rr, othCells)) {
      for (int i = 0; i < cell.size(); i++) {
        DynamicObject o = cell.get(i);
        if (o == null || o.pos == null) continue;
        double dx = o.pos.getX() - p.getX();
        double dy = o.pos.getY() - p.getY();
        double d2 = dx * dx + dy * dy;
        if (d2 > rr2) continue;
        double d = Math.sqrt(Math.max(0.0, d2));

        double w = 1.0;
        if (o.type != null) {
          Double ow = otherWeights.get(o.type.toLowerCase());
          if (ow != null) w = Math.max(0.0, ow);
        }

        double k = Math.exp(-0.5 * (d * d) / Math.max(1e-6, (0.45 * 0.45)));
        pen += w * k;
        wSum += Math.max(0.05, w);
      }
    }

    if (wSum <= 1e-9) return 0.0;
    return Math.min(1.25, pen / wSum);
  }

  private static double densityKernel(double d, double sigma) {
    double s2 = sigma * sigma;
    return Math.exp(-0.5 * (d * d) / Math.max(1e-6, s2));
  }
}
