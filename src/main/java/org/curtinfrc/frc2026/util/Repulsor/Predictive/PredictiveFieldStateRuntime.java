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
import java.util.List;
import java.util.Set;
import java.util.function.Predicate;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldMapBuilder.CategorySpec;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.Candidate;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.CollectProbe;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.DynamicObject;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.PointCandidate;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Model.ResourceSpec;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.RepulsorSetpoint;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Model.Alliance;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Model.GameElement;

public class PredictiveFieldStateRuntime {
  private final PredictiveFieldStateOps ops = new PredictiveFieldStateOps();

  static final double COLLECT_AGE_DECAY = 0.75;
  static final double RESOURCE_SIGMA_ABS_MAX = 0.45;
  static final double RESOURCE_SIGMA_REL_MAX = 1.25;
  static final double RESOURCE_SIGMA_MIN = 0.06;
  static final double RESOURCE_HARD_MAX_AGE_S = 0.95;

  static boolean error() {
    return PredictiveFieldStateOps.error();
  }

  public CollectProbe probeCollect(Translation2d p) {
    return ops.probeCollect(p);
  }

  public CollectProbe probeCollect(Translation2d p, double countR) {
    return ops.probeCollect(p, countR);
  }

  public boolean footprintHasCollectResource(Translation2d center, double cellM) {
    return ops.footprintHasCollectResource(center, cellM);
  }

  public boolean footprintHasFuel(Translation2d center, double cellM) {
    return ops.footprintHasFuel(center, cellM);
  }

  public void markCollectDepleted(Translation2d p, double cellM, double strength) {
    ops.markCollectDepleted(p, cellM, strength);
  }

  public void registerResourceSpec(String type, ResourceSpec spec) {
    ops.registerResourceSpec(type, spec);
  }

  public void registerOtherTypeWeight(String type, double weight) {
    ops.registerOtherTypeWeight(type, weight);
  }

  public void setCollectResourceTypes(Set<String> types) {
    ops.setCollectResourceTypes(types);
  }

  Set<String> getCollectResourceTypes() {
    return ops.getCollectResourceTypes();
  }

  public void addCollectResourceType(String type) {
    ops.addCollectResourceType(type);
  }

  void removeCollectResourceType(String type) {
    ops.removeCollectResourceType(type);
  }

  public boolean isCollectResourceType(String type) {
    return ops.isCollectResourceType(type);
  }

  public void setCollectResourcePositionFilter(Predicate<Translation2d> filter) {
    ops.setCollectResourcePositionFilter(filter);
  }

  public void setDynamicObjects(List<DynamicObject> objs) {
    ops.setDynamicObjects(objs);
  }

  public void setWorld(List<GameElement> elements, Alliance ours) {
    ops.setWorld(elements, ours);
  }

  public void updateAlly(int id, Translation2d pos, Translation2d velHint, Double speedCap) {
    ops.updateAlly(id, pos, velHint, speedCap);
  }

  public void updateEnemy(int id, Translation2d pos, Translation2d velHint, Double speedCap) {
    ops.updateEnemy(id, pos, velHint, speedCap);
  }

  public void clearStale(double maxAgeS) {
    ops.clearStale(maxAgeS);
  }

  public List<Candidate> rank(
      Translation2d ourPos, double ourSpeedCap, CategorySpec cat, int limit) {
    return ops.rank(ourPos, ourSpeedCap, cat, limit);
  }

  public List<RepulsorSetpoint> rankSetpoints(
      Translation2d ourPos, double ourSpeedCap, CategorySpec cat, int limit) {
    return ops.rankSetpoints(ourPos, ourSpeedCap, cat, limit);
  }

  public int resourceObservationCount() {
    return ops.resourceObservationCount();
  }

  public Translation2d snapToCollectCentroid(Translation2d seed, double r, double minMass) {
    return ops.snapToCollectCentroid(seed, r, minMass);
  }

  public Translation2d nearestCollectResource(Translation2d p, double maxDist) {
    return ops.nearestCollectResource(p, maxDist);
  }

  public PointCandidate rankCollectNearest(
      Translation2d ourPos,
      double ourSpeedCap,
      Translation2d[] points,
      double cellM,
      int goalUnits,
      int limit) {
    return ops.rankCollectNearest(ourPos, ourSpeedCap, points, cellM, goalUnits, limit);
  }

  public PointCandidate rankCollectHierarchical(
      Translation2d ourPos,
      double ourSpeedCap,
      Translation2d[] points,
      double cellM,
      int goalUnits,
      int coarseTopK,
      int refineGrid) {
    return ops.rankCollectHierarchical(
        ourPos, ourSpeedCap, points, cellM, goalUnits, coarseTopK, refineGrid);
  }

  public Translation2d bestCollectHotspot(Translation2d[] points, double cellM) {
    return ops.bestCollectHotspot(points, cellM);
  }

  public PointCandidate rankCollectPoints(
      Translation2d ourPos, double ourSpeedCap, Translation2d[] points, int goalUnits, int limit) {
    return ops.rankCollectPoints(ourPos, ourSpeedCap, points, goalUnits, limit);
  }
}
