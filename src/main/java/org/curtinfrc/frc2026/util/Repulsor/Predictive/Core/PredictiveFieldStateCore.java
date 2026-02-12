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
package org.curtinfrc.frc2026.util.Repulsor.Predictive.Core;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import java.util.Set;
import java.util.function.Predicate;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldMapBuilder.CategorySpec;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Core.Model.Candidate;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Core.Model.CollectProbe;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Core.Model.DynamicObject;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Core.Model.PointCandidate;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Core.Model.ResourceSpec;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.RepulsorSetpoint;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Core.Model.Alliance;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Core.Model.GameElement;

public class PredictiveFieldStateCore {
  static final double COLLECT_AGE_DECAY = 0.75;
  static final double RESOURCE_SIGMA_ABS_MAX = 0.45;
  static final double RESOURCE_SIGMA_REL_MAX = 1.25;
  static final double RESOURCE_SIGMA_MIN = 0.06;
  static final double RESOURCE_HARD_MAX_AGE_S = 0.95;

  private final PredictiveFieldStateRuntime engine = new PredictiveFieldStateRuntime();

  public static boolean error() {
    return PredictiveFieldStateRuntime.error();
  }

  public CollectProbe probeCollect(Translation2d p) {
    return engine.probeCollect(p);
  }

  public CollectProbe probeCollect(Translation2d p, double countR) {
    return engine.probeCollect(p, countR);
  }

  public boolean footprintHasCollectResource(Translation2d center, double cellM) {
    return engine.footprintHasCollectResource(center, cellM);
  }

  public boolean footprintHasFuel(Translation2d center, double cellM) {
    return engine.footprintHasFuel(center, cellM);
  }

  public void markCollectDepleted(Translation2d p, double cellM, double strength) {
    engine.markCollectDepleted(p, cellM, strength);
  }

  public void registerResourceSpec(String type, ResourceSpec spec) {
    engine.registerResourceSpec(type, spec);
  }

  public void registerOtherTypeWeight(String type, double weight) {
    engine.registerOtherTypeWeight(type, weight);
  }

  public void setCollectResourceTypes(Set<String> types) {
    engine.setCollectResourceTypes(types);
  }

  public Set<String> getCollectResourceTypes() {
    return engine.getCollectResourceTypes();
  }

  public void addCollectResourceType(String type) {
    engine.addCollectResourceType(type);
  }

  public void removeCollectResourceType(String type) {
    engine.removeCollectResourceType(type);
  }

  public boolean isCollectResourceType(String type) {
    return engine.isCollectResourceType(type);
  }

  public void setCollectResourcePositionFilter(Predicate<Translation2d> filter) {
    engine.setCollectResourcePositionFilter(filter);
  }

  public void setDynamicObjects(List<DynamicObject> objs) {
    engine.setDynamicObjects(objs);
  }

  public void setWorld(List<GameElement> elements, Alliance ours) {
    engine.setWorld(elements, ours);
  }

  public void updateAlly(int id, Translation2d pos, Translation2d velHint, Double speedCap) {
    engine.updateAlly(id, pos, velHint, speedCap);
  }

  public void updateEnemy(int id, Translation2d pos, Translation2d velHint, Double speedCap) {
    engine.updateEnemy(id, pos, velHint, speedCap);
  }

  public void clearStale(double maxAgeS) {
    engine.clearStale(maxAgeS);
  }

  public List<Candidate> rank(
      Translation2d ourPos, double ourSpeedCap, CategorySpec cat, int limit) {
    return engine.rank(ourPos, ourSpeedCap, cat, limit);
  }

  public List<RepulsorSetpoint> rankSetpoints(
      Translation2d ourPos, double ourSpeedCap, CategorySpec cat, int limit) {
    return engine.rankSetpoints(ourPos, ourSpeedCap, cat, limit);
  }

  public int resourceObservationCount() {
    return engine.resourceObservationCount();
  }

  public Translation2d snapToCollectCentroid(Translation2d seed, double r, double minMass) {
    return engine.snapToCollectCentroid(seed, r, minMass);
  }

  public Translation2d nearestCollectResource(Translation2d p, double maxDist) {
    return engine.nearestCollectResource(p, maxDist);
  }

  public PointCandidate rankCollectNearest(
      Translation2d ourPos,
      double ourSpeedCap,
      Translation2d[] points,
      double cellM,
      int goalUnits,
      int limit) {
    return engine.rankCollectNearest(ourPos, ourSpeedCap, points, cellM, goalUnits, limit);
  }

  public PointCandidate rankCollectHierarchical(
      Translation2d ourPos,
      double ourSpeedCap,
      Translation2d[] points,
      double cellM,
      int goalUnits,
      int coarseTopK,
      int refineGrid) {
    return engine.rankCollectHierarchical(
        ourPos, ourSpeedCap, points, cellM, goalUnits, coarseTopK, refineGrid);
  }

  public Translation2d bestCollectHotspot(Translation2d[] points, double cellM) {
    return engine.bestCollectHotspot(points, cellM);
  }

  public PointCandidate rankCollectPoints(
      Translation2d ourPos, double ourSpeedCap, Translation2d[] points, int goalUnits, int limit) {
    return engine.rankCollectPoints(ourPos, ourSpeedCap, points, goalUnits, limit);
  }
}
