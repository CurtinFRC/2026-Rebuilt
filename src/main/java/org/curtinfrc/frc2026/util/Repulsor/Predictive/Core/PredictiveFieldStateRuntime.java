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

final class PredictiveFieldStateRuntime {
  private final PredictiveFieldStateOps ops = new PredictiveFieldStateOps();

  static boolean error() {
    return PredictiveFieldStateOps.error();
  }

  CollectProbe probeCollect(Translation2d p) {
    return ops.probeCollect(p);
  }

  CollectProbe probeCollect(Translation2d p, double countR) {
    return ops.probeCollect(p, countR);
  }

  boolean footprintHasCollectResource(Translation2d center, double cellM) {
    return ops.footprintHasCollectResource(center, cellM);
  }

  boolean footprintHasFuel(Translation2d center, double cellM) {
    return ops.footprintHasFuel(center, cellM);
  }

  void markCollectDepleted(Translation2d p, double cellM, double strength) {
    ops.markCollectDepleted(p, cellM, strength);
  }

  void registerResourceSpec(String type, ResourceSpec spec) {
    ops.registerResourceSpec(type, spec);
  }

  void registerOtherTypeWeight(String type, double weight) {
    ops.registerOtherTypeWeight(type, weight);
  }

  void setCollectResourceTypes(Set<String> types) {
    ops.setCollectResourceTypes(types);
  }

  Set<String> getCollectResourceTypes() {
    return ops.getCollectResourceTypes();
  }

  void addCollectResourceType(String type) {
    ops.addCollectResourceType(type);
  }

  void removeCollectResourceType(String type) {
    ops.removeCollectResourceType(type);
  }

  boolean isCollectResourceType(String type) {
    return ops.isCollectResourceType(type);
  }

  void setCollectResourcePositionFilter(Predicate<Translation2d> filter) {
    ops.setCollectResourcePositionFilter(filter);
  }

  void setDynamicObjects(List<DynamicObject> objs) {
    ops.setDynamicObjects(objs);
  }

  void setWorld(List<GameElement> elements, Alliance ours) {
    ops.setWorld(elements, ours);
  }

  void updateAlly(int id, Translation2d pos, Translation2d velHint, Double speedCap) {
    ops.updateAlly(id, pos, velHint, speedCap);
  }

  void updateEnemy(int id, Translation2d pos, Translation2d velHint, Double speedCap) {
    ops.updateEnemy(id, pos, velHint, speedCap);
  }

  void clearStale(double maxAgeS) {
    ops.clearStale(maxAgeS);
  }

  List<Candidate> rank(Translation2d ourPos, double ourSpeedCap, CategorySpec cat, int limit) {
    return ops.rank(ourPos, ourSpeedCap, cat, limit);
  }

  List<RepulsorSetpoint> rankSetpoints(
      Translation2d ourPos, double ourSpeedCap, CategorySpec cat, int limit) {
    return ops.rankSetpoints(ourPos, ourSpeedCap, cat, limit);
  }

  int resourceObservationCount() {
    return ops.resourceObservationCount();
  }

  Translation2d snapToCollectCentroid(Translation2d seed, double r, double minMass) {
    return ops.snapToCollectCentroid(seed, r, minMass);
  }

  Translation2d nearestCollectResource(Translation2d p, double maxDist) {
    return ops.nearestCollectResource(p, maxDist);
  }

  PointCandidate rankCollectNearest(
      Translation2d ourPos,
      double ourSpeedCap,
      Translation2d[] points,
      double cellM,
      int goalUnits,
      int limit) {
    return ops.rankCollectNearest(ourPos, ourSpeedCap, points, cellM, goalUnits, limit);
  }

  PointCandidate rankCollectHierarchical(
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

  Translation2d bestCollectHotspot(Translation2d[] points, double cellM) {
    return ops.bestCollectHotspot(points, cellM);
  }

  PointCandidate rankCollectPoints(
      Translation2d ourPos, double ourSpeedCap, Translation2d[] points, int goalUnits, int limit) {
    return ops.rankCollectPoints(ourPos, ourSpeedCap, points, goalUnits, limit);
  }
}
