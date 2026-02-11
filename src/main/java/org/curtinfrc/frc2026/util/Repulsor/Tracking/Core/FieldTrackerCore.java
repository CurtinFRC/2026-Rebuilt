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

package org.curtinfrc.frc2026.util.Repulsor.Tracking.Core;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.concurrent.ConcurrentHashMap;
import java.util.function.Predicate;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldLayoutProvider;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldMapBuilder.CategorySpec;
import org.curtinfrc.frc2026.util.Repulsor.Fields.Rebuilt2026;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Core.Model.Candidate;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Core.Model.DynamicObject;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Core.Model.ResourceSpec;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Core.PredictiveFieldStateCore;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.RepulsorSetpoint;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Core.Collect.FieldTrackerCollectPlanner;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Core.Internal.ObjectiveCache;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Core.Model.Alliance;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Core.Model.GameElement;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Core.Vision.FieldVision;

public class FieldTrackerCore {
  private static volatile FieldTrackerCore instance;
  private static volatile FieldLayoutProvider defaultProvider = new Rebuilt2026();
  private static final String DEFAULT_COLLECT_RESOURCE_TYPE = "fuel";

  public GameElement[] field_map;

  private final PredictiveFieldStateCore predictor;
  private final ObjectiveCache collectCache;
  private final FieldTrackerDynamicTracker dynamicTracker = new FieldTrackerDynamicTracker();
  private final FieldTrackerCollectPlanner collectPlanner;

  private final ConcurrentHashMap<String, String> typeAliases = new ConcurrentHashMap<>();

  public static FieldTrackerCore getInstance() {
    FieldTrackerCore local = instance;
    if (local == null) {
      synchronized (FieldTrackerCore.class) {
        local = instance;
        if (local == null) {
          local = new FieldTrackerCore(defaultProvider);
          instance = local;
        }
      }
    }
    return local;
  }

  public static void setDefaultProvider(FieldLayoutProvider provider) {
    if (provider == null) throw new IllegalArgumentException("provider cannot be null");
    defaultProvider = provider;
  }

  public static void resetInstance(FieldLayoutProvider provider) {
    if (provider == null) throw new IllegalArgumentException("provider cannot be null");
    synchronized (FieldTrackerCore.class) {
      instance = new FieldTrackerCore(provider);
    }
  }

  public FieldTrackerCore() {
    this(defaultProvider);
  }

  public FieldTrackerCore(FieldLayoutProvider provider) {
    if (provider == null) throw new IllegalArgumentException("provider cannot be null");

    this.field_map = provider.build(this);
    this.predictor = new PredictiveFieldStateCore();
    this.collectCache = new ObjectiveCache();
    this.collectPlanner =
        new FieldTrackerCollectPlanner(
            predictor,
            () -> collectCache.points,
            this::snapshotDynamics,
            this::isCollectResourceType);

    rebuildObjectiveCaches();
    configureCollectResourceProfile(
        DEFAULT_COLLECT_RESOURCE_TYPE, new ResourceSpec(0.075, 1.0, 0.95));
  }

  public void resetAll() {
    collectPlanner.resetAll();
  }

  public void registerTypeAlias(String from, String to) {
    if (from == null || from.isEmpty()) {
      throw new IllegalArgumentException("from cannot be null/empty");
    }
    if (to == null || to.isEmpty()) throw new IllegalArgumentException("to cannot be null/empty");
    typeAliases.put(from, to);
  }

  public String canonicalizeType(String type) {
    if (type == null || type.isEmpty()) return "unknown";
    String t = type;
    for (int i = 0; i < 4; i++) {
      String next = typeAliases.get(t);
      if (next == null || next.equals(t)) break;
      t = next;
    }
    return t;
  }

  public PredictiveFieldStateCore getPredictor() {
    return predictor;
  }

  private boolean isCollectResourceType(String type) {
    if (type == null || type.isEmpty()) return false;
    return predictor != null
        ? predictor.isCollectResourceType(type)
        : DEFAULT_COLLECT_RESOURCE_TYPE.equalsIgnoreCase(type);
  }

  public void configureCollectResourceProfile(String type, ResourceSpec resourceSpec) {
    if (type == null || type.isEmpty()) throw new IllegalArgumentException("type cannot be empty");
    if (resourceSpec == null) throw new IllegalArgumentException("resourceSpec cannot be null");
    predictor.registerResourceSpec(type, resourceSpec);
    predictor.addCollectResourceType(type);
  }

  public void updatePredictorWorld(Alliance ours) {
    GameElement[] fm = field_map;
    if (fm == null || fm.length == 0) {
      predictor.setWorld(List.of(), ours);
      predictor.setDynamicObjects(List.of());
      return;
    }
    ArrayList<GameElement> list = new ArrayList<>(fm.length);
    Collections.addAll(list, fm);
    predictor.setWorld(list, ours);
    predictor.setDynamicObjects(snapshotDynamics());
  }

  public void rebuild(FieldLayoutProvider provider) {
    if (provider == null) throw new IllegalArgumentException("provider cannot be null");
    this.field_map = provider.build(this);
    rebuildObjectiveCaches();
  }

  public GameElement[] getFieldMap() {
    GameElement[] fm = field_map;
    if (fm == null) return new GameElement[0];
    GameElement[] copy = new GameElement[fm.length];
    System.arraycopy(fm, 0, copy, 0, fm.length);
    return copy;
  }

  public List<GameElement> getAvailableElements(Predicate<GameElement> pred) {
    GameElement[] fm = field_map;
    if (fm == null || fm.length == 0) return List.of();
    ArrayList<GameElement> out = new ArrayList<>(fm.length);
    for (GameElement e : fm) {
      if (e == null) continue;
      if (!e.isAtCapacity() && (pred == null || pred.test(e))) out.add(e);
    }
    return out;
  }

  public List<RepulsorSetpoint> getScoringCandidates(
      Alliance alliance, Translation2d from, CategorySpec cat) {
    GameElement[] fm = field_map;
    if (fm == null || fm.length == 0) return List.of();
    ArrayList<GameElement> elems = new ArrayList<>(fm.length);
    for (GameElement e : fm) {
      if (e == null) continue;
      if (e.getAlliance() == alliance
          && !e.isAtCapacity()
          && e.getRelatedPoint().isPresent()
          && (cat == null || e.getCategory() == cat)) {
        elems.add(e);
      }
    }

    elems.sort(
        Comparator.comparingDouble(
            e ->
                from.getDistance(
                    new Translation2d(
                        e.getModel().getPosition().getX(), e.getModel().getPosition().getY()))));

    ArrayList<RepulsorSetpoint> out = new ArrayList<>(elems.size());
    for (GameElement e : elems) {
      e.getRelatedPoint().ifPresent(out::add);
    }
    return out;
  }

  public void predictorClearStale(double maxAgeS) {
    predictor.clearStale(maxAgeS);
  }

  public void predictorUpdateAlly(
      int id, Translation2d pos, Translation2d velHint, Double speedCap) {
    predictor.updateAlly(id, pos, velHint, speedCap);
  }

  public void predictorUpdateEnemy(
      int id, Translation2d pos, Translation2d velHint, Double speedCap) {
    predictor.updateEnemy(id, pos, velHint, speedCap);
  }

  public List<Candidate> getPredictedCandidates(
      Alliance alliance, Translation2d ourPos, double ourSpeedCap, CategorySpec cat, int limit) {
    updatePredictorWorld(alliance);
    return predictor.rank(ourPos, ourSpeedCap, cat, limit);
  }

  public List<RepulsorSetpoint> getPredictedSetpoints(
      Alliance alliance, Translation2d ourPos, double ourSpeedCap, CategorySpec cat, int limit) {
    List<Candidate> c = getPredictedCandidates(alliance, ourPos, ourSpeedCap, cat, limit);
    ArrayList<RepulsorSetpoint> out = new ArrayList<>(c.size());
    for (Candidate k : c) out.add(k.setpoint);
    return out;
  }

  public void ingestTracked(String id, String type, Pose3d p, long nowNs) {
    dynamicTracker.ingestTracked(id, type, p, nowNs);
  }

  private void rebuildObjectiveCaches() {
    FieldTrackerObjectiveCache.rebuildObjectiveCacheForCategory(
        collectCache, CategorySpec.kCollect, field_map);
    collectPlanner.clearState();
  }

  private List<DynamicObject> snapshotDynamics() {
    return dynamicTracker.snapshotDynamics();
  }

  public Pose2d nextObjectiveGoalBlue(
      Pose2d robotPoseBlue, double ourSpeedCap, int goalUnits, CategorySpec cat) {
    return collectPlanner.nextObjectiveGoalBlue(robotPoseBlue, ourSpeedCap, goalUnits, cat);
  }

  public Pose2d nextCollectionGoalBlue(Pose2d robotPoseBlue, double ourSpeedCap, int goalUnits) {
    return nextObjectiveGoalBlue(robotPoseBlue, ourSpeedCap, goalUnits, CategorySpec.kCollect);
  }

  public FieldVision createFieldVision(String name) {
    return new FieldVision(this, name);
  }
}
