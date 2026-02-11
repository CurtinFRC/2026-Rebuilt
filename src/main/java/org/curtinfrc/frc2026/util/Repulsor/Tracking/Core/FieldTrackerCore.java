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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.concurrent.ConcurrentHashMap;
import java.util.function.Predicate;
import java.util.function.ToDoubleBiFunction;
import org.curtinfrc.frc2026.util.Repulsor.Constants;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldLayoutProvider;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldMapBuilder.CategorySpec;
import org.curtinfrc.frc2026.util.Repulsor.Fields.Rebuilt2026;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Core.Model.Candidate;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Core.Model.CollectProbe;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Core.Model.DynamicObject;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Core.Model.PointCandidate;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Core.Model.ResourceSpec;
import org.curtinfrc.frc2026.util.Repulsor.Predictive.Core.PredictiveFieldStateCore;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.RepulsorSetpoint;
import org.curtinfrc.frc2026.util.Repulsor.StickyTarget;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Core.Model.Alliance;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Core.Model.GameElement;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Core.Model.GameElementModel;
import org.curtinfrc.frc2026.util.Repulsor.Tracking.Core.Vision.FieldVision;

public class FieldTrackerCore {
  private static volatile FieldTrackerCore instance;
  private static volatile FieldLayoutProvider defaultProvider = new Rebuilt2026();
  private static final String DEFAULT_COLLECT_RESOURCE_TYPE = "fuel";

  private StickyTarget<Translation2d> collectStickySelector = new StickyTarget<>(0.22, 1.25, 1.80);

  public void resetAll() {
    collectStickySelector = new StickyTarget<>(0.22, 1.25, 1.80);

    clearCollectSticky();
  }

  private volatile long collectStickyReachedTsNs = 0L;

  private volatile Translation2d collectStickyApproachHat = null;
  private volatile double collectStickyPushM = 0.0;
  private volatile long collectStickyNoProgressSinceNs = 0L;
  private volatile double collectStickyLastDistM = Double.POSITIVE_INFINITY;

  private volatile Translation2d collectStickyDriveTarget = null;
  private volatile int collectStickySide = 0;
  private volatile long collectStickyLastSwitchNs = 0L;
  private Translation2d collectDriveLastTarget = null;
  private long collectDriveLastTargetNs = 0L;

  private static final double COLLECT_GROUP_CELL_M = 0.40;
  private static final double COLLECT_GROUP_R1_M = 0.95;
  private static final double COLLECT_GROUP_R2_M = 1.70;
  private static final double COLLECT_GROUP_MIN_COUNT = 2.0;
  private static final double COLLECT_RELOCK_ENABLE_DIST_M = 1.6;
  private static final double COLLECT_HALF_KEEP_MID_BAND_M = 1.2;
  private double collectStickyInvalidSec = 0.0;

  private volatile Translation2d collectForcedDriveCand = null;
  private volatile long collectForcedDriveSinceNs = 0L;

  private static final double COLLECT_STUCK_RADIUS_M = 0.10;
  private static final double COLLECT_STUCK_RESET_MOVE_M = 0.22;

  private Translation2d collectStuckAnchorPos = new Translation2d();
  private long collectStuckAnchorNs = 0L;

  private volatile int collectStickyHalfLock = 0;

  PointCandidate lastBest;

  private static final double COLLECT_GROUP_W_C1 = 1.00;
  private static final double COLLECT_GROUP_W_C2 = 0.35;
  private static final double COLLECT_GROUP_W_ETA = 1.25;
  private static final double COLLECT_GROUP_W_SPREAD = 0.60;
  private static final double COLLECT_GROUP_W_CENTER = 0.18;
  private static final double COLLECT_NEARBY_RADIUS_M = 2.2;
  private static final int COLLECT_NEARBY_MIN_COUNT = 1;

  private static final double COLLECT_LIVE_FUEL_NEAR_TARGET_R_M = 0.65;
  private static final double COLLECT_REACHED_EMPTY_FORCE_DROP_SEC = 0.18;
  private static final double COLLECT_REACHED_EMPTY_NEAR_TARGET_M = 0.95;
  private double collectReachedEmptySec = 0.0;

  private static final double COLLECT_DONE_NO_FUEL_SEC = 0.35;
  private static final double COLLECT_DONE_STUCK_SEC = 0.35;
  private static final double COLLECT_STUCK_SPEED_MPS = 0.15;

  private static final double COLLECT_EMPTY_DRIVE_DONE_SEC = 0.35;
  private static final double COLLECT_EMPTY_DRIVE_NEAR_ROBOT_M = 1.05;
  private static final double COLLECT_EMPTY_DRIVE_PROBE_R_M = 0.55;
  private static final double COLLECT_EMPTY_DRIVE_MIN_UNITS = 0.06;

  private double collectEmptyDriveSec = 0.0;

  private static final double COLLECT_CELL_M = 0.14;
  private static final int COLLECT_COARSE_TOPK = 4;
  private static final int COLLECT_REFINE_GRID = 3;

  private static final double COLLECT_DRIVE_PROBE_R_M = 0.55;
  private static final double COLLECT_DRIVE_MIN_UNITS = 0.045;
  private static final double COLLECT_DRIVE_SEARCH_STEP_M = 0.14;
  private static final int COLLECT_DRIVE_SEARCH_GRID = 3;

  private Translation2d collectStickyStillFiltPos = new Translation2d();
  private Translation2d collectStickyStillFiltLastPos = new Translation2d();
  private int collectStickyRobotHalfLast = 0;
  private double collectStickyRobotFlickerSec = 0.0;

  private static final double COLLECT_STICKY_NEAR_DIST_M = 1.40;
  private static final double COLLECT_STICKY_FAR_DIST_M = 4.80;

  private static final double COLLECT_POINT_CANONICAL_SNAP_M = 0.45;

  private static final double COLLECT_STICKY_SAME_M = 0.45;
  private static final double COLLECT_SWITCH_CLOSE_M = 0.85;

  private static final double COLLECT_STICKY_REACHED_M = 0.22;
  private static final double COLLECT_STICKY_TARGET_RECALC_EPS_M = 0.14;

  private static final double COLLECT_STICKY_NO_PROGRESS_S = 0.40;
  private static final double COLLECT_STICKY_NO_PROGRESS_DROP_M = 0.06;
  private static final double COLLECT_STICKY_NO_PROGRESS_MIN_DIST_M = 0.50;

  private static final double COLLECT_STICKY_FLAP_COOLDOWN_S = 0.95;

  private static final double COLLECT_SWITCH_MIN_MOVE_M = 0.18;
  private static final double COLLECT_SWITCH_APPROACH_COS_MIN = Math.cos(Math.toRadians(78.0));

  private static final double COLLECT_SWITCH_COOLDOWN_S = 0.70;

  private double collectNoFuelSec = 0.0;
  private double collectStuckSec = 0.0;

  private Translation2d lastRobotPosForStuck = new Translation2d();

  private volatile double collectStickyEtaS = 0.0;
  private volatile long lastObjectiveTickNs = 0L;

  private static final double COLLECT_RESOURCE_SNAP_MAX_DIST_M = 0.55;
  private static final double COLLECT_RESOURCE_SNAP_TINY_M = 0.14;
  private static final double COLLECT_VALID_NEAR_FUEL_M = 0.25;
  private static final double COLLECT_RESOURCE_SNAP_MIN_UNITS = 0.07;

  private static final double COLLECT_SNAP_TO_POINT_M = 0.14;
  private static final double COLLECT_SNAP_HYST_M = 0.06;
  private boolean collectSnapActive = false;
  private Translation2d collectStickyStillLastPos = new Translation2d();
  private double collectStickyStillSec = 0.0;

  private static boolean stickySame(Translation2d a, Translation2d b) {
    return samePoint(a, b, COLLECT_STICKY_SAME_M);
  }

  private static boolean stickySwitched(Translation2d prev, Translation2d next) {
    if (prev == null && next != null) return true;
    if (prev == null || next == null) return false;
    return !stickySame(prev, next);
  }

  private Translation2d collectStickyLastSwitchRobotPos = null;

  private static Translation2d canonicalizeCollectPoint(
      Translation2d p, Translation2d[] setpoints) {
    if (p == null || setpoints == null || setpoints.length == 0) return p;

    Translation2d best = null;
    double bestD2 = Double.POSITIVE_INFINITY;

    for (Translation2d s : setpoints) {
      if (s == null) continue;
      double dx = p.getX() - s.getX();
      double dy = p.getY() - s.getY();
      double d2 = dx * dx + dy * dy;
      if (d2 < bestD2) {
        bestD2 = d2;
        best = s;
      }
    }

    if (best == null) return p;
    double snap2 = COLLECT_POINT_CANONICAL_SNAP_M * COLLECT_POINT_CANONICAL_SNAP_M;
    return bestD2 <= snap2 ? best : p;
  }

  private void clearCollectSticky() {
    collectStickyPoint = null;
    collectStickyScore = -1e18;
    collectStickyTsNs = 0L;
    collectStickyReachedTsNs = 0L;
    collectStickyApproachHat = null;
    collectStickyPushM = 0.0;
    collectStickyDriveTarget = null;
    collectStickyNoProgressSinceNs = 0L;
    collectStickyLastDistM = Double.POSITIVE_INFINITY;
    collectStickyLastSwitchNs = 0L;
    collectStickyLastSwitchRobotPos = null;
    collectStickyEtaS = 0.0;
    collectStickySide = 0;
    collectSnapActive = false;
    collectStickyHalfLock = 0;
    collectNoFuelSec = 0.0;
    collectStuckSec = 0.0;
    lastRobotPosForStuck = new Translation2d();
    collectEmptyDriveSec = 0.0;
    collectStuckAnchorPos = new Translation2d();
    collectStuckAnchorNs = 0L;
    collectReachedEmptySec = 0.0;
    collectStickySelector.clear();
    collectStickyStillLastPos = new Translation2d();
    collectStickyStillSec = 0.0;
    collectStickyStillFiltPos = new Translation2d();
    collectStickyStillFiltLastPos = new Translation2d();
    collectStickyRobotHalfLast = 0;
    collectStickyRobotFlickerSec = 0.0;
    collectStickyInvalidSec = 0.0;
    collectDriveLastTarget = null;
    collectDriveLastTargetNs = 0L;
  }

  private int countLiveCollectResourcesWithin(
      java.util.List<DynamicObject> dyn, Translation2d center, double r) {
    if (dyn == null || dyn.isEmpty() || center == null) return 0;
    double r2 = r * r;
    int n = 0;
    for (int i = 0; i < dyn.size(); i++) {
      DynamicObject o = dyn.get(i);
      if (o == null || o.pos == null || o.type == null) continue;
      if (!isCollectResourceType(o.type)) continue;
      double dx = o.pos.getX() - center.getX();
      double dy = o.pos.getY() - center.getY();
      if ((dx * dx + dy * dy) <= r2) n++;
    }
    return n;
  }

  private static double dot(Translation2d a, Translation2d b) {
    return a.getX() * b.getX() + a.getY() * b.getY();
  }

  private static Translation2d unit(Translation2d v) {
    double n = v.getNorm();
    if (n <= 1e-9) return new Translation2d(1.0, 0.0);
    return v.div(n);
  }

  private static boolean movedEnough(Translation2d a, Translation2d b, double minMove) {
    if (a == null || b == null) return true;
    return a.getDistance(b) >= minMove;
  }

  private static boolean violatesApproach(Translation2d approachHat, Translation2d toCandHat) {
    if (approachHat == null || toCandHat == null) return false;
    return dot(approachHat, toCandHat) < COLLECT_SWITCH_APPROACH_COS_MIN;
  }

  private Translation2d relockCollectPointToLiveFuel(
      Translation2d desiredCollectPoint,
      java.util.function.Function<Translation2d, Translation2d> clampToFieldRobotSafe,
      java.util.function.Predicate<Translation2d> inForbidden,
      java.util.function.Predicate<Translation2d> violatesWall,
      java.util.function.Function<Translation2d, Translation2d> nudgeOutOfForbidden) {

    if (desiredCollectPoint == null) return null;

    Translation2d p = desiredCollectPoint;

    Translation2d nearTiny = predictor.nearestCollectResource(p, COLLECT_RESOURCE_SNAP_TINY_M);
    if (nearTiny != null) {
      p = nearTiny;
    } else {
      Translation2d near = predictor.nearestCollectResource(p, COLLECT_RESOURCE_SNAP_MAX_DIST_M);
      if (near != null) {
        p = near;
      } else {
        Translation2d centroid = predictor.snapToCollectCentroid(p, 0.75, 0.15);
        if (centroid != null) p = centroid;
      }
    }

    p = clampToFieldRobotSafe.apply(p);
    if (inForbidden.test(p)) p = nudgeOutOfForbidden.apply(p);
    p = clampToFieldRobotSafe.apply(p);

    if (inForbidden.test(p) || violatesWall.test(p)) return desiredCollectPoint;
    return p;
  }

  private static Translation2d unitOrDefault(Translation2d v, Translation2d def) {
    double n = v.getNorm();
    if (n <= 1e-9) return def;
    return v.div(n);
  }

  private Translation2d computeFrozenDriveTarget(
      Translation2d resource,
      Translation2d approachHat,
      double pushM,
      java.util.function.Function<Translation2d, Translation2d> clampToFieldRobotSafe,
      java.util.function.Predicate<Translation2d> inForbidden,
      java.util.function.Predicate<Translation2d> violatesWall,
      java.util.function.Function<Translation2d, Translation2d> nudgeOutOfForbidden) {

    Translation2d t = clampToFieldRobotSafe.apply(resource.plus(approachHat.times(pushM)));
    if (inForbidden.test(t)) t = nudgeOutOfForbidden.apply(t);
    t = clampToFieldRobotSafe.apply(t);

    if (inForbidden.test(t) || violatesWall.test(t)) {
      Translation2d base = clampToFieldRobotSafe.apply(resource);
      if (inForbidden.test(base)) base = nudgeOutOfForbidden.apply(base);
      base = clampToFieldRobotSafe.apply(base);
      if (!inForbidden.test(base) && !violatesWall.test(base)) return base;
    }
    return t;
  }

  private static int sideSignX(double x) {
    return sideSignXBand(x, 0.10);
  }

  private static int sideSignXBand(double x, double band) {
    double mid = Constants.FIELD_LENGTH * 0.5;
    double b = Math.max(0.0, band);
    if (x < mid - b) return -1;
    if (x > mid + b) return 1;
    return 0;
  }

  private static int sideSign(Translation2d p) {
    if (p == null) return 0;
    return sideSignX(p.getX());
  }

  private static double lerp(double a, double b, double t) {
    return a + (b - a) * t;
  }

  private static double clamp01(double x) {
    if (x < 0.0) return 0.0;
    if (x > 1.0) return 1.0;
    return x;
  }

  private Pose2d fallbackCollectPose(Pose2d robotPoseBlue) {
    Translation2d p =
        collectStickyDriveTarget != null ? collectStickyDriveTarget : collectStickyPoint;
    if (p == null && lastBest != null) p = lastBest.point;
    if (p == null && robotPoseBlue != null) p = robotPoseBlue.getTranslation();
    if (p == null) p = new Translation2d();
    Rotation2d rot = robotPoseBlue != null ? robotPoseBlue.getRotation() : new Rotation2d();
    return new Pose2d(p, rot);
  }

  private volatile Translation2d collectStickyPoint = null;
  private volatile double collectStickyScore = -1e18;
  private volatile long collectStickyTsNs = 0L;

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
    rebuildObjectiveCaches();
    configureCollectResourceProfile(
        DEFAULT_COLLECT_RESOURCE_TYPE, new ResourceSpec(0.075, 1.0, 0.95));
  }

  private final ConcurrentHashMap<String, String> typeAliases = new ConcurrentHashMap<>();

  public void registerTypeAlias(String from, String to) {
    if (from == null || from.isEmpty())
      throw new IllegalArgumentException("from cannot be null/empty");
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

  private PredictiveFieldStateCore predictor;

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

  public GameElement[] field_map;

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

  private static final class TrackedObj {
    final String id;
    volatile String type;
    volatile Pose3d pos;
    volatile Pose3d prev;
    volatile long tNs;
    volatile double vx;
    volatile double vy;
    volatile double vz;

    TrackedObj(String id) {
      this.id = id;
      this.type = "unknown";
      this.pos = null;
      this.prev = null;
      this.tNs = 0L;
      this.vx = 0.0;
      this.vy = 0.0;
      this.vz = 0.0;
    }
  }

  private final ConcurrentHashMap<String, TrackedObj> tracked = new ConcurrentHashMap<>();

  public void ingestTracked(String id, String type, Pose3d p, long nowNs) {
    if (id == null || id.isEmpty() || p == null) return;
    TrackedObj st = tracked.computeIfAbsent(id, TrackedObj::new);
    Pose3d prev = st.pos;
    long t0 = st.tNs;
    st.prev = prev;
    st.pos = p;
    st.type = type != null ? type : "unknown";
    st.tNs = nowNs;

    if (prev != null && t0 != 0L && nowNs > t0) {
      double dt = (nowNs - t0) / 1e9;
      if (dt > 1e-6) {
        st.vx = (p.getX() - prev.getX()) / dt;
        st.vy = (p.getY() - prev.getY()) / dt;
        st.vz = (p.getZ() - prev.getZ()) / dt;
      }
    }
  }

  private static final class ObjectiveCache {
    volatile Translation2d[] points = new Translation2d[0];
    volatile int lastHash = 0;
  }

  private final ObjectiveCache collectCache;

  private static int mixHash(int h, double x, double y) {
    long a = Double.doubleToLongBits(x);
    long b = Double.doubleToLongBits(y);
    h ^= (int) (a ^ (a >>> 32));
    h = (h * 16777619) ^ (int) (b ^ (b >>> 32));
    return h;
  }

  private void rebuildObjectiveCaches() {
    rebuildObjectiveCacheForCategory(collectCache, CategorySpec.kCollect);
    clearCollectSticky();
  }

  private void rebuildObjectiveCacheForCategory(ObjectiveCache cache, CategorySpec cat) {
    GameElement[] fm = field_map;
    if (fm == null || fm.length == 0) {
      cache.points = new Translation2d[0];
      cache.lastHash = 0;
      return;
    }
    ArrayList<Translation2d> pts = new ArrayList<>(256);
    int h = 146959810;
    for (GameElement e : fm) {
      if (e == null) continue;
      if (e.getCategory() != cat) continue;
      GameElementModel m = e.getModel();
      if (m == null) continue;
      Pose3d p = m.getPosition();
      if (p == null) continue;
      double x = p.getX();
      double y = p.getY();
      pts.add(new Translation2d(x, y));
      h = mixHash(h, x, y);
    }
    Translation2d[] arr = pts.toArray(new Translation2d[0]);
    cache.points = arr;
    cache.lastHash = h;
  }

  private List<DynamicObject> snapshotDynamics() {
    long nowNs = System.nanoTime();
    boolean isSim = RobotBase.isSimulation();

    ArrayList<DynamicObject> out = new ArrayList<>(tracked.size());
    for (TrackedObj o : tracked.values()) {
      if (o == null) continue;
      Pose3d p = o.pos;
      if (p == null) continue;
      long t = o.tNs;
      if (t == 0L) continue;

      double ageS = (nowNs - t) / 1e9;
      if (ageS < 0.0) continue;

      String ty = o.type != null ? o.type : "unknown";

      if (isSim) ageS = 0.0;

      out.add(
          new DynamicObject(
              o.id,
              ty,
              new Translation2d(p.getX(), p.getY()),
              new Translation2d(o.vx, o.vy),
              ageS));
    }
    return out;
  }

  private static boolean samePoint(Translation2d a, Translation2d b, double tol) {
    if (a == null || b == null) return false;
    return a.getDistance(b) <= tol;
  }

  private static final double COLLECT_EMPTY_SPACE_MAX_DIST_TO_FUEL_M = 0.18;
  private static final double COLLECT_HOTSPOT_SNAP_RADIUS_M = 0.85;
  private static final double COLLECT_MAX_DRIVE_OFFSET_FROM_FUEL_M = 0.12;
  private static final double COLLECT_SNAP_TO_NEAREST_FUEL_M = 0.22;

  private static final class NearestPoint {
    final Translation2d p;
    final double d;

    NearestPoint(Translation2d p, double d) {
      this.p = p;
      this.d = d;
    }
  }

  private static NearestPoint nearestPointTo(Translation2d q, Translation2d[] pts) {
    if (q == null || pts == null || pts.length == 0)
      return new NearestPoint(null, Double.POSITIVE_INFINITY);
    Translation2d best = null;
    double bestD = Double.POSITIVE_INFINITY;
    for (Translation2d p : pts) {
      if (p == null) continue;
      double d = q.getDistance(p);
      if (d < bestD) {
        bestD = d;
        best = p;
      }
    }
    return new NearestPoint(best, bestD);
  }

  private static final class _CellKey {
    final int ix;
    final int iy;

    _CellKey(int ix, int iy) {
      this.ix = ix;
      this.iy = iy;
    }

    @Override
    public int hashCode() {
      return (ix * 73856093) ^ (iy * 19349663);
    }

    @Override
    public boolean equals(Object o) {
      if (this == o) return true;
      if (!(o instanceof _CellKey)) return false;
      _CellKey k = (_CellKey) o;
      return ix == k.ix && iy == k.iy;
    }
  }

  private static final class _SpatialIndex {
    final double cell;
    final java.util.HashMap<_CellKey, java.util.ArrayList<Translation2d>> map =
        new java.util.HashMap<>();

    _SpatialIndex(double cell) {
      this.cell = Math.max(1e-6, cell);
    }

    void add(Translation2d p) {
      int ix = (int) Math.floor(p.getX() / cell);
      int iy = (int) Math.floor(p.getY() / cell);
      _CellKey k = new _CellKey(ix, iy);
      map.computeIfAbsent(k, kk -> new java.util.ArrayList<>()).add(p);
    }

    java.util.ArrayList<Translation2d> getCell(int ix, int iy) {
      return map.get(new _CellKey(ix, iy));
    }

    int ix(double x) {
      return (int) Math.floor(x / cell);
    }

    int iy(double y) {
      return (int) Math.floor(y / cell);
    }
  }

  private static final class _GroupCandidate {
    final Translation2d point;
    final double groupScore;

    _GroupCandidate(Translation2d point, double groupScore) {
      this.point = point;
      this.groupScore = groupScore;
    }
  }

  private static double _dist2(Translation2d a, Translation2d b) {
    double dx = a.getX() - b.getX();
    double dy = a.getY() - b.getY();
    return dx * dx + dy * dy;
  }

  private static double _centerPenalty(Translation2d p) {
    double dx = p.getX() - (Constants.FIELD_LENGTH * 0.5);
    double dy = p.getY() - (Constants.FIELD_WIDTH * 0.5);
    return Math.sqrt(dx * dx + dy * dy);
  }

  private _SpatialIndex buildIndex(Translation2d[] pts, double cellM) {
    _SpatialIndex idx = new _SpatialIndex(cellM);
    for (Translation2d p : pts) {
      if (p != null) idx.add(p);
    }
    return idx;
  }

  private static Translation2d perp(Translation2d v) {
    return new Translation2d(-v.getY(), v.getX());
  }

  private static double nowSFromNs(long ns) {
    return ns / 1e9;
  }

  private static double holdSForDist(double d) {
    double t =
        clamp01(
            (d - COLLECT_STICKY_NEAR_DIST_M)
                / (COLLECT_STICKY_FAR_DIST_M - COLLECT_STICKY_NEAR_DIST_M));
    double min = lerp(0.70, 0.25, t);
    double max = lerp(3.80, 1.80, t);
    return Math.max(min, Math.min(max, max));
  }

  private static double switchMarginForDist(double d) {
    double t =
        clamp01(
            (d - COLLECT_STICKY_NEAR_DIST_M)
                / (COLLECT_STICKY_FAR_DIST_M - COLLECT_STICKY_NEAR_DIST_M));
    return lerp(1.45, 0.50, t);
  }

  public Pose2d nextObjectiveGoalBlue(
      Pose2d robotPoseBlue, double ourSpeedCap, int goalUnits, CategorySpec cat) {
    if (robotPoseBlue == null) return Pose2d.kZero;
    double cap = Math.max(0.2, ourSpeedCap);

    for (int pass = 0; pass < 2; pass++) {

      ObjectiveCache cache = cat == CategorySpec.kCollect ? collectCache : collectCache;
      Translation2d[] pts = cache.points;

      if (pts == null || pts.length == 0) {
        clearCollectSticky();
        return fallbackCollectPose(robotPoseBlue);
      }

      final double FORBID_MARGIN_M = 0.6;

      final double robotHalf =
          0.5
              * Math.max(
                  org.curtinfrc.frc2026.Constants.ROBOT_X, org.curtinfrc.frc2026.Constants.ROBOT_Y);
      final double robotWallMargin = robotHalf + 0.03;

      double L = Constants.FIELD_LENGTH;
      double halfW = 1.1938 * 0.5 + FORBID_MARGIN_M;

      double leftSqCx = 4.625594;
      double leftRectCx = (L * 0.5) - 3.63982;

      double rightSqCx = L - 4.625594;
      double rightRectCx = (L * 0.5) + 3.63982;

      double leftBandX0 = Math.min(leftSqCx, leftRectCx) - halfW;
      double leftBandX1 = Math.max(leftSqCx, leftRectCx) + halfW;

      double rightBandX0 = Math.min(rightSqCx, rightRectCx) - halfW;
      double rightBandX1 = Math.max(rightSqCx, rightRectCx) + halfW;

      java.util.function.Function<Translation2d, Translation2d> clampToFieldRobotSafe =
          p -> {
            double x = p.getX();
            double y = p.getY();
            double minX = robotWallMargin;
            double maxX = Constants.FIELD_LENGTH - robotWallMargin;
            double minY = robotWallMargin;
            double maxY = Constants.FIELD_WIDTH - robotWallMargin;
            if (x < minX) x = minX;
            if (x > maxX) x = maxX;
            if (y < minY) y = minY;
            if (y > maxY) y = maxY;
            return new Translation2d(x, y);
          };

      java.util.function.Function<Translation2d, Double> wallClearance =
          p -> {
            double x = p.getX();
            double y = p.getY();
            double dL = x;
            double dR = Constants.FIELD_LENGTH - x;
            double dB = y;
            double dT = Constants.FIELD_WIDTH - y;
            return Math.min(Math.min(dL, dR), Math.min(dB, dT));
          };

      java.util.function.Function<Translation2d, Pose2d> holdPose =
          p -> {
            Translation2d h = p;
            if (h == null) h = collectStickyDriveTarget;
            if (h == null) h = robotPoseBlue.getTranslation();
            return new Pose2d(h, robotPoseBlue.getRotation());
          };

      java.util.function.Predicate<Translation2d> violatesWall =
          p -> wallClearance.apply(p) < robotWallMargin;

      java.util.function.Predicate<Translation2d> inForbidden =
          p -> {
            double x = p.getX();
            return (x >= leftBandX0 && x <= leftBandX1) || (x >= rightBandX0 && x <= rightBandX1);
          };

      java.util.function.Function<Translation2d, Translation2d> nudgeOutOfForbidden =
          p -> {
            if (!inForbidden.test(p)) return p;
            double x = p.getX();
            double y = p.getY();
            double pad = 0.06;
            if (x >= leftBandX0 && x <= leftBandX1) {
              double dl = Math.abs(x - leftBandX0);
              double dr = Math.abs(leftBandX1 - x);
              x = (dl < dr) ? (leftBandX0 - pad) : (leftBandX1 + pad);
            } else if (x >= rightBandX0 && x <= rightBandX1) {
              double dl = Math.abs(x - rightBandX0);
              double dr = Math.abs(rightBandX1 - x);
              x = (dl < dr) ? (rightBandX0 - pad) : (rightBandX1 + pad);
            }
            return clampToFieldRobotSafe.apply(new Translation2d(x, y));
          };

      java.util.function.Function<Translation2d, Translation2d> safePushedFromRobot =
          resource -> {
            Translation2d rp = robotPoseBlue.getTranslation();
            Translation2d dir = resource.minus(rp);
            double n = dir.getNorm();
            if (n <= 1e-6) return clampToFieldRobotSafe.apply(resource);

            double dist = rp.getDistance(resource);
            double pushT = clamp01((dist - 0.35) / (2.50 - 0.35));
            double pushM = Math.min(lerp(0.42, 0.18, pushT), COLLECT_MAX_DRIVE_OFFSET_FROM_FUEL_M);

            Translation2d dirHat = dir.div(n);

            Translation2d targetFull =
                clampToFieldRobotSafe.apply(resource.plus(dirHat.times(pushM)));
            if (!inForbidden.test(targetFull) && !violatesWall.test(targetFull)) return targetFull;

            double lo = 0.0;
            double hi = 1.0;
            Translation2d best = clampToFieldRobotSafe.apply(resource);
            if (!inForbidden.test(best) && !violatesWall.test(best)) return best;

            for (int i = 0; i < 16; i++) {
              double mid = 0.5 * (lo + hi);
              Translation2d cand =
                  clampToFieldRobotSafe.apply(resource.plus(dirHat.times(pushM * mid)));
              boolean ok2 = !inForbidden.test(cand) && !violatesWall.test(cand);
              if (ok2) {
                best = cand;
                lo = mid;
              } else {
                hi = mid;
              }
            }

            best = nudgeOutOfForbidden.apply(best);
            best = clampToFieldRobotSafe.apply(best);
            return best;
          };

      ArrayList<Translation2d> ok = new ArrayList<>(pts.length);
      for (Translation2d p : pts) {
        if (p == null) continue;
        Translation2d drive = safePushedFromRobot.apply(p);
        drive = clampToFieldRobotSafe.apply(drive);
        if (inForbidden.test(drive)) drive = nudgeOutOfForbidden.apply(drive);
        drive = clampToFieldRobotSafe.apply(drive);
        if (violatesWall.test(drive)) continue;
        if (!inForbidden.test(drive)) ok.add(p);
      }

      Translation2d[] usePts = ok.toArray(new Translation2d[0]);
      if (usePts.length == 0) {
        clearCollectSticky();
        return fallbackCollectPose(robotPoseBlue);
      }

      Translation2d robotPos = robotPoseBlue.getTranslation();
      double midX = Constants.FIELD_LENGTH * 0.5;
      boolean robotInCenterBand = Math.abs(robotPos.getX() - midX) <= COLLECT_HALF_KEEP_MID_BAND_M;

      java.util.List<DynamicObject> dynAll = snapshotDynamics();

      int lockHalf = collectStickyHalfLock;
      int sensedHalf = sideSignXBand(robotPos.getX(), COLLECT_HALF_KEEP_MID_BAND_M);

      if (sensedHalf != 0) {
        if (lockHalf != sensedHalf) {
          lockHalf = sensedHalf;
          collectStickyHalfLock = sensedHalf;
        }
      } else {
        if (lockHalf == 0 && collectStickyPoint != null) {
          int s = sideSign(collectStickyPoint);
          if (s != 0) {
            lockHalf = s;
            collectStickyHalfLock = s;
          }
        }
      }

      java.util.List<DynamicObject> dynUse = dynAll;

      if (lockHalf != 0 && dynAll != null && !dynAll.isEmpty()) {
        double mid = Constants.FIELD_LENGTH * 0.5;
        java.util.ArrayList<DynamicObject> filtered = new java.util.ArrayList<>(dynAll.size());

        for (int i = 0; i < dynAll.size(); i++) {
          DynamicObject o = dynAll.get(i);
          if (o == null || o.pos == null) continue;

          double x = o.pos.getX();

          if (Math.abs(x - mid) <= COLLECT_HALF_KEEP_MID_BAND_M) {
            filtered.add(o);
            continue;
          }

          if (sideSignX(x) == lockHalf) filtered.add(o);
        }

        if (!filtered.isEmpty()) dynUse = filtered;
      }

      predictor.setDynamicObjects(dynAll);

      long nowNs = System.nanoTime();
      long prevNs = lastObjectiveTickNs;
      lastObjectiveTickNs = nowNs;
      double dt = prevNs != 0L ? (nowNs - prevNs) / 1e9 : 0.02;
      if (dt < 1e-3) dt = 1e-3;
      if (dt > 0.08) dt = 0.08;

      double rawMoveM = robotPos.getDistance(collectStickyStillLastPos);
      collectStickyStillLastPos = robotPos;

      double a = 1.0 - Math.exp(-dt / 0.12);
      collectStickyStillFiltPos = collectStickyStillFiltPos.interpolate(robotPos, a);

      double filtMoveM = collectStickyStillFiltPos.getDistance(collectStickyStillFiltLastPos);
      if (filtMoveM <= 0.02) collectStickyStillSec += dt;
      else collectStickyStillSec = 0.0;
      collectStickyStillFiltLastPos = collectStickyStillFiltPos;

      int halfNow = sideSignXBand(robotPos.getX(), 0.08);
      if (halfNow != 0
          && collectStickyRobotHalfLast != 0
          && halfNow != collectStickyRobotHalfLast
          && rawMoveM <= 0.06) {
        collectStickyRobotFlickerSec += dt;
      } else {
        collectStickyRobotFlickerSec = 0.0;
      }
      if (halfNow != 0) collectStickyRobotHalfLast = halfNow;

      int nearbyFuelCount = 0;
      double nearbySX = 0.0;
      double nearbySY = 0.0;

      if (dynUse != null && !dynUse.isEmpty()) {
        for (int i = 0; i < dynUse.size(); i++) {
          DynamicObject o = dynUse.get(i);
          if (o == null || o.pos == null || o.type == null) continue;
          if (!isCollectResourceType(o.type)) continue;
          double d = o.pos.getDistance(robotPos);
          if (d <= COLLECT_NEARBY_RADIUS_M) {
            nearbyFuelCount++;
            nearbySX += o.pos.getX();
            nearbySY += o.pos.getY();
          }
        }
      }

      Translation2d nearbyCentroid =
          nearbyFuelCount > 0
              ? new Translation2d(nearbySX / nearbyFuelCount, nearbySY / nearbyFuelCount)
              : null;

      java.util.HashMap<Long, CollectProbe> probeCache = new java.util.HashMap<>(512);
      java.util.HashMap<Long, Boolean> footprintCache = new java.util.HashMap<>(512);
      java.util.HashMap<Long, Boolean> nearFuelCache = new java.util.HashMap<>(512);
      java.util.HashMap<Long, Boolean> validCache = new java.util.HashMap<>(512);
      java.util.HashMap<Long, Boolean> validRelaxedCache = new java.util.HashMap<>(512);
      java.util.HashMap<Long, Double> scoreCache = new java.util.HashMap<>(512);

      java.util.function.ToLongFunction<Translation2d> pointKey =
          p -> {
            long kx = (long) Math.round(p.getX() * 1000.0);
            long ky = (long) Math.round(p.getY() * 1000.0);
            return (kx << 32) ^ (ky & 0xffffffffL);
          };

      java.util.function.Function<Translation2d, CollectProbe> probe =
          p -> {
            if (p == null) return null;
            long key = pointKey.applyAsLong(p);
            if (probeCache.containsKey(key)) return probeCache.get(key);
            CollectProbe pr = predictor.probeCollect(p, 0.75);
            probeCache.put(key, pr);
            return pr;
          };

      java.util.function.Predicate<Translation2d> footprintHasFuel =
          p -> {
            if (p == null) return false;
            long key = pointKey.applyAsLong(p);
            Boolean cached = footprintCache.get(key);
            if (cached != null) return cached;
            boolean ok1 = predictor.footprintHasCollectResource(p, COLLECT_CELL_M);
            if (footprintCache.size() > 2048) footprintCache.clear();
            footprintCache.put(key, ok1);
            return ok1;
          };

      java.util.function.Predicate<Translation2d> hasNearFuel =
          p -> {
            if (p == null) return false;
            long key = pointKey.applyAsLong(p);
            Boolean cached = nearFuelCache.get(key);
            if (cached != null) return cached;
            boolean ok1 = predictor.nearestCollectResource(p, COLLECT_VALID_NEAR_FUEL_M) != null;
            if (nearFuelCache.size() > 2048) nearFuelCache.clear();
            nearFuelCache.put(key, ok1);
            return ok1;
          };

      java.util.function.Predicate<Translation2d> collectValid =
          p -> {
            if (p == null) return false;
            long key = pointKey.applyAsLong(p);
            Boolean cached = validCache.get(key);
            if (cached != null) return cached;
            CollectProbe pr = probe.apply(p);
            if (pr == null) {
              validCache.put(key, false);
              return false;
            }
            if (pr.count < 1 || pr.units < Math.max(0.02, COLLECT_RESOURCE_SNAP_MIN_UNITS * 0.75)) {
              validCache.put(key, false);
              return false;
            }
            if (!hasNearFuel.test(p)) {
              validCache.put(key, false);
              return false;
            }
            boolean ok1 = footprintHasFuel.test(p);
            validCache.put(key, ok1);
            return ok1;
          };

      java.util.function.Predicate<Translation2d> collectValidRelaxed =
          p -> {
            if (p == null) return false;
            long key = pointKey.applyAsLong(p);
            Boolean cached = validRelaxedCache.get(key);
            if (cached != null) return cached;
            CollectProbe pr = probe.apply(p);
            if (pr == null) {
              validRelaxedCache.put(key, false);
              return false;
            }
            if (pr.count < 1 || pr.units < Math.max(0.02, COLLECT_RESOURCE_SNAP_MIN_UNITS * 0.75)) {
              validRelaxedCache.put(key, false);
              return false;
            }
            boolean ok1 = footprintHasFuel.test(p);
            validRelaxedCache.put(key, ok1);
            return ok1;
          };

      java.util.function.Function<Translation2d, Double> collectUnits =
          p -> {
            CollectProbe pr = probe.apply(p);
            if (pr == null) return 0.0;
            return pr.units;
          };

      PointCandidate best = null;

      for (int attempt = 0; attempt < 5; attempt++) {
        best =
            predictor.rankCollectNearest(
                robotPos,
                cap,
                usePts,
                COLLECT_CELL_M,
                goalUnits,
                Math.min(160, Math.max(32, usePts.length)));

        if (best != null) { // && collectValid.test(best.point)
          lastBest = best;
          // Logger.recordOutput("method", "collect_nearest_attempt");
          break;
        }

        if (best == null && lastBest != null) {
          best = lastBest;
          break;
        }

        best =
            predictor.rankCollectHierarchical(
                robotPos,
                cap,
                usePts,
                COLLECT_CELL_M,
                goalUnits,
                COLLECT_COARSE_TOPK,
                COLLECT_REFINE_GRID);

        if (best != null && collectValid.test(best.point)) {
          // Logger.recordOutput("method", "collect_hierarchical_attempt");
          break;
        }

        best =
            predictor.rankCollectPoints(
                robotPos, cap, usePts, goalUnits, Math.max(24, usePts.length));

        if (best != null && collectValid.test(best.point)) {
          // Logger.recordOutput("method", "collect_point_rank_attempt");
          break;
        }

        // Logger.recordOutput("method", "hotspot_snap_attempt");
        Translation2d hot = predictor.bestCollectHotspot(usePts, COLLECT_CELL_M);
        if (hot != null) {
          NearestPoint snap = nearestPointTo(hot, usePts);
          if (snap.p != null
              && snap.d <= COLLECT_HOTSPOT_SNAP_RADIUS_M
              && collectValid.test(snap.p)) {
            best =
                new PointCandidate(
                    snap.p,
                    new Rotation2d(),
                    robotPos.getDistance(snap.p) / Math.max(0.1, cap),
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    -1e9);
            break;
          }
        }
      }

      // // Logger.recordOutput("best", new Pose2d(best.point, Rotation2d.kZero));
      Translation2d rawCandidate = (best != null ? best.point : null);
      // Logger.recordOutput("best1", rawCandidate);
      // Logger.recordOutput("collectValid", collectValid.test(rawCandidate));
      // Logger.recordOutput("collectValidRelaxed", collectValidRelaxed.test(rawCandidate));

      if (rawCandidate == null || !collectValid.test(rawCandidate)) {
        Translation2d fallback = null;
        double bestU = 0.0;
        boolean anyStrict = false;
        for (Translation2d p : usePts) {
          if (p == null) continue;
          CollectProbe pr = probe.apply(p);
          if (pr == null
              || pr.count < 1
              || pr.units < Math.max(0.02, COLLECT_RESOURCE_SNAP_MIN_UNITS * 0.75)) continue;
          if (!hasNearFuel.test(p)) continue;
          if (!footprintHasFuel.test(p)) continue;
          anyStrict = true;
          double u = pr.units;
          if (u > bestU) {
            bestU = u;
            fallback = p;
          }
        }
        if (fallback == null && !anyStrict) {
          for (Translation2d p : usePts) {
            if (p == null) continue;
            CollectProbe pr = probe.apply(p);
            if (pr == null
                || pr.count < 1
                || pr.units < Math.max(0.02, COLLECT_RESOURCE_SNAP_MIN_UNITS * 0.75)) continue;
            if (!footprintHasFuel.test(p)) continue;
            double u = pr.units;
            if (u > bestU) {
              bestU = u;
              fallback = p;
            }
          }
        }
        if (fallback == null || !collectValid.test(fallback)) {
          Translation2d near =
              predictor.nearestCollectResource(robotPos, COLLECT_SNAP_TO_NEAREST_FUEL_M);
          if (near != null && collectValidRelaxed.test(near)) {
            fallback = near;
          }
        }
        if (fallback == null || !collectValidRelaxed.test(fallback)) {
          Translation2d lastBestPoint = (lastBest != null) ? lastBest.point : null;
          if (lastBestPoint != null && collectValidRelaxed.test(lastBestPoint)) {
            fallback = lastBestPoint;
          }
        }
        if (fallback == null || !collectValidRelaxed.test(fallback)) {
          if (rawCandidate != null && collectValidRelaxed.test(rawCandidate)) {
            fallback = rawCandidate;
          }
        }
        if (fallback == null || !collectValidRelaxed.test(fallback)) {
          clearCollectSticky();
          return new Pose2d(robotPos, robotPoseBlue.getRotation());
        }
        rawCandidate = fallback;
      }

      // rawCandidate = canonicalizeCollectPoint(rawCandidate, usePts);

      // // Logger.recordOutput("sticky_dbg_rawCandU", new Pose2d(rawCandidate, Rotation2d.kZero));

      java.util.function.Function<Translation2d, Double> scoreResource =
          p -> {
            if (p == null) return -1e18;
            long key = pointKey.applyAsLong(p);
            Double cached = scoreCache.get(key);
            if (cached != null) return cached;

            if (!collectValid.test(p)) {
              scoreCache.put(key, -1e18);
              return -1e18;
            }

            double u = collectUnits.apply(p);

            Translation2d d = safePushedFromRobot.apply(p);
            if (d == null) d = p;

            d = clampToFieldRobotSafe.apply(d);
            if (inForbidden.test(d)) d = nudgeOutOfForbidden.apply(d);
            d = clampToFieldRobotSafe.apply(d);
            if (inForbidden.test(d) || violatesWall.test(d)) return -1e18;

            double eta = robotPos.getDistance(d) / Math.max(0.2, cap);

            double scoreV = (u * 1.0) - (0.55 * eta);
            scoreCache.put(key, scoreV);
            return scoreV;
          };

      Translation2d bestCandidate = rawCandidate;

      boolean allowCentroidCandidate =
          (robotPos.getDistance(rawCandidate) <= COLLECT_NEARBY_RADIUS_M)
              && (collectStickyStillSec < 0.20);

      if (allowCentroidCandidate && nearbyCentroid != null) {
        Translation2d c = clampToFieldRobotSafe.apply(nearbyCentroid);
        if (inForbidden.test(c)) c = nudgeOutOfForbidden.apply(c);
        c = clampToFieldRobotSafe.apply(c);

        if (!inForbidden.test(c) && !violatesWall.test(c) && collectValid.test(c)) {
          double sc = scoreResource.apply(c);
          double sb = scoreResource.apply(bestCandidate);
          if (sc >= sb - 0.08) bestCandidate = c;
        }
      }

      Translation2d relockCand =
          relockCollectPointToLiveFuel(
              bestCandidate, clampToFieldRobotSafe, inForbidden, violatesWall, nudgeOutOfForbidden);

      if (relockCand != null && collectValid.test(relockCand)) {
        double sr = scoreResource.apply(relockCand);
        double sb = scoreResource.apply(bestCandidate);
        if (sr >= sb - 0.04) bestCandidate = relockCand;
      }

      bestCandidate = canonicalizeCollectPoint(bestCandidate, usePts);

      double distToCand = robotPos.getDistance(bestCandidate);
      double distToCur =
          collectStickyPoint != null ? robotPos.getDistance(collectStickyPoint) : Double.NaN;
      double holdS = holdSForDist(distToCand);
      double keepMargin = switchMarginForDist(distToCand);
      double immediateDelta = Math.max(keepMargin * 1.85, keepMargin + 0.28);

      if (Double.isFinite(distToCur) && distToCur - distToCand > 0.35) {
        holdS = Math.min(holdS, 0.35);
        keepMargin *= 0.70;
        immediateDelta *= 0.70;
      }

      if (robotInCenterBand) {
        holdS = Math.max(holdS, 0.85);
        keepMargin *= 1.12;
        immediateDelta *= 1.08;
      }

      final double keepMarginF = keepMargin;
      final boolean centerF = robotInCenterBand;
      final int lockHalfF = lockHalf;

      ToDoubleBiFunction<Translation2d, Translation2d> transitionExtra =
          (from, to) -> {
            if (from == null || to == null) return 0.0;
            double extra = 0.0;

            int p = sideSignXBand(from.getX(), 0.10);
            int q = sideSignXBand(to.getX(), 0.10);

            if (centerF && p != 0 && q != 0 && p != q) extra += keepMarginF * 0.90 + 0.18;
            if (lockHalfF != 0 && q != 0 && q != lockHalfF) extra += keepMarginF * 0.55 + 0.10;
            if (from.getDistance(to) <= COLLECT_SWITCH_CLOSE_M) extra += keepMarginF * 1.55 + 0.18;

            return extra;
          };

      // Logger.recordOutput(
      // "sticky_dbg_valid_sticky",
      // collectStickyPoint != null && collectValid.test(collectStickyPoint));
      // Logger.recordOutput(
      // "sticky_dbg_valid_bestCand", bestCandidate != null && collectValid.test(bestCandidate));
      // Logger.recordOutput("sticky_dbg_dynUse_n", dynUse != null ? dynUse.size() : -1);
      // Logger.recordOutput("sticky_dbg_lockHalf", collectStickyHalfLock);
      // Logger.recordOutput("sticky_dbg_sensedHalf", sensedHalf);
      Translation2d selectedResource;
      // Logger.recordOutput("pass", pass);

      if (pass == 0) {
        // Logger.recordOutput("tracker_dbg_bestCandidate", bestCandidate);
        selectedResource =
            collectStickySelector.update(
                bestCandidate,
                scoreResource.apply(bestCandidate),
                scoreResource::apply,
                collectValid,
                holdS,
                keepMargin,
                immediateDelta,
                transitionExtra,
                (p, q) -> p.getDistance(q),
                COLLECT_STICKY_SAME_M,
                collectStickyStillSec,
                collectStickyRobotFlickerSec);
      } else {
        Translation2d cur = collectStickyPoint;
        selectedResource = (cur != null && collectValid.test(cur)) ? cur : bestCandidate;
      }

      Translation2d prevSticky = collectStickyPoint;

      if (pass == 0
          && prevSticky != null
          && selectedResource != null
          && !stickySame(prevSticky, selectedResource)) {
        Translation2d toPrev = prevSticky.minus(robotPos);
        Translation2d toNext = selectedResource.minus(robotPos);

        Translation2d prevHat = unitOrDefault(toPrev, new Translation2d(1.0, 0.0));
        Translation2d nextHat = unitOrDefault(toNext, prevHat);

        boolean opposite = dot(prevHat, nextHat) < 0.15;
        boolean tooSoon = nowSFromNs(nowNs - collectStickyLastSwitchNs) < 0.55;

        if ((opposite && !robotInCenterBand) || (tooSoon && collectStickyStillSec < 0.10)) {
          selectedResource = prevSticky;
          collectStickySelector.force(prevSticky);
        }
      }

      // Logger.recordOutput("sticky_dbg_out", selectedResource);
      // Logger.recordOutput("sticky_dbg_prev", collectStickyPoint);
      // Logger.recordOutput(
      // "sticky_dbg_out_dist_prev",
      // (collectStickyPoint != null && selectedResource != null)
      //     ? collectStickyPoint.getDistance(selectedResource)
      //     : -1.0);

      Pose2d selectedPose =
          (selectedResource != null)
              ? new Pose2d(selectedResource, new Rotation2d())
              : new Pose2d();
      // Logger.recordOutput("selectedResourceX", selectedPose);

      // selectedResource = canonicalizeCollectPoint(selectedResource, usePts);

      prevSticky = collectStickyPoint;
      boolean switched = stickySwitched(prevSticky, selectedResource);

      if (switched && collectStickyLastSwitchNs != 0L) {
        double sinceLastSwitchS = nowSFromNs(nowNs - collectStickyLastSwitchNs);
        double movedSinceLastSwitch =
            collectStickyLastSwitchRobotPos != null
                ? robotPos.getDistance(collectStickyLastSwitchRobotPos)
                : Double.POSITIVE_INFINITY;
        if (sinceLastSwitchS < COLLECT_SWITCH_COOLDOWN_S
            && movedSinceLastSwitch < COLLECT_SWITCH_MIN_MOVE_M) {
          if (prevSticky != null) {
            selectedResource = prevSticky;
            collectStickySelector.force(prevSticky);
            switched = false;
          }
        }
      }

      if (robotInCenterBand && collectStickyHalfLock == 0) {
        int s = sideSign(selectedResource);
        if (s != 0) collectStickyHalfLock = s;
      }

      collectStickyPoint = selectedResource;
      collectStickyScore = scoreResource.apply(selectedResource);
      collectStickyTsNs = nowNs;

      if (switched) {
        collectStickyReachedTsNs = 0L;
        collectStickyLastSwitchNs = nowNs;
        collectStickyLastSwitchRobotPos = robotPos;
        collectStickyNoProgressSinceNs = nowNs;
        collectStickyLastDistM = Double.POSITIVE_INFINITY;
        collectSnapActive = false;

        Translation2d dir = selectedResource.minus(robotPos);
        Translation2d hat = unitOrDefault(dir, new Translation2d(1.0, 0.0));

        double dist = robotPos.getDistance(selectedResource);
        double pushT = clamp01((dist - 0.35) / (2.50 - 0.35));
        double pushM = Math.min(lerp(0.42, 0.18, pushT), COLLECT_MAX_DRIVE_OFFSET_FROM_FUEL_M);

        collectStickyApproachHat = hat;
        collectStickyPushM = pushM;
        collectStickySide = 0;

        Translation2d frozen =
            computeFrozenDriveTarget(
                selectedResource,
                collectStickyApproachHat,
                collectStickyPushM,
                clampToFieldRobotSafe,
                inForbidden,
                violatesWall,
                nudgeOutOfForbidden);
        collectStickyDriveTarget = frozen;
        collectForcedDriveCand = null;
        collectForcedDriveSinceNs = 0L;
      }

      Translation2d desiredCollectPoint = collectStickyPoint;
      // Translation2d desiredCollectPoint = selectedResource;

      if (desiredCollectPoint == null) {
        clearCollectSticky();
        return fallbackCollectPose(robotPoseBlue);
      }
      Translation2d desiredDriveTarget = collectStickyDriveTarget;

      if (!collectValid.test(desiredCollectPoint)) {
        if (!footprintHasFuel.test(desiredCollectPoint)) {
          predictor.markCollectDepleted(desiredCollectPoint, COLLECT_CELL_M, 1.0);
          clearCollectSticky();
          collectStickySelector.forceInvalidate();
          // Logger.recordOutput("collect_sticky_invalid_footprint", 1.0);
          Translation2d fallbackTarget =
              (bestCandidate != null && collectValid.test(bestCandidate)) ? bestCandidate : null;
          if (fallbackTarget != null) {
            collectStickyPoint = fallbackTarget;
            collectStickyScore = scoreResource.apply(fallbackTarget);
            collectStickyTsNs = nowNs;
            collectStickyInvalidSec = 0.0;
            collectStickySelector.force(fallbackTarget);
            desiredCollectPoint = fallbackTarget;
          } else {
            return holdPose.apply(
                desiredDriveTarget != null ? desiredDriveTarget : robotPoseBlue.getTranslation());
          }
        }
        collectStickyInvalidSec += dt;
        // Logger.recordOutput("collect_sticky_invalid_s", collectStickyInvalidSec);

        if (collectStickyInvalidSec >= 0.55 && pass == 0) {
          predictor.markCollectDepleted(desiredCollectPoint, COLLECT_CELL_M, 1.0);
          clearCollectSticky();
        }

        return holdPose.apply(
            desiredDriveTarget != null ? desiredDriveTarget : robotPoseBlue.getTranslation());
      } else {
        collectStickyInvalidSec = 0.0;
      }

      if (desiredDriveTarget == null) {
        desiredDriveTarget =
            computeFrozenDriveTarget(
                desiredCollectPoint,
                collectStickyApproachHat != null
                    ? collectStickyApproachHat
                    : unit(desiredCollectPoint.minus(robotPos)),
                collectStickyPushM > 1e-6 ? collectStickyPushM : 0.18,
                clampToFieldRobotSafe,
                inForbidden,
                violatesWall,
                nudgeOutOfForbidden);
        collectStickyDriveTarget = desiredDriveTarget;
      }

      desiredDriveTarget = clampToFieldRobotSafe.apply(desiredDriveTarget);
      if (inForbidden.test(desiredDriveTarget))
        desiredDriveTarget = nudgeOutOfForbidden.apply(desiredDriveTarget);
      desiredDriveTarget = clampToFieldRobotSafe.apply(desiredDriveTarget);

      if (inForbidden.test(desiredDriveTarget) || violatesWall.test(desiredDriveTarget)) {
        desiredDriveTarget =
            computeFrozenDriveTarget(
                desiredCollectPoint,
                collectStickyApproachHat != null
                    ? collectStickyApproachHat
                    : unit(desiredCollectPoint.minus(robotPos)),
                collectStickyPushM > 1e-6 ? collectStickyPushM : 0.18,
                clampToFieldRobotSafe,
                inForbidden,
                violatesWall,
                nudgeOutOfForbidden);
        collectStickyDriveTarget = desiredDriveTarget;
      }

      double dToTarget = robotPos.getDistance(desiredDriveTarget);

      if (dToTarget <= COLLECT_STICKY_REACHED_M) {
        if (collectStickyReachedTsNs == 0L) collectStickyReachedTsNs = nowNs;
      } else {
        collectStickyReachedTsNs = 0L;
      }

      double dToCollect = robotPos.getDistance(desiredCollectPoint);

      boolean snapNow = collectSnapActive;
      if (!snapNow) {
        if (dToCollect <= COLLECT_SNAP_TO_POINT_M) snapNow = true;
      } else {
        if (dToCollect >= (COLLECT_SNAP_TO_POINT_M + COLLECT_SNAP_HYST_M)) snapNow = false;
      }
      collectSnapActive = snapNow;

      if (collectSnapActive) {
        desiredDriveTarget = desiredCollectPoint;
        collectStickyDriveTarget = desiredDriveTarget;
      }

      Translation2d forced =
          forceDriveTargetOntoFuel(
              robotPos,
              cap,
              desiredCollectPoint,
              desiredDriveTarget,
              clampToFieldRobotSafe,
              inForbidden,
              violatesWall,
              nudgeOutOfForbidden);

      Translation2d chosenForced = null;

      if (forced != null) {
        double dCand =
            (collectForcedDriveCand != null) ? collectForcedDriveCand.getDistance(forced) : 1e9;

        if (collectForcedDriveCand == null || dCand > 0.16) {
          collectForcedDriveCand = forced;
          collectForcedDriveSinceNs = nowNs;
        }

        double forcedStableS = (nowNs - collectForcedDriveSinceNs) / 1e9;

        double jump =
            (desiredDriveTarget != null)
                ? desiredDriveTarget.getDistance(collectForcedDriveCand)
                : 0.0;

        boolean allowBigJump = collectStickyReachedTsNs != 0L || dToTarget <= 0.90;
        boolean stableEnough = forcedStableS >= 0.32;

        if (stableEnough && (allowBigJump || jump <= 0.35)) {
          chosenForced = collectForcedDriveCand;
        }
      } else {
        collectForcedDriveCand = null;
        collectForcedDriveSinceNs = 0L;
      }

      if (chosenForced != null) {
        desiredDriveTarget = chosenForced;
        collectStickyDriveTarget = chosenForced;
      }

      if (desiredDriveTarget != null) {
        if (collectDriveLastTarget == null
            || collectDriveLastTarget.getDistance(desiredDriveTarget) > 0.12) {
          collectDriveLastTarget = desiredDriveTarget;
          collectDriveLastTargetNs = nowNs;
          collectStickyNoProgressSinceNs = nowNs;
          collectStickyLastDistM = robotPos.getDistance(desiredDriveTarget);
        }
      }

      double distNow = robotPos.getDistance(desiredDriveTarget);
      if (distNow + 1e-6 < collectStickyLastDistM - COLLECT_STICKY_NO_PROGRESS_DROP_M) {
        collectStickyLastDistM = distNow;
        collectStickyNoProgressSinceNs = nowNs;
      } else {
        double sinceS = nowSFromNs(nowNs - collectStickyNoProgressSinceNs);
        if (sinceS >= COLLECT_STICKY_NO_PROGRESS_S
            && distNow <= COLLECT_STICKY_NO_PROGRESS_MIN_DIST_M) {

          double sinceTargetChangeS = nowSFromNs(nowNs - collectDriveLastTargetNs);
          if (sinceTargetChangeS < 0.25) {
            collectStickyNoProgressSinceNs = nowNs;
            collectStickyLastDistM = distNow;
          } else {
            double cooldownS = nowSFromNs(nowNs - collectStickyLastSwitchNs);
            if (cooldownS >= COLLECT_STICKY_FLAP_COOLDOWN_S) {
              Translation2d hat =
                  collectStickyApproachHat != null
                      ? collectStickyApproachHat
                      : unit(desiredCollectPoint.minus(robotPos));
              Translation2d sideHat = perp(hat);

              int nextSide =
                  collectStickySide == 0
                      ? (sideSign(desiredCollectPoint) >= 0 ? 1 : -1)
                      : -collectStickySide;
              collectStickySide = nextSide;
              collectStickyLastSwitchNs = nowNs;
              collectStickyLastSwitchRobotPos = robotPos;

              Translation2d cand =
                  desiredCollectPoint
                      .plus(hat.times(collectStickyPushM))
                      .plus(sideHat.times((double) nextSide * 0.22));

              cand = clampToFieldRobotSafe.apply(cand);
              if (inForbidden.test(cand)) cand = nudgeOutOfForbidden.apply(cand);
              cand = clampToFieldRobotSafe.apply(cand);

              if (!inForbidden.test(cand) && !violatesWall.test(cand)) {
                collectStickyDriveTarget = cand;
                desiredDriveTarget = cand;
              }

              collectStickyNoProgressSinceNs = nowNs;
              collectStickyLastDistM = robotPos.getDistance(desiredDriveTarget);
            }
          }
        }
      }

      desiredDriveTarget = clampToFieldRobotSafe.apply(desiredDriveTarget);
      if (inForbidden.test(desiredDriveTarget))
        desiredDriveTarget = nudgeOutOfForbidden.apply(desiredDriveTarget);
      desiredDriveTarget = clampToFieldRobotSafe.apply(desiredDriveTarget);

      if (inForbidden.test(desiredDriveTarget) || violatesWall.test(desiredDriveTarget)) {
        Translation2d escape = nudgeOutOfForbidden.apply(desiredDriveTarget);
        escape = clampToFieldRobotSafe.apply(escape);
        if (!inForbidden.test(escape) && !violatesWall.test(escape)) {
          desiredDriveTarget = escape;
          collectStickyDriveTarget = escape;
        } else {
          double escapeX;
          double safePad = 0.60; // meters to push outside the band
          if (robotPos.getX() < midX) {
            escapeX = leftBandX0 - safePad;
          } else {
            escapeX = rightBandX1 + safePad;
          }
          Translation2d escape2 =
              clampToFieldRobotSafe.apply(new Translation2d(escapeX, robotPos.getY()));
          if (!inForbidden.test(escape2) && !violatesWall.test(escape2)) {
            desiredDriveTarget = escape2;
            collectStickyDriveTarget = escape2;
          } else {
            clearCollectSticky();
            return fallbackCollectPose(robotPoseBlue);
          }
        }
      }

      CollectProbe drivePr =
          predictor.probeCollect(desiredDriveTarget, COLLECT_EMPTY_DRIVE_PROBE_R_M);

      boolean driveLooksEmpty =
          drivePr == null || drivePr.count <= 0 || drivePr.units < COLLECT_EMPTY_DRIVE_MIN_UNITS;

      int liveFuelNearTarget =
          countLiveCollectResourcesWithin(
              dynUse, desiredDriveTarget, COLLECT_LIVE_FUEL_NEAR_TARGET_R_M);

      boolean nearTarget =
          robotPos.getDistance(desiredDriveTarget) <= COLLECT_REACHED_EMPTY_NEAR_TARGET_M;

      boolean reachedOrNear = (collectStickyReachedTsNs != 0L) || nearTarget;

      if (reachedOrNear && liveFuelNearTarget <= 0 && driveLooksEmpty) {
        collectReachedEmptySec += dt;
      } else {
        collectReachedEmptySec = 0.0;
      }

      // Logger.recordOutput("collect_reached_empty_s", collectReachedEmptySec);
      // Logger.recordOutput("collect_live_fuel_near_target_n", liveFuelNearTarget);

      if (collectReachedEmptySec >= COLLECT_REACHED_EMPTY_FORCE_DROP_SEC && pass == 0) {
        Pose2d hold = holdPose.apply(desiredDriveTarget);
        if (desiredCollectPoint != null) {
          predictor.markCollectDepleted(desiredCollectPoint, COLLECT_CELL_M, 1.0);
        }
        clearCollectSticky();
        return hold;
      }

      if (robotPos.getDistance(desiredDriveTarget) <= COLLECT_EMPTY_DRIVE_NEAR_ROBOT_M) {
        if (driveLooksEmpty) collectEmptyDriveSec += dt;
        else collectEmptyDriveSec = 0.0;
      } else {
        collectEmptyDriveSec = 0.0;
      }

      // Logger.recordOutput("collect_empty_drive_s", collectEmptyDriveSec);

      if (collectEmptyDriveSec >= COLLECT_EMPTY_DRIVE_DONE_SEC && pass == 0) {
        Pose2d hold = holdPose.apply(desiredDriveTarget);
        clearCollectSticky();
        return hold;
      }

      double stepDist = 0.0;
      double speed = 0.0;

      if (collectStuckAnchorNs == 0L) {
        collectStuckAnchorNs = nowNs;
        collectStuckAnchorPos = robotPos;
        collectStuckSec = 0.0;
        lastRobotPosForStuck = robotPos;
      } else {
        stepDist = robotPos.getDistance(lastRobotPosForStuck);
        speed = stepDist / Math.max(1e-3, dt);
        lastRobotPosForStuck = robotPos;

        double movedFromAnchor = robotPos.getDistance(collectStuckAnchorPos);

        if (movedFromAnchor >= COLLECT_STUCK_RESET_MOVE_M) {
          collectStuckAnchorNs = nowNs;
          collectStuckAnchorPos = robotPos;
          collectStuckSec = 0.0;
        } else {
          boolean lowSpeed = speed <= COLLECT_STUCK_SPEED_MPS;
          boolean nearAnchor = movedFromAnchor <= COLLECT_STUCK_RADIUS_M;

          if (lowSpeed && nearAnchor) collectStuckSec += dt;
          else collectStuckSec = 0.0;
        }
      }

      if (nearbyFuelCount < COLLECT_NEARBY_MIN_COUNT) collectNoFuelSec += dt;
      else collectNoFuelSec = 0.0;

      boolean patchDone = collectNoFuelSec >= COLLECT_DONE_NO_FUEL_SEC;
      boolean stuckTooLong = collectStuckSec >= COLLECT_DONE_STUCK_SEC;

      if ((patchDone || stuckTooLong) && pass == 0) {
        Pose2d hold = holdPose.apply(desiredDriveTarget);
        if (stuckTooLong && desiredCollectPoint != null) {
          predictor.markCollectDepleted(desiredCollectPoint, COLLECT_CELL_M, 0.65);
        }
        clearCollectSticky();
        return hold;
      }

      // Logger.recordOutput("collect_resource_xy", desiredCollectPoint);
      // Logger.recordOutput("collect_target_xy", desiredDriveTarget);
      // Logger.recordOutput("collect_points_n", usePts.length);
      // Logger.recordOutput("collect_snap_active", collectSnapActive);
      // Logger.recordOutput("collect_sticky_side", collectStickySide);
      // Logger.recordOutput("collect_nearby_fuel_n", nearbyFuelCount);
      // Logger.recordOutput("collect_done_nofuel_s", collectNoFuelSec);
      // Logger.recordOutput("collect_done_stuck_s", collectStuckSec);

      if (best != null) {
        // Logger.recordOutput("collect_score", best.score);
        // Logger.recordOutput("collect_value", best.value);
        // Logger.recordOutput("collect_our_eta", best.ourEtaS);
        // Logger.recordOutput("collect_enemy_pressure", best.enemyPressure);
        // Logger.recordOutput("collect_ally_congestion", best.allyCongestion);
        // Logger.recordOutput("collect_intent_enemy", best.enemyIntent);
        // Logger.recordOutput("collect_intent_ally", best.allyIntent);
      }

      return new Pose2d(desiredDriveTarget, best != null ? best.rotation : Rotation2d.kZero);
    }

    clearCollectSticky();
    return fallbackCollectPose(robotPoseBlue);
  }

  private Translation2d forceDriveTargetOntoFuel(
      Translation2d robotPos,
      double cap,
      Translation2d collectPoint,
      Translation2d driveSeed,
      java.util.function.Function<Translation2d, Translation2d> clampToFieldRobotSafe,
      java.util.function.Predicate<Translation2d> inForbidden,
      java.util.function.Predicate<Translation2d> violatesWall,
      java.util.function.Function<Translation2d, Translation2d> nudgeOutOfForbidden) {

    Translation2d drive = driveSeed != null ? driveSeed : collectPoint;
    if (drive == null) return null;

    drive = clampToFieldRobotSafe.apply(drive);
    if (inForbidden.test(drive)) drive = nudgeOutOfForbidden.apply(drive);
    drive = clampToFieldRobotSafe.apply(drive);

    if (inForbidden.test(drive) || violatesWall.test(drive)) return null;

    Translation2d near = predictor.nearestCollectResource(drive, COLLECT_SNAP_TO_NEAREST_FUEL_M);
    if (near != null) {
      Translation2d snapped = clampToFieldRobotSafe.apply(near);
      if (inForbidden.test(snapped)) snapped = nudgeOutOfForbidden.apply(snapped);
      snapped = clampToFieldRobotSafe.apply(snapped);
      if (!inForbidden.test(snapped) && !violatesWall.test(snapped)) return snapped;
    }

    Translation2d centroid = predictor.snapToCollectCentroid(drive, 0.75, 0.15);
    if (centroid != null) {
      centroid = clampToFieldRobotSafe.apply(centroid);
      if (inForbidden.test(centroid)) centroid = nudgeOutOfForbidden.apply(centroid);
      centroid = clampToFieldRobotSafe.apply(centroid);
      if (!inForbidden.test(centroid) && !violatesWall.test(centroid)) {
        Translation2d near2 =
            predictor.nearestCollectResource(centroid, COLLECT_SNAP_TO_NEAREST_FUEL_M);
        if (near2 != null) {
          Translation2d snapped2 = clampToFieldRobotSafe.apply(near2);
          if (inForbidden.test(snapped2)) snapped2 = nudgeOutOfForbidden.apply(snapped2);
          snapped2 = clampToFieldRobotSafe.apply(snapped2);
          if (!inForbidden.test(snapped2) && !violatesWall.test(snapped2)) return snapped2;
        }
        return centroid;
      }
    }

    Translation2d fallback = clampToFieldRobotSafe.apply(collectPoint);
    if (inForbidden.test(fallback)) fallback = nudgeOutOfForbidden.apply(fallback);
    fallback = clampToFieldRobotSafe.apply(fallback);
    if (!inForbidden.test(fallback) && !violatesWall.test(fallback)) return fallback;

    return drive;
  }

  public Pose2d nextCollectionGoalBlue(Pose2d robotPoseBlue, double ourSpeedCap, int goalUnits) {
    return nextObjectiveGoalBlue(robotPoseBlue, ourSpeedCap, goalUnits, CategorySpec.kCollect);
  }

  public FieldVision createFieldVision(String name) {
    return new FieldVision(this, name);
  }
}
