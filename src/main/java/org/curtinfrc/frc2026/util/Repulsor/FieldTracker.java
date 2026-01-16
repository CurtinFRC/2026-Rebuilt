// File: src/main/java/org/curtinfrc/frc2026/util/Repulsor/FieldTracker.java
package org.curtinfrc.frc2026.util.Repulsor;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import java.util.function.Predicate;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldLayoutProvider;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldMapBuilder.CategorySpec;
import org.curtinfrc.frc2026.util.Repulsor.Fields.Rebuilt2026;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.RepulsorSetpoint;
import org.littletonrobotics.junction.Logger;

public class FieldTracker {
  private static volatile FieldTracker instance;
  private static volatile FieldLayoutProvider defaultProvider = new Rebuilt2026();

  private static final double COLLECT_CELL_M = 0.45;
  private static final int COLLECT_COARSE_TOPK = 4;
  private static final int COLLECT_REFINE_GRID = 3;

  private static final double COLLECT_STICKY_MIN_HOLD_S_FAR = 0.18;
  private static final double COLLECT_STICKY_MIN_HOLD_S_NEAR = 0.65;

  private static final double COLLECT_STICKY_MAX_HOLD_S_FAR = 0.55;
  private static final double COLLECT_STICKY_MAX_HOLD_S_NEAR = 1.60;

  private static final double COLLECT_STICKY_SWITCH_MARGIN_FAR = 0.18;
  private static final double COLLECT_STICKY_SWITCH_MARGIN_NEAR = 0.70;

  private static final double COLLECT_STICKY_REACHED_M = 0.65;
  private static final double COLLECT_STICKY_SAME_M = 0.10;

  private static final double COLLECT_STICKY_NEAR_DIST_M = 1.10;
  private static final double COLLECT_STICKY_FAR_DIST_M = 3.80;

  private static double lerp(double a, double b, double t) {
    return a + (b - a) * t;
  }

  private static double clamp01(double x) {
    if (x < 0.0) return 0.0;
    if (x > 1.0) return 1.0;
    return x;
  }

  private volatile Translation2d collectStickyPoint = null;
  private volatile double collectStickyScore = -1e18;
  private volatile long collectStickyTsNs = 0L;

  public static FieldTracker getInstance() {
    FieldTracker local = instance;
    if (local == null) {
      synchronized (FieldTracker.class) {
        local = instance;
        if (local == null) {
          local = new FieldTracker(defaultProvider);
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
    synchronized (FieldTracker.class) {
      instance = new FieldTracker(provider);
    }
  }

  public FieldTracker() {
    this(defaultProvider);
  }

  public FieldTracker(FieldLayoutProvider provider) {
    if (provider == null) throw new IllegalArgumentException("provider cannot be null");
    this.field_map = provider.build(this);
    this.predictor = new PredictiveFieldState();
    this.collectCache = new ObjectiveCache();
    rebuildObjectiveCaches();
    predictor.registerResourceSpec("fuel", new PredictiveFieldState.ResourceSpec(0.38, 1.0, 0.95));
  }

  public abstract class PrimitiveObject {
    public abstract boolean intersects(Pose3d pos);
  }

  public class Pipe extends PrimitiveObject {
    private Pose3d position;
    private Distance radius;
    private Angle angle;

    public Pipe(Pose3d position, Distance radius) {
      this(position, radius, Radians.of(0));
    }

    public Pipe(Pose3d position, Distance radius, Angle angle) {
      this.position = position;
      this.radius = radius;
      this.angle = angle;
    }

    public Pose3d getPosition() {
      return position;
    }

    public void setPosition(Pose3d position) {
      this.position = position;
    }

    public Distance getRadius() {
      return radius;
    }

    public void setRadius(Distance radius) {
      this.radius = radius;
    }

    public Angle getAngle() {
      return angle;
    }

    public void setAngle(Angle angle) {
      this.angle = angle;
    }

    @Override
    public boolean intersects(Pose3d pos) {
      double dx = pos.getX() - position.getX();
      double dy = pos.getY() - position.getY();
      double dz = pos.getZ() - position.getZ();
      double r = radius.in(Meters);
      return (dx * dx + dy * dy + dz * dz) <= r * r;
    }
  }

  public static final class GameObject {
    private final String id;
    private final String type;
    private final Pose3d position;

    public GameObject(String id, String type) {
      this(id, type, null);
    }

    public GameObject(String id, String type, Pose3d position) {
      if (id == null || id.isEmpty()) throw new IllegalArgumentException("id cannot be null/empty");
      if (type == null || type.isEmpty())
        throw new IllegalArgumentException("type cannot be null/empty");
      this.id = id;
      this.type = type;
      this.position = position;
    }

    public String getId() {
      return id;
    }

    public String getType() {
      return type;
    }

    public Pose3d getPosition() {
      return position;
    }
  }

  public class GameElementModel {
    private PrimitiveObject[] composition = new PrimitiveObject[0];
    private Pose3d position;

    public GameElementModel(Pose3d position) {
      this.position = position;
    }

    public GameElementModel(Pose3d position, PrimitiveObject[] composition) {
      this.position = position;
      this.composition = composition != null ? composition : new PrimitiveObject[0];
    }

    public Pose3d getPosition() {
      return position;
    }

    public void setPosition(Pose3d position) {
      this.position = position;
    }

    public PrimitiveObject[] getComposition() {
      return composition;
    }

    public void setComposition(PrimitiveObject[] composition) {
      this.composition = composition != null ? composition : new PrimitiveObject[0];
    }
  }

  public class GameElement {
    public enum Alliance {
      kBlue,
      kRed
    }

    private final GameObject[] containedStorage;
    private int containedCount;
    private int maxContained;

    private GameElementModel model;
    private Predicate<GameObject> filter;
    private Optional<RepulsorSetpoint> relatedPoint = Optional.empty();
    private CategorySpec category;
    private Alliance alliance;

    private GameObject[] cachedExact;
    private boolean dirtyCache;

    public GameElement(GameElementModel model) {
      this(Alliance.kBlue, 10, model, g -> true, null, CategorySpec.kScore);
    }

    public GameElement(
        Alliance alliance,
        int maxContained,
        GameElementModel model,
        Predicate<GameObject> filter,
        RepulsorSetpoint relatedPoint,
        CategorySpec category) {
      if (alliance == null) throw new IllegalArgumentException("Alliance cannot be null");
      if (maxContained < 0) throw new IllegalArgumentException("Max contained cannot be negative");
      this.alliance = alliance;
      this.maxContained = maxContained;
      this.model = model;
      this.filter = (filter != null) ? filter : (go -> true);
      if (relatedPoint != null) this.relatedPoint = Optional.ofNullable(relatedPoint);
      this.category = category;
      this.containedStorage = new GameObject[Math.max(1, maxContained)];
      this.containedCount = 0;
      this.cachedExact = new GameObject[0];
      this.dirtyCache = true;
    }

    public boolean filter(GameObject gameObject) {
      if (gameObject == null) throw new IllegalArgumentException("GameObject cannot be null");
      return filter.test(gameObject);
    }

    public Alliance getAlliance() {
      return alliance;
    }

    public CategorySpec getCategory() {
      return category;
    }

    public void setAlliance(Alliance alliance) {
      if (alliance == null) throw new IllegalArgumentException("Alliance cannot be null");
      this.alliance = alliance;
    }

    public int getMaxContained() {
      return maxContained;
    }

    public void setMaxContained(int maxContained) {
      if (maxContained < 0) throw new IllegalArgumentException("Max contained cannot be negative");
      this.maxContained = maxContained;
      if (containedCount > maxContained) {
        containedCount = maxContained;
        dirtyCache = true;
      }
    }

    public GameObject[] getContained() {
      if (!dirtyCache && cachedExact.length == containedCount) {
        return cachedExact;
      }
      GameObject[] out = new GameObject[containedCount];
      if (containedCount > 0) {
        System.arraycopy(containedStorage, 0, out, 0, containedCount);
      }
      cachedExact = out;
      dirtyCache = false;
      return out;
    }

    public void setContained(GameObject[] contained) {
      clearContained();
      if (contained == null || contained.length == 0) return;
      int n = Math.min(contained.length, maxContained);
      for (int i = 0; i < n; i++) {
        containedStorage[i] = contained[i];
      }
      containedCount = n;
      dirtyCache = true;
    }

    public GameObject getContained(int index) {
      if (index < 0 || index >= containedCount) {
        throw new IndexOutOfBoundsException("Index: " + index + ", Size: " + containedCount);
      }
      return containedStorage[index];
    }

    public int getContainedCount() {
      return containedCount;
    }

    public boolean isAtCapacity() {
      return containedCount >= maxContained;
    }

    public GameElementModel getModel() {
      return model;
    }

    public void setModel(GameElementModel model) {
      this.model = model;
    }

    public Optional<RepulsorSetpoint> getRelatedPoint() {
      return relatedPoint;
    }

    public void setRelatedPoint(RepulsorSetpoint newPoint) {
      this.relatedPoint = Optional.ofNullable(newPoint);
    }

    public void setFilter(Predicate<GameObject> filter) {
      this.filter = (filter != null) ? filter : (go -> true);
    }

    public void setCategory(CategorySpec category) {
      this.category = category;
    }

    public void clearContained() {
      if (containedCount != 0) {
        for (int i = 0; i < containedCount; i++) {
          containedStorage[i] = null;
        }
        containedCount = 0;
        dirtyCache = true;
      }
    }

    public boolean tryAdd(GameObject obj) {
      if (obj == null) return false;
      if (containedCount >= maxContained) return false;
      containedStorage[containedCount++] = obj;
      dirtyCache = true;
      return true;
    }
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

  private PredictiveFieldState predictor;

  public PredictiveFieldState getPredictor() {
    return predictor;
  }

  public void updatePredictorWorld(GameElement.Alliance ours) {
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
      GameElement.Alliance alliance, Translation2d from, CategorySpec cat) {
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

  public List<PredictiveFieldState.Candidate> getPredictedCandidates(
      GameElement.Alliance alliance,
      Translation2d ourPos,
      double ourSpeedCap,
      CategorySpec cat,
      int limit) {
    updatePredictorWorld(alliance);
    return predictor.rank(ourPos, ourSpeedCap, cat, limit);
  }

  public List<RepulsorSetpoint> getPredictedSetpoints(
      GameElement.Alliance alliance,
      Translation2d ourPos,
      double ourSpeedCap,
      CategorySpec cat,
      int limit) {
    List<PredictiveFieldState.Candidate> c =
        getPredictedCandidates(alliance, ourPos, ourSpeedCap, cat, limit);
    ArrayList<RepulsorSetpoint> out = new ArrayList<>(c.size());
    for (PredictiveFieldState.Candidate k : c) out.add(k.setpoint);
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

  private void ingestTracked(String id, String type, Pose3d p, long nowNs) {
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
    collectStickyPoint = null;
    collectStickyScore = -1e18;
    collectStickyTsNs = 0L;
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

  private static final double DYN_STALE_S = 0.3;

  private List<PredictiveFieldState.DynamicObject> snapshotDynamics() {
    long nowNs = System.nanoTime();
    ArrayList<PredictiveFieldState.DynamicObject> out = new ArrayList<>(tracked.size());
    for (TrackedObj o : tracked.values()) {
      if (o == null) continue;
      Pose3d p = o.pos;
      if (p == null) continue;
      long t = o.tNs;
      if (t == 0L) continue;
      double ageS = (nowNs - t) / 1e9;
      if (ageS < 0.0 || ageS > DYN_STALE_S) continue;
      String ty = o.type != null ? o.type : "unknown";
      out.add(
          new PredictiveFieldState.DynamicObject(
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

  // Replace nextObjectiveGoalBlue() with this version:

  public Pose2d nextObjectiveGoalBlue(
      Pose2d robotPoseBlue, double ourSpeedCap, int goalUnits, CategorySpec cat) {
    if (robotPoseBlue == null) return Pose2d.kZero;
    double cap = Math.max(0.2, ourSpeedCap);

    ObjectiveCache cache = cat == CategorySpec.kCollect ? collectCache : collectCache;
    Translation2d[] pts = cache.points;

    if (pts == null || pts.length == 0) {
      return new Pose2d(
          Constants.FIELD_LENGTH * 0.5, Constants.FIELD_WIDTH * 0.5, robotPoseBlue.getRotation());
    }

    predictor.setDynamicObjects(snapshotDynamics());

    PredictiveFieldState.PointCandidate best =
        predictor.rankCollectHierarchical(
            robotPoseBlue.getTranslation(),
            cap,
            pts,
            COLLECT_CELL_M,
            goalUnits,
            COLLECT_COARSE_TOPK,
            COLLECT_REFINE_GRID);

    long nowNs = System.nanoTime();
    Translation2d proposed = best != null ? best.point : null;
    double proposedScore = best != null ? best.score : -1e18;

    Translation2d sticky = collectStickyPoint;
    double stickyScore = collectStickyScore;
    long stickyTs = collectStickyTsNs;

    Translation2d chosen = proposed != null ? proposed : (sticky != null ? sticky : pts[0]);
    double chosenScore = proposed != null ? proposedScore : (sticky != null ? stickyScore : -1e18);

    if (sticky != null) {
      double distToSticky = robotPoseBlue.getTranslation().getDistance(sticky);

      double t =
          clamp01(
              (distToSticky - COLLECT_STICKY_NEAR_DIST_M)
                  / (COLLECT_STICKY_FAR_DIST_M - COLLECT_STICKY_NEAR_DIST_M));

      double minHoldS = lerp(COLLECT_STICKY_MIN_HOLD_S_NEAR, COLLECT_STICKY_MIN_HOLD_S_FAR, t);
      double maxHoldS = lerp(COLLECT_STICKY_MAX_HOLD_S_NEAR, COLLECT_STICKY_MAX_HOLD_S_FAR, t);
      double switchMargin =
          lerp(COLLECT_STICKY_SWITCH_MARGIN_NEAR, COLLECT_STICKY_SWITCH_MARGIN_FAR, t);

      double holdS = stickyTs != 0L ? (nowNs - stickyTs) / 1e9 : 1e9;
      boolean reached = distToSticky <= COLLECT_STICKY_REACHED_M;

      if (!reached) {
        boolean minHold = holdS < minHoldS;
        boolean maxHold = holdS > maxHoldS;

        if (proposed == null) {
          chosen = sticky;
          chosenScore = stickyScore;
        } else if (samePoint(proposed, sticky, COLLECT_STICKY_SAME_M)) {
          chosen = sticky;
          chosenScore = Math.max(stickyScore, proposedScore);
        } else if (minHold) {
          if (proposedScore < stickyScore + switchMargin) {
            chosen = sticky;
            chosenScore = stickyScore;
          }
        } else if (!maxHold) {
          if (proposedScore < stickyScore + switchMargin) {
            chosen = sticky;
            chosenScore = stickyScore;
          }
        }
      }

      if (reached) {
        collectStickyPoint = null;
        collectStickyScore = -1e18;
        collectStickyTsNs = 0L;
        sticky = null;
      }
    }

    if (chosen != null) {
      Translation2d curSticky = collectStickyPoint;
      if (curSticky == null || !samePoint(curSticky, chosen, COLLECT_STICKY_SAME_M)) {
        collectStickyPoint = chosen;
        collectStickyScore = chosenScore;
        collectStickyTsNs = nowNs;
      } else {
        collectStickyScore = Math.max(collectStickyScore, chosenScore);
      }
    }

    Logger.recordOutput("collect_target_xy", chosen);
    Logger.recordOutput("collect_points_n", pts.length);
    Logger.recordOutput("collect_sticky_active", collectStickyPoint != null);
    Logger.recordOutput(
        "collect_sticky_age_s", collectStickyTsNs != 0L ? (nowNs - collectStickyTsNs) / 1e9 : 0.0);

    if (best != null) {
      Logger.recordOutput("collect_score", best.score);
      Logger.recordOutput("collect_value", best.value);
      Logger.recordOutput("collect_our_eta", best.ourEtaS);
      Logger.recordOutput("collect_enemy_pressure", best.enemyPressure);
      Logger.recordOutput("collect_ally_congestion", best.allyCongestion);
      Logger.recordOutput("collect_intent_enemy", best.enemyIntent);
      Logger.recordOutput("collect_intent_ally", best.allyIntent);
    }

    Translation2d target = chosen != null ? chosen : pts[0];
    return new Pose2d(target, robotPoseBlue.getRotation());
  }

  public Pose2d nextCollectionGoalBlue(Pose2d robotPoseBlue, double ourSpeedCap, int goalUnits) {
    return nextObjectiveGoalBlue(robotPoseBlue, ourSpeedCap, goalUnits, CategorySpec.kCollect);
  }

  public class FieldVision {
    private static final int MAX_OBJECTS_PER_TICK = 256;

    private static final class FieldVisionData {
      private final Pose3d position;
      private final String type;

      FieldVisionData(Pose3d position, String type) {
        this.position = position;
        this.type = type;
      }

      public Pose3d getPosition() {
        return position;
      }

      public String getType() {
        return type;
      }
    }

    private final String name;
    private final String host;
    private final NetworkTable table;

    private final HashMap<String, FieldVisionData> objects = new HashMap<>(256);

    public FieldVision(String name) {
      if (name == null || name.isEmpty())
        throw new IllegalArgumentException("name cannot be null/empty");
      this.name = name;
      this.host = name + "-vision";
      this.table = NetworkTableInstance.getDefault().getTable("FieldVision/" + name);
    }

    public String getName() {
      return name;
    }

    public String getHost() {
      return host;
    }

    public void update(Pose2d currentPose) {
      if (currentPose == null) return;

      Pose3d field_T_robot =
          new Pose3d(
              currentPose.getX(),
              currentPose.getY(),
              0.0,
              new Rotation3d(0.0, 0.0, currentPose.getRotation().getRadians()));

      double ex = table.getEntry("extrinsics/x").getDouble(0.0);
      double ey = table.getEntry("extrinsics/y").getDouble(0.0);
      double ez = table.getEntry("extrinsics/z").getDouble(0.0);
      double eroll = table.getEntry("extrinsics/roll").getDouble(0.0);
      double epitch = table.getEntry("extrinsics/pitch").getDouble(0.0);
      double eyaw = table.getEntry("extrinsics/yaw").getDouble(0.0);

      Transform3d robot_T_camera =
          new Transform3d(new Translation3d(ex, ey, ez), new Rotation3d(eroll, epitch, eyaw));

      Pose3d field_T_camera =
          field_T_robot.transformBy(
              new Transform3d(robot_T_camera.getTranslation(), robot_T_camera.getRotation()));

      objects.clear();
      Set<String> keys = table.getKeys();
      int seen = 0;

      long nowNs = System.nanoTime();

      for (String key : keys) {
        if (seen >= MAX_OBJECTS_PER_TICK) break;
        if (!key.startsWith("object_")) continue;

        String objectId = key.substring(7);

        String frame = table.getEntry(key + "/frame").getString("field");

        double roll = table.getEntry(key + "/roll").getDouble(0.0);
        double pitch = table.getEntry(key + "/pitch").getDouble(0.0);
        double yaw = table.getEntry(key + "/yaw").getDouble(0.0);
        Rotation3d localRot = new Rotation3d(roll, pitch, yaw);

        String rawType = table.getEntry(key + "/type").getString("unknown");
        String type = canonicalizeType(rawType);

        Pose3d fieldPose;

        if ("camera".equalsIgnoreCase(frame)) {
          double px = table.getEntry(key + "/px").getDouble(0.0);
          double py = table.getEntry(key + "/py").getDouble(0.0);
          double pz = table.getEntry(key + "/pz").getDouble(0.0);
          Pose3d camera_T_object = new Pose3d(px, py, pz, localRot);
          fieldPose =
              field_T_camera.transformBy(
                  new Transform3d(camera_T_object.getTranslation(), camera_T_object.getRotation()));
        } else if ("robot".equalsIgnoreCase(frame)) {
          double px = table.getEntry(key + "/px").getDouble(0.0);
          double py = table.getEntry(key + "/py").getDouble(0.0);
          double pz = table.getEntry(key + "/pz").getDouble(0.0);
          Pose3d robot_T_object = new Pose3d(px, py, pz, localRot);
          fieldPose =
              field_T_robot.transformBy(
                  new Transform3d(robot_T_object.getTranslation(), robot_T_object.getRotation()));
        } else {
          double x = table.getEntry(key + "/x").getDouble(0.0);
          double y2 = table.getEntry(key + "/y").getDouble(0.0);
          double z = table.getEntry(key + "/z").getDouble(0.0);
          fieldPose = new Pose3d(x, y2, z, localRot);
        }

        objects.put(objectId, new FieldVisionData(fieldPose, type));
        ingestTracked(objectId, type, fieldPose, nowNs);
        seen++;
      }

      GameElement[] fm = field_map;
      if (fm == null || fm.length == 0) return;

      for (GameElement element : fm) {
        if (element != null) element.clearContained();
      }

      for (var entry : objects.entrySet()) {
        String objectId = entry.getKey();
        FieldVisionData data = entry.getValue();
        Pose3d position = data.getPosition();
        String type = data.getType();
        GameObject obj = new GameObject(objectId, type, position);

        for (GameElement element : fm) {
          if (element == null) continue;
          if (element.getContainedCount() >= element.getMaxContained()) continue;
          if (!element.filter(obj)) continue;

          PrimitiveObject[] primitives =
              element.getModel() != null ? element.getModel().getComposition() : null;
          if (primitives == null || primitives.length == 0) continue;

          boolean hit = false;
          for (PrimitiveObject primitive : primitives) {
            if (primitive != null && primitive.intersects(position)) {
              hit = true;
              break;
            }
          }
          if (hit) {
            element.tryAdd(obj);
          }
        }
      }
    }
  }
}
