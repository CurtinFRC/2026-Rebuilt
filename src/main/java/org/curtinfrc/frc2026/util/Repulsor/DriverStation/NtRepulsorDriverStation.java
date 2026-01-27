package org.curtinfrc.frc2026.util.Repulsor.DriverStation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import org.curtinfrc.frc2026.util.Repulsor.Simulation.NetworkTablesValue;

public abstract class NtRepulsorDriverStation extends RepulsorDriverStation {
  protected final NetworkTableInstance inst;
  protected final String root;

  private final List<NetworkTablesValue<?>> owned = new ArrayList<>();

  private final Map<String, NetworkTablesValue<Boolean>> configBools = new HashMap<>();
  private final Map<String, NetworkTablesValue<Double>> configDoubles = new HashMap<>();
  private final Map<String, NetworkTablesValue<Long>> configInts = new HashMap<>();
  private final Map<String, NetworkTablesValue<String>> configStrings = new HashMap<>();
  private final Map<String, NetworkTablesValue<double[]>> configDoubleArrays = new HashMap<>();

  private final Map<String, PoseOverrideCommand> poseOverrideCommands = new HashMap<>();
  private final Map<String, PoseResetCommand> poseResetCommands = new HashMap<>();
  private final Map<String, GoalSetpointCommand> goalSetpointCommands = new HashMap<>();

  protected NtRepulsorDriverStation(NetworkTableInstance inst, String root) {
    this.inst = Objects.requireNonNull(inst, "inst");
    if (root == null || root.isEmpty()) throw new IllegalArgumentException("root");
    this.root = normalizeRoot(root);

    declareSharedConfig(new Schema(this));
    flushAll();
  }

  protected abstract void declareSharedConfig(Schema schema);

  protected final String configPath(String key) {
    return root + "/config/" + normalizeKey(key);
  }

  protected final String commandPath(String key) {
    return root + "/commands/" + normalizeKey(key);
  }

  public final boolean getConfigBool(String key) {
    NetworkTablesValue<Boolean> v = require(configBools, key);
    return Boolean.TRUE.equals(v.get());
  }

  public final void setConfigBool(String key, boolean value) {
    require(configBools, key).set(value);
  }

  public final double getConfigDouble(String key) {
    NetworkTablesValue<Double> v = require(configDoubles, key);
    Double d = v.get();
    return d != null ? d : 0.0;
  }

  public final void setConfigDouble(String key, double value) {
    require(configDoubles, key).set(value);
  }

  public final long getConfigInt(String key) {
    NetworkTablesValue<Long> v = require(configInts, key);
    Long d = v.get();
    return d != null ? d : 0L;
  }

  public final void setConfigInt(String key, long value) {
    require(configInts, key).set(value);
  }

  public final String getConfigString(String key) {
    NetworkTablesValue<String> v = require(configStrings, key);
    String s = v.get();
    return s != null ? s : "";
  }

  public final void setConfigString(String key, String value) {
    require(configStrings, key).set(value != null ? value : "");
  }

  public final double[] getConfigDoubleArray(String key) {
    NetworkTablesValue<double[]> v = require(configDoubleArrays, key);
    double[] a = v.get();
    return a != null ? a : new double[0];
  }

  public final void setConfigDoubleArray(String key, double[] value) {
    require(configDoubleArrays, key).set(value != null ? value : new double[0]);
  }

  public final Optional<Pose2d> consumePoseOverride(String name) {
    PoseOverrideCommand cmd = require(poseOverrideCommands, name);
    return cmd.consume();
  }

  public final void requestPoseOverride(String name, Pose2d pose, boolean enabled) {
    PoseOverrideCommand cmd = require(poseOverrideCommands, name);
    cmd.request(pose, enabled);
  }

  public final Optional<Pose2d> consumePoseReset(String name) {
    PoseResetCommand cmd = require(poseResetCommands, name);
    return cmd.consume();
  }

  public final void requestPoseReset(String name, Pose2d pose) {
    PoseResetCommand cmd = require(poseResetCommands, name);
    cmd.request(pose);
  }

  public final Optional<Pose2d> forcedGoalPose(String name) {
    GoalSetpointCommand cmd = require(goalSetpointCommands, name);
    return cmd.forcedPose();
  }

  public final Optional<GoalSetpoint> consumeGoalSetpointApplied(String name) {
    GoalSetpointCommand cmd = require(goalSetpointCommands, name);
    return cmd.consumeApplied();
  }

  public final void requestGoalSetpoint(String name, Pose2d goalPose, boolean enabled) {
    GoalSetpointCommand cmd = require(goalSetpointCommands, name);
    cmd.request(goalPose, enabled);
  }

  @Override
  public final void tick() {
    for (PoseOverrideCommand c : poseOverrideCommands.values()) c.tick();
    for (PoseResetCommand c : poseResetCommands.values()) c.tick();
    for (GoalSetpointCommand c : goalSetpointCommands.values()) c.tick();
    onTick();
  }

  protected void onTick() {}

  public final void flushAll() {
    for (NetworkTablesValue<?> v : owned) v.flush();
  }

  @Override
  public void close() {
    for (NetworkTablesValue<?> v : owned) v.close();
    owned.clear();
    configBools.clear();
    configDoubles.clear();
    configInts.clear();
    configStrings.clear();
    configDoubleArrays.clear();
    poseOverrideCommands.clear();
    poseResetCommands.clear();
    goalSetpointCommands.clear();
  }

  public static final class GoalSetpoint {
    public final Pose2d pose;
    public final boolean enabled;

    public GoalSetpoint(Pose2d pose, boolean enabled) {
      this.pose = pose != null ? pose : new Pose2d();
      this.enabled = enabled;
    }
  }

  protected final class Schema {
    private final NtRepulsorDriverStation ds;

    private Schema(NtRepulsorDriverStation ds) {
      this.ds = ds;
    }

    public void configBool(String key, boolean initialValue) {
      String k = normalizeKey(key);
      putUnique(configBools, k, own(NetworkTablesValue.ofBoolean(inst, configPath(k), initialValue)));
    }

    public void configDouble(String key, double initialValue) {
      String k = normalizeKey(key);
      putUnique(
          configDoubles, k, own(NetworkTablesValue.ofDouble(inst, configPath(k), initialValue)));
    }

    public void configInt(String key, long initialValue) {
      String k = normalizeKey(key);
      putUnique(configInts, k, own(NetworkTablesValue.ofInteger(inst, configPath(k), initialValue)));
    }

    public void configString(String key, String initialValue) {
      String k = normalizeKey(key);
      putUnique(
          configStrings, k, own(NetworkTablesValue.ofString(inst, configPath(k), initialValue)));
    }

    public void configDoubleArray(String key, double[] initialValue) {
      String k = normalizeKey(key);
      putUnique(
          configDoubleArrays,
          k,
          own(NetworkTablesValue.ofDoubleArray(inst, configPath(k), initialValue)));
    }

    public void poseOverrideCommand(String name, Pose2d initialPose, boolean initialEnabled) {
      String k = normalizeKey(name);
      putUnique(poseOverrideCommands, k, new PoseOverrideCommand(ds, k, initialPose, initialEnabled));
    }

    public void poseResetCommand(String name, Pose2d initialPose) {
      String k = normalizeKey(name);
      putUnique(poseResetCommands, k, new PoseResetCommand(ds, k, initialPose));
    }

    public void goalSetpointCommand(String name, Pose2d initialPose, boolean initialEnabled) {
      String k = normalizeKey(name);
      putUnique(goalSetpointCommands, k, new GoalSetpointCommand(ds, k, initialPose, initialEnabled));
    }
  }

  private static String normalizeRoot(String root) {
    String r = root.trim();
    if (!r.startsWith("/")) r = "/" + r;
    while (r.endsWith("/")) r = r.substring(0, r.length() - 1);
    return r;
  }

  private static String normalizeKey(String key) {
    if (key == null) throw new IllegalArgumentException("key");
    String k = key.trim();
    if (k.isEmpty()) throw new IllegalArgumentException("key");
    while (k.startsWith("/")) k = k.substring(1);
    while (k.endsWith("/")) k = k.substring(0, k.length() - 1);
    if (k.isEmpty()) throw new IllegalArgumentException("key");
    return k;
  }

  private <T> NetworkTablesValue<T> own(NetworkTablesValue<T> v) {
    owned.add(v);
    return v;
  }

  private static <T> void putUnique(Map<String, T> map, String key, T value) {
    if (map.containsKey(key)) throw new IllegalStateException("Duplicate key: " + key);
    map.put(key, value);
  }

  private static <V> V require(Map<String, V> map, String key) {
    String k = normalizeKey(key);
    V v = map.get(k);
    if (v == null) throw new IllegalStateException("Unknown key: " + k);
    return v;
  }

  private static final class PoseOverrideCommand {
    private final NetworkTablesValue<Boolean> apply;
    private final NetworkTablesValue<Boolean> enabled;
    private final NetworkTablesValue<double[]> pose;

    private volatile Optional<Pose2d> pending = Optional.empty();

    private PoseOverrideCommand(
        NtRepulsorDriverStation ds, String name, Pose2d initialPose, boolean initialEnabled) {
      String base = ds.commandPath(name + "/pose_override");
      this.apply = ds.own(NetworkTablesValue.ofBoolean(ds.inst, base + "/apply", false));
      this.enabled = ds.own(NetworkTablesValue.ofBoolean(ds.inst, base + "/enabled", initialEnabled));
      this.pose =
          ds.own(
              NetworkTablesValue.ofDoubleArray(ds.inst, base + "/pose", PoseCodec.encode(initialPose)));
    }

    private void tick() {
      if (Boolean.TRUE.equals(apply.get())) {
        apply.set(false);
        apply.flush();
        if (Boolean.TRUE.equals(enabled.get())) {
          pending = Optional.of(PoseCodec.decode(pose.get()));
        }
      }
    }

    private void request(Pose2d p, boolean en) {
      pose.set(PoseCodec.encode(p));
      enabled.set(en);
      apply.set(true);
      apply.flush();
    }

    private Optional<Pose2d> consume() {
      Optional<Pose2d> out = pending;
      pending = Optional.empty();
      return out;
    }
  }

  private static final class PoseResetCommand {
    private final NetworkTablesValue<Boolean> apply;
    private final NetworkTablesValue<double[]> pose;

    private volatile Optional<Pose2d> pending = Optional.empty();

    private PoseResetCommand(NtRepulsorDriverStation ds, String name, Pose2d initialPose) {
      String base = ds.commandPath(name + "/pose_reset");
      this.apply = ds.own(NetworkTablesValue.ofBoolean(ds.inst, base + "/apply", false));
      this.pose =
          ds.own(
              NetworkTablesValue.ofDoubleArray(ds.inst, base + "/pose", PoseCodec.encode(initialPose)));
    }

    private void tick() {
      if (Boolean.TRUE.equals(apply.get())) {
        apply.set(false);
        apply.flush();
        pending = Optional.of(PoseCodec.decode(pose.get()));
      }
    }

    private void request(Pose2d p) {
      pose.set(PoseCodec.encode(p));
      apply.set(true);
      apply.flush();
    }

    private Optional<Pose2d> consume() {
      Optional<Pose2d> out = pending;
      pending = Optional.empty();
      return out;
    }
  }

  private static final class GoalSetpointCommand {
    private final NetworkTablesValue<Boolean> apply;
    private final NetworkTablesValue<Boolean> enabled;
    private final NetworkTablesValue<double[]> pose;

    private volatile Pose2d forcedPose = new Pose2d();
    private volatile boolean forcedEnabled;

    private volatile Optional<GoalSetpoint> appliedEdge = Optional.empty();

    private GoalSetpointCommand(
        NtRepulsorDriverStation ds, String name, Pose2d initialPose, boolean initialEnabled) {
      String base = ds.commandPath(name + "/goal_setpoint");
      this.apply = ds.own(NetworkTablesValue.ofBoolean(ds.inst, base + "/apply", false));
      this.enabled = ds.own(NetworkTablesValue.ofBoolean(ds.inst, base + "/enabled", initialEnabled));
      this.pose =
          ds.own(
              NetworkTablesValue.ofDoubleArray(ds.inst, base + "/pose", PoseCodec.encode(initialPose)));

      this.forcedPose = initialPose != null ? initialPose : new Pose2d();
      this.forcedEnabled = initialEnabled;
    }

    private void tick() {
      if (Boolean.TRUE.equals(apply.get())) {
        apply.set(false);
        apply.flush();

        Pose2d p = PoseCodec.decode(pose.get());
        boolean en = Boolean.TRUE.equals(enabled.get());

        forcedPose = p;
        forcedEnabled = en;

        appliedEdge = Optional.of(new GoalSetpoint(p, en));
      }
    }

    private void request(Pose2d p, boolean en) {
      pose.set(PoseCodec.encode(p));
      enabled.set(en);
      apply.set(true);
      apply.flush();
    }

    private Optional<Pose2d> forcedPose() {
      return forcedEnabled ? Optional.of(forcedPose) : Optional.empty();
    }

    private Optional<GoalSetpoint> consumeApplied() {
      Optional<GoalSetpoint> out = appliedEdge;
      appliedEdge = Optional.empty();
      return out;
    }
  }

  private static final class PoseCodec {
    private static double[] encode(Pose2d pose) {
      Pose2d p = pose != null ? pose : new Pose2d();
      return new double[] {p.getX(), p.getY(), p.getRotation().getRadians()};
    }

    private static Pose2d decode(double[] arr) {
      if (arr == null || arr.length < 3) return new Pose2d();
      return new Pose2d(arr[0], arr[1], new edu.wpi.first.math.geometry.Rotation2d(arr[2]));
    }
  }
}
