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
import java.util.function.Predicate;
import org.curtinfrc.frc2026.util.Repulsor.FieldTracker.GameElement.Alliance;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldLayoutProvider;
import org.curtinfrc.frc2026.util.Repulsor.Fields.FieldMapBuilder.CategorySpec;
import org.curtinfrc.frc2026.util.Repulsor.Fields.Reefscape2025;
import org.curtinfrc.frc2026.util.Repulsor.Setpoints.RepulsorSetpoint;

public class FieldTracker {
  private static FieldTracker instance;

  public static FieldTracker getInstance() {
    if (instance == null) {
      instance = new FieldTracker();
    }
    return instance;
  }

  public FieldTracker() {
    this(new Reefscape2025());
  }

  public FieldTracker(FieldLayoutProvider provider) {
    this.field_map = provider.build(this);
    predictor = new PredictiveFieldState();
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

  public enum GameObjectType {
    kCoral,
    kAlgae,
    kUndefined
  }

  public class GameObject {
    private final String id;
    private final GameObjectType type;

    GameObject(String id) {
      this(id, GameObjectType.kUndefined);
    }

    GameObject(String id, GameObjectType type) {
      if (type == null || type == GameObjectType.kUndefined) {
        throw new IllegalArgumentException("GameObjectType cannot be null/undefined");
      }
      this.id = id;
      this.type = type;
    }

    public String getId() {
      return id;
    }

    public GameObjectType getType() {
      return type;
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

    private GameObject[] contained;
    private Alliance alliance;
    private int maxContained;
    private GameElementModel model;
    private java.util.function.Predicate<GameObject> filter;
    private Optional<RepulsorSetpoint> relatedPoint = Optional.empty();
    private CategorySpec category;

    public GameElement(GameElementModel model) {
      this(Alliance.kBlue, 10, model, g -> true, null, CategorySpec.kScore);
    }

    public GameElement(
        Alliance alliance,
        int maxContained,
        GameElementModel model,
        java.util.function.Predicate<GameObject> filter,
        RepulsorSetpoint relatedPoint,
        CategorySpec category) {
      if (alliance == null) throw new IllegalArgumentException("Alliance cannot be null");
      if (maxContained < 0) throw new IllegalArgumentException("Max contained cannot be negative");
      this.contained = new GameObject[0];
      this.alliance = alliance;
      this.maxContained = maxContained;
      this.model = model;
      this.filter = (filter != null) ? filter : (go -> true);
      if (relatedPoint != null) this.relatedPoint = Optional.ofNullable(relatedPoint);
      this.category = category;
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
      this.alliance = alliance;
    }

    public int getMaxContained() {
      return maxContained;
    }

    public void setMaxContained(int maxContained) {
      if (maxContained < 0) throw new IllegalArgumentException("Max contained cannot be negative");
      this.maxContained = maxContained;
      if (contained.length > maxContained) {
        GameObject[] newContained = new GameObject[maxContained];
        System.arraycopy(contained, 0, newContained, 0, maxContained);
        contained = newContained;
      }
    }

    public GameObject[] getContained() {
      return contained;
    }

    public void setContained(GameObject[] contained) {
      this.contained = contained != null ? contained : new GameObject[0];
    }

    public GameObject getContained(int index) {
      if (index < 0 || index >= contained.length) {
        throw new IndexOutOfBoundsException("Index: " + index + ", Size: " + contained.length);
      }
      return contained[index];
    }

    public int getContainedCount() {
      return contained.length;
    }

    public boolean isAtCapacity() {
      return contained.length >= maxContained;
    }

    public GameElementModel getModel() {
      return model;
    }

    public Optional<RepulsorSetpoint> getRelatedPoint() {
      return relatedPoint;
    }

    public void setRelatedPoint(RepulsorSetpoint newPoint) {
      this.relatedPoint = Optional.ofNullable(newPoint);
    }
  }

  private PredictiveFieldState predictor;

  public PredictiveFieldState getPredictor() {
    return predictor;
  }

  public void updatePredictorWorld(Alliance ours) {
    List<GameElement> list = new ArrayList<>();
    Collections.addAll(list, field_map);
    predictor.setWorld(list, ours);
  }

  public GameElement[] field_map;

  public GameElement[] getFieldMap() {
    GameElement[] copy = new GameElement[field_map.length];
    System.arraycopy(field_map, 0, copy, 0, field_map.length);
    return copy;
  }

  public List<GameElement> getAvailableElements(Predicate<GameElement> pred) {
    List<GameElement> out = new ArrayList<>();
    for (GameElement e : field_map) {
      if (!e.isAtCapacity() && (pred == null || pred.test(e))) out.add(e);
    }
    return out;
  }

  public List<RepulsorSetpoint> getScoringCandidates(
      Alliance alliance, Translation2d from, CategorySpec cat) {
    if (field_map == null || field_map.length == 0) return List.of();
    List<RepulsorSetpoint> out = new ArrayList<>();
    List<GameElement> elems = new ArrayList<>();
    for (GameElement e : field_map) {
      if (e.getAlliance() == alliance
          && !e.isAtCapacity()
          && e.getRelatedPoint().isPresent()
          && e.getCategory() == cat) {
        elems.add(e);
      }
    }
    Collections.sort(
        elems,
        Comparator.comparingDouble(
            e ->
                from.getDistance(
                    new Translation2d(
                        e.getModel().getPosition().getX(), e.getModel().getPosition().getY()))));
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
      Alliance alliance, Translation2d ourPos, double ourSpeedCap, CategorySpec cat, int limit) {
    updatePredictorWorld(alliance);
    return predictor.rank(ourPos, ourSpeedCap, cat, limit);
  }

  public List<RepulsorSetpoint> getPredictedSetpoints(
      Alliance alliance, Translation2d ourPos, double ourSpeedCap, CategorySpec cat, int limit) {
    List<PredictiveFieldState.Candidate> c =
        getPredictedCandidates(alliance, ourPos, ourSpeedCap, cat, limit);
    List<RepulsorSetpoint> out = new ArrayList<>();
    for (PredictiveFieldState.Candidate k : c) out.add(k.setpoint);
    return out;
  }

  public class FieldVision {
    private class FieldVisionData {
      private final Pose3d position;
      private final GameObjectType type;

      public FieldVisionData(Pose3d position, GameObjectType type) {
        if (type == null || type == GameObjectType.kUndefined) {
          throw new IllegalArgumentException("GameObjectType cannot be null/undefined");
        }
        this.position = position;
        this.type = type;
      }

      public Pose3d getPosition() {
        return position;
      }

      public GameObjectType getType() {
        return type;
      }
    }

    private final String name;
    private final String host;
    private final NetworkTable table;

    public FieldVision(String name) {
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
      Set<String> keys = table.getKeys();
      HashMap<String, FieldVisionData> objects = new HashMap<>();

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

      for (String key : keys) {
        if (!key.startsWith("object_")) continue;

        String objectId = key.substring(7);
        String frame = table.getEntry(key + "/frame").getString("field");

        double roll = table.getEntry(key + "/roll").getDouble(0.0);
        double pitch = table.getEntry(key + "/pitch").getDouble(0.0);
        double yaw = table.getEntry(key + "/yaw").getDouble(0.0);
        Rotation3d localRot = new Rotation3d(roll, pitch, yaw);

        GameObjectType type =
            GameObjectType.valueOf(table.getEntry(key + "/type").getString("kUndefined"));

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
          double y = table.getEntry(key + "/y").getDouble(0.0);
          double z = table.getEntry(key + "/z").getDouble(0.0);
          fieldPose = new Pose3d(x, y, z, localRot);
        }

        objects.put(objectId, new FieldVisionData(fieldPose, type));
      }

      for (GameElement element : field_map) {
        if (element.getContainedCount() > 0) {
          element.setContained(new GameObject[0]);
        }
      }

      for (String objectId : objects.keySet()) {
        FieldVisionData data = objects.get(objectId);
        Pose3d position = data.getPosition();
        GameObjectType type = data.getType();

        for (GameElement element : field_map) {
          if (element.getContainedCount() < element.getMaxContained()) {
            for (PrimitiveObject primitive : element.getModel().getComposition()) {
              if (primitive.intersects(position)) {
                GameObject gameObject = new GameObject(objectId, type);
                if (!element.filter(gameObject)) break;
                GameObject[] contained = element.getContained();
                GameObject[] newContained = new GameObject[contained.length + 1];
                System.arraycopy(contained, 0, newContained, 0, contained.length);
                newContained[contained.length] = gameObject;
                element.setContained(newContained);
                break;
              }
            }
          }
        }
      }
    }
  }
}
